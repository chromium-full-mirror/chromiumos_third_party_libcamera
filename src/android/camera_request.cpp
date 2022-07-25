/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2019-2021, Google Inc.
 *
 * camera_request.cpp - libcamera Android Camera Request Descriptor
 */

#include "camera_request.h"

#include <libcamera/base/span.h>

#include "camera_buffer.h"

using namespace libcamera;

/*
 * \class Camera3RequestDescriptor
 *
 * A utility class that groups information about a capture request to be later
 * reused at request complete time to notify the framework.
 *
 *******************************************************************************
 * Lifetime of a Camera3RequestDescriptor tracking a capture request placed by
 * Android Framework
 *******************************************************************************
 *
 *
 *  Android Framework
 *     │
 *     │    ┌──────────────────────────────────┐
 *     │    │camera3_capture_request_t         │
 *     │    │                                  │
 *     │    │Requested output streams          │
 *     │    │  stream1   stream2   stream3 ... │
 *     │    └──────────────────────────────────┘
 *     ▼
 * ┌─────────────────────────────────────────────────────────────┐
 * │ libcamera HAL                                               │
 * ├─────────────────────────────────────────────────────────────┤
 * │  CameraDevice                                               │
 * │                                                             │
 * │  processCaptureRequest(camera3_capture_request_t request)   │
 * │                                                             │
 * │   - Create Camera3RequestDescriptor tracking this request   │
 * │   - Streams requiring post-processing are stored in the     │
 * │     pendingStreamsToProcess map                             │
 * │   - Add this Camera3RequestDescriptor to descriptors' queue │
 * │     CameraDevice::descriptors_                              │
 * │                                                             │ ┌─────────────────────────┐
 * │   - Queue the capture request to libcamera core ────────────┤►│libcamera core           │
 * │                                                             │ ├─────────────────────────┤
 * │                                                             │ │- Capture from Camera    │
 * │                                                             │ │                         │
 * │                                                             │ │- Emit                   │
 * │                                                             │ │  Camera::requestComplete│
 * │  requestCompleted(Request *request) ◄───────────────────────┼─┼────                     │
 * │                                                             │ │                         │
 * │   - Check request completion status                         │ └─────────────────────────┘
 * │                                                             │
 * │   - if (pendingStreamsToProcess > 0)                        │
 * │      Queue all entries from pendingStreamsToProcess         │
 * │    else                                   │                 │
 * │      completeDescriptor()                 │                 └──────────────────────┐
 * │                                           │                                        │
 * │                ┌──────────────────────────┴───┬──────────────────┐                 │
 * │                │                              │                  │                 │
 * │     ┌──────────▼────────────┐     ┌───────────▼─────────┐        ▼                 │
 * │     │CameraStream1          │     │CameraStream2        │      ....                │
 * │     ├┬───┬───┬──────────────┤     ├┬───┬───┬────────────┤                          │
 * │     ││   │   │              │     ││   │   │            │                          │
 * │     │▼───▼───▼──────────────┤     │▼───▼───▼────────────┤                          │
 * │     │PostProcessorWorker    │     │PostProcessorWorker  │                          │
 * │     │                       │     │                     │                          │
 * │     │ +------------------+  │     │ +------------------+│                          │
 * │     │ | PostProcessor    |  │     │ | PostProcessor    |│                          │
 * │     │ |     process()    |  │     │ |     process()    |│                          │
 * │     │ |                  |  │     │ |                  |│                          │
 * │     │ | Emit             |  │     │ | Emit             |│                          │
 * │     │ | processComplete  |  │     │ | processComplete  |│                          │
 * │     │ |                  |  │     │ |                  |│                          │
 * │     │ +--------------│---+  │     │ +--------------│---+│                          │
 * │     │                │      │     │                │    │                          │
 * │     │                │      │     │                │    │                          │
 * │     └────────────────┼──────┘     └────────────────┼────┘                          │
 * │                      │                             │                               │
 * │                      │                             │                               │
 * │                      │                             │                               │
 * │                      ▼                             ▼                               │
 * │ +---------------------------------------+     +--------------+                     │
 * │ | CameraDevice                          |     |              |                     │
 * │ |                                       |     |              |                     │
 * │ | streamProcessingComplete()            |     |              |                     │
 * │ |                                       |     |              |                     │
 * │ | - Check and set buffer status         |     |     ....     |                     │
 * │ | - Remove post+processing entry        |     |              |                     │
 * │ |   from pendingStreamsToProcess        |     |              |                     │
 * │ |                                       |     |              |                     │
 * │ | - if (pendingStreamsToProcess.empty())|     |              |                     │
 * │ |        completeDescriptor             |     |              |                     │
 * │ |                                       |     |              |                     │
 * │ +---------------------------------------+     +--------------+                     │
 * │                                                                                    │
 * └────────────────────────────────────────────────────────────────────────────────────┘
 *
 *   +-------------+
 *   |             | - PostProcessorWorker's thread
 *   |             |
 *   +-------------+
 */

Camera3RequestDescriptor::Camera3RequestDescriptor(
	Camera *camera, const camera3_capture_request_t *camera3Request)
{
	frameNumber_ = camera3Request->frame_number;

	/* Copy the camera3 request stream information for later access. */
	const Span<const camera3_stream_buffer_t> buffers{
		camera3Request->output_buffers,
		camera3Request->num_output_buffers
	};

	buffers_.reserve(buffers.size());

	for (const camera3_stream_buffer_t &buffer : buffers) {
		CameraStream *stream =
			static_cast<CameraStream *>(buffer.stream->priv);

		buffers_.emplace_back(stream, buffer, this);
	}

	/* Clone the controls associated with the camera3 request. */
	settings_ = CameraMetadata(camera3Request->settings);

	/*
	 * Create the CaptureRequest, stored as a unique_ptr<> to tie its
	 * lifetime to the descriptor.
	 */
	request_ = camera->createRequest(reinterpret_cast<uint64_t>(this));
}

Camera3RequestDescriptor::~Camera3RequestDescriptor() = default;

/**
 * \class StreamBuffer
 * \brief Group information for per-stream buffer of Camera3RequestDescriptor
 *
 * A capture request placed to the libcamera HAL can contain multiple streams.
 * Each stream will have an associated buffer to be filled. StreamBuffer
 * tracks this buffer with contextual information which aids in the stream's
 * generation. The generation of the stream will depend on its type (refer to
 * the CameraStream::Type documentation).
 *
 * \var StreamBuffer::stream
 * \brief Pointer to the corresponding CameraStream
 *
 * \var StreamBuffer::camera3Buffer
 * \brief Native handle to the buffer
 *
 * \var StreamBuffer::frameBuffer
 * \brief Encapsulate the dmabuf handle inside a libcamera::FrameBuffer for
 * direct streams
 *
 * \var StreamBuffer::fence
 * \brief Acquire fence of the buffer
 *
 * \var StreamBuffer::status
 * \brief Track the status of the buffer
 *
 * \var StreamBuffer::internalBuffer
 * \brief Pointer to a buffer internally handled by CameraStream (if any)
 *
 * \var StreamBuffer::srcBuffer
 * \brief Pointer to the source frame buffer used for post-processing
 *
 * \var StreamBuffer::dstBuffer
 * \brief Pointer to the destination frame buffer used for post-processing
 *
 * \var StreamBuffer::request
 * \brief Back pointer to the Camera3RequestDescriptor to which the StreamBuffer belongs
 */
StreamBuffer::StreamBuffer(
	CameraStream *cameraStream, const camera3_stream_buffer_t &buffer,
	Camera3RequestDescriptor *requestDescriptor)
	: stream(cameraStream), camera3Buffer(buffer.buffer),
	  fence(buffer.acquire_fence), request(requestDescriptor)
{
}

StreamBuffer::~StreamBuffer() = default;

StreamBuffer::StreamBuffer(StreamBuffer &&) = default;

StreamBuffer &StreamBuffer::operator=(StreamBuffer &&) = default;
