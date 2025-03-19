/*
 * Copyright (C) 2025 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <cstdint>
#include <deque>

#include "libcamera/base/log.h"
#include "libcamera/base/mutex.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(MtkISP7)

template<typename T>
class History
{
public:
	static const uint32_t kMaxSize = 32;

	History(int size = kMaxSize)
		: size_(size)
	{
	}

	void add(uint32_t id, T t)
	{
		MutexLocker locker(lock_);

		resultHistory_.push_back(std::make_pair(id, std::move(t)));
		if (resultHistory_.size() > size_)
			resultHistory_.pop_front();
	}

	T *query(uint32_t id)
	{
		MutexLocker locker(lock_);

		ASSERT(!resultHistory_.empty());

		for (auto &[resultId, result] : resultHistory_) {
			if (id == resultId)
				return &result;
		}

		LOG(MtkISP7, Error) << " Cannot find result of id " << id
				    << ". Use the latest one.";

		return &resultHistory_.back().second;
	}

	void release()
	{
		MutexLocker locker(lock_);

		resultHistory_.clear();
	}

private:
	uint32_t size_;

	Mutex lock_;
	std::deque<std::pair<uint32_t, T>> resultHistory_
		LIBCAMERA_TSA_GUARDED_BY(lock_);
};

} // namespace libcamera
