/*
 * Copyright (C) 2022 MediaTek Inc.
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

#ifndef INCLUDE_MTKCAM_HALIF_DEF_UITYPES_H_
#define INCLUDE_MTKCAM_HALIF_DEF_UITYPES_H_

namespace NSCam {

/**
 * Camera UI Types.
 */

/** Camera size type. */
struct MSize {
  typedef int value_type;  ///< Data type of attributes.
  value_type w = 0;  ///< Width.
  value_type h = 0;  ///< Height.

 public:  ////                Instantiation.
  /** Default constructor, set w and h to 0 */
  MSize() = default;

  /**
   * Constructs `MSize` with a given width `_w`.
   *  @param _w The given width.
   */
  explicit MSize(int _w) : w(_w) {}

  /**
   * Constructs `MSize` with given width and height `_w`, `_h`.
   *  @param _w The given width.
   *  @param _h The given height.
   */
  inline MSize(int _w, int _h) : w(_w), h(_h) {}

 public:  ////                Operations.
  /**
   * Returns the rectangle area by production of w and h.
   *  @return Rectangle area.
   */
  inline value_type size() const { return w * h; }

 public:  ////                Operators.
  /**
   * Checks for invalid size with width <= 0 or height <= 0.
   *  @return The check result, `true` indicates to invalid size.
   */
  inline bool operator!() const { return (w <= 0) || (h <= 0); }

  /**
   * Checks for equality between two sizes.
   *  @param rhs Target to compare.
   *  @return The check result, `true` if equal.
   */
  inline bool operator==(MSize const &rhs) const {
    return (w == rhs.w) && (h == rhs.h);
  }

  /**
   * Checks for inequality between two sizes.
   *  @see MSize::operator==
   */
  inline bool operator!=(MSize const &rhs) const { return !operator==(rhs); }

  /**
   * Adds a size to this size.
   *  @param rhs Target to add.
   *  @return The reference of `this`.
   */
  inline MSize &operator+=(MSize const &rhs) {
    w += rhs.w;
    h += rhs.h;
    return *this;
  }

  /**
   * Subtracts a size from this size.
   *  @param rhs Target to subtract.
   *  @return The reference of `this`.
   */
  inline MSize &operator-=(MSize const &rhs) {
    w -= rhs.w;
    h -= rhs.h;
    return *this;
  }

  /**
   * Return a MSize instance that adds `this` and `rhs`.
   *  @param rhs Target to add.
   *  @return The added MSize instance.
   */
  inline MSize operator+(MSize const &rhs) const {
    MSize const result(w + rhs.w, h + rhs.h);
    return result;
  }

  /**
   * Return a MSize instance that substracts `this` and `rhs`.
   *  @param rhs Target to substract.
   *  @return The substracted MSize instance.
   */
  inline MSize operator-(MSize const &rhs) const {
    MSize const result(w - rhs.w, h - rhs.h);
    return result;
  }

  /**
   * Return a MSize instance that multiplies `this` by a scalar.
   *  @param scalar The scalar in integer.
   *  @return The multipled MSize instance.
   */
  inline MSize operator*(value_type scalar) const {
    MSize const result(w * scalar, h * scalar);
    return result;
  }

  /**
   * Return a MSize instance that divides `this` by a scalar.
   *  @param scalar The scalar in integer.
   *  @return The divided MSize instance.
   */
  inline MSize operator/(value_type scalar) const {
    MSize const result(w / scalar, h / scalar);
    return result;
  }

  /**
   * Return a MSize instance that shifts the bits count `shift` to the right.
   *  @param shift The bit count to right shift.
   *  @return The shifted MSize instance.
   */
  inline MSize operator>>(value_type shift) const {
    MSize const result(w >> shift, h >> shift);
    return result;
  }

  /**
   * Return a MSize instance that shifts the bit count `shift` to the left.
   *  @param shift The bit count to left shift.
   *  @return The shifted MSize instance.
   */
  inline MSize operator<<(value_type shift) const {
    MSize const result(w << shift, h << shift);
    return result;
  }
};

}      // namespace NSCam
#endif  // INCLUDE_MTKCAM_HALIF_DEF_UITYPES_H_
