#pragma once

#include <cstdint>
#include <type_traits>

#include "component/container/spsc_ringbuf.h"

namespace mr::comp {

class SpscRingBufView final {
 public:
  SpscRingBufView() = default;

  SpscRingBufView(uint8_t* storage, uint16_t size) {
    (void)Init(storage, size);
  }

  int8_t Init(uint8_t* storage, uint16_t size) {
    return SpscRingBuf_Init(&rb_, storage, size);
  }

  uint16_t Push(const uint8_t* data, uint16_t len) {
    return SpscRingBuf_Put(&rb_, data, len);
  }

  bool TryPush(const uint8_t* data, uint16_t len) {
    if (data == nullptr || Space() < len) {
      return false;
    }
    return Push(data, len) == len;
  }

  uint16_t Pop(uint8_t* dst, uint16_t len) {
    return SpscRingBuf_Get(&rb_, dst, len);
  }

  bool TryPop(uint8_t* dst, uint16_t len) {
    if (dst == nullptr || Size() < len) {
      return false;
    }
    return Pop(dst, len) == len;
  }

  uint16_t Peek(uint8_t* dst, uint16_t len) const {
    return SpscRingBuf_Peek(&rb_, dst, len);
  }

  uint16_t Drop(uint16_t len) {
    return SpscRingBuf_Drop(&rb_, len);
  }

  uint16_t Size() const {
    return SpscRingBuf_Size(&rb_);
  }

  uint16_t Space() const {
    return SpscRingBuf_Space(&rb_);
  }

  uint16_t Capacity() const {
    return (rb_.size > 0u) ? static_cast<uint16_t>(rb_.size - 1u) : 0u;
  }

  bool Empty() const {
    return SpscRingBuf_IsEmpty(&rb_);
  }

  void Reset() {
    SpscRingBuf_Reset(&rb_);
  }

  template <typename T>
  bool TryPushObject(const T& value) {
    static_assert(std::is_trivially_copyable<T>::value,
                  "SpscRingBufView::TryPushObject requires a trivially copyable type");
    return TryPush(reinterpret_cast<const uint8_t*>(&value),
                   static_cast<uint16_t>(sizeof(T)));
  }

  template <typename T>
  bool TryPopObject(T* out) {
    static_assert(std::is_trivially_copyable<T>::value,
                  "SpscRingBufView::TryPopObject requires a trivially copyable type");
    return TryPop(reinterpret_cast<uint8_t*>(out),
                  static_cast<uint16_t>(sizeof(T)));
  }

  template <typename T>
  bool TryPeekObject(T* out) const {
    static_assert(std::is_trivially_copyable<T>::value,
                  "SpscRingBufView::TryPeekObject requires a trivially copyable type");
    if (out == nullptr || Size() < sizeof(T)) {
      return false;
    }
    return Peek(reinterpret_cast<uint8_t*>(out),
                static_cast<uint16_t>(sizeof(T))) == sizeof(T);
  }

  SpscRingBuf_t* raw() {
    return &rb_;
  }

  const SpscRingBuf_t* raw() const {
    return &rb_;
  }

 private:
  SpscRingBuf_t rb_{};
};

template <uint16_t StorageSize>
class StaticSpscRingBuf final {
 public:
  static_assert(StorageSize >= 2u,
                "StaticSpscRingBuf storage must be at least 2 bytes");

  StaticSpscRingBuf() {
    (void)view_.Init(storage_, StorageSize);
  }

  uint16_t Push(const uint8_t* data, uint16_t len) {
    return view_.Push(data, len);
  }

  bool TryPush(const uint8_t* data, uint16_t len) {
    return view_.TryPush(data, len);
  }

  uint16_t Pop(uint8_t* dst, uint16_t len) {
    return view_.Pop(dst, len);
  }

  bool TryPop(uint8_t* dst, uint16_t len) {
    return view_.TryPop(dst, len);
  }

  uint16_t Peek(uint8_t* dst, uint16_t len) const {
    return view_.Peek(dst, len);
  }

  uint16_t Drop(uint16_t len) {
    return view_.Drop(len);
  }

  uint16_t Size() const {
    return view_.Size();
  }

  uint16_t Space() const {
    return view_.Space();
  }

  uint16_t Capacity() const {
    return view_.Capacity();
  }

  bool Empty() const {
    return view_.Empty();
  }

  void Reset() {
    view_.Reset();
  }

  template <typename T>
  bool TryPushObject(const T& value) {
    return view_.TryPushObject(value);
  }

  template <typename T>
  bool TryPopObject(T* out) {
    return view_.TryPopObject(out);
  }

  template <typename T>
  bool TryPeekObject(T* out) const {
    return view_.TryPeekObject(out);
  }

  SpscRingBufView& view() {
    return view_;
  }

  const SpscRingBufView& view() const {
    return view_;
  }

  SpscRingBuf_t* raw() {
    return view_.raw();
  }

  const SpscRingBuf_t* raw() const {
    return view_.raw();
  }

 private:
  uint8_t storage_[StorageSize]{};
  SpscRingBufView view_;
};

}  // namespace mr::comp
