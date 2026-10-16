#ifndef RELIABLESTREAMBYTEQUEUE_H
#define RELIABLESTREAMBYTEQUEUE_H

#include <array>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

template<size_t Capacity>
class ReliableStreamByteQueue {
  static_assert(Capacity > 0, "byte queue capacity must be nonzero");
public:
  explicit ReliableStreamByteQueue(size_t clearThreshold = Capacity)
      : clearThreshold_(minSize(clearThreshold, Capacity)) {}

  static constexpr size_t capacity() { return Capacity; }
  size_t size() const { return size_; }
  size_t clearThreshold() const { return clearThreshold_; }
  size_t highWaterMark() const { return highWaterMark_; }
  uint32_t overrunCount() const { return overrunCount_; }
  bool empty() const { return size_ == 0; }

  void setClearThreshold(size_t clearThreshold) {
    clearThreshold_ = minSize(clearThreshold, Capacity);
    if (size_ > clearThreshold_)
      clear();
  }

  bool append(const uint8_t *data, size_t length) {
    if (length == 0)
      return true;
    if (data == NULL || length > clearThreshold_ ||
        size_ > clearThreshold_ - length) {
      clear();
      overrunCount_++;
      return false;
    }

    size_t first = minSize(length, Capacity - tail_);
    memcpy(storage_.data() + tail_, data, first);
    if (length > first)
      memcpy(storage_.data(), data + first, length - first);
    tail_ = (tail_ + length) % Capacity;
    size_ += length;
    if (size_ > highWaterMark_)
      highWaterMark_ = size_;
    return true;
  }

  bool peek(size_t offset, uint8_t &value) const {
    if (offset >= size_)
      return false;
    value = storage_[(head_ + offset) % Capacity];
    return true;
  }

  size_t find(uint8_t value, size_t start = 0) const {
    for (size_t i = start; i < size_; ++i) {
      if (storage_[(head_ + i) % Capacity] == value)
        return i;
    }
    return size_;
  }

  size_t copy(size_t offset, uint8_t *out, size_t length) const {
    if (out == NULL || offset >= size_)
      return 0;
    length = minSize(length, size_ - offset);
    size_t start = (head_ + offset) % Capacity;
    size_t first = minSize(length, Capacity - start);
    memcpy(out, storage_.data() + start, first);
    if (length > first)
      memcpy(out + first, storage_.data(), length - first);
    return length;
  }

  size_t read(uint8_t *out, size_t length) {
    size_t copied = copy(0, out, length);
    consume(copied);
    return copied;
  }

  void consume(size_t length) {
    length = minSize(length, size_);
    head_ = (head_ + length) % Capacity;
    size_ -= length;
    if (size_ == 0)
      head_ = tail_;
  }

  void clear() {
    head_ = tail_;
    size_ = 0;
  }

private:
  static size_t minSize(size_t left, size_t right) {
    return left < right ? left : right;
  }

  std::array<uint8_t, Capacity> storage_{};
  size_t head_ = 0;
  size_t tail_ = 0;
  size_t size_ = 0;
  size_t clearThreshold_;
  size_t highWaterMark_ = 0;
  uint32_t overrunCount_ = 0;
};

#endif
