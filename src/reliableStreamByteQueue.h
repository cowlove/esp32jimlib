#ifndef RELIABLESTREAMBYTEQUEUE_H
#define RELIABLESTREAMBYTEQUEUE_H

#include <array>
#include <memory>
#include <new>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include <utility>

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
  uint32_t clearCount() const { return clearCount_; }
  bool empty() const { return size_ == 0; }

  void setClearThreshold(size_t clearThreshold) {
    clearThreshold_ = minSize(clearThreshold, Capacity);
    if (size_ > clearThreshold_) {
      clear();
      clearCount_++;
    }
  }

  bool append(const uint8_t *data, size_t length) {
    if (length == 0)
      return true;
    if (data == NULL || length > clearThreshold_ ||
        size_ > clearThreshold_ - length) {
      clear();
      overrunCount_++;
      clearCount_++;
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
  uint32_t clearCount_ = 0;
};

class ReliableStreamDynamicByteQueue {
public:
  explicit ReliableStreamDynamicByteQueue(size_t capacity = 0,
      size_t clearThreshold = 0) {
    if (capacity != 0)
      setCapacity(capacity, clearThreshold == 0 ? capacity : clearThreshold);
  }

  size_t capacity() const { return capacity_; }
  size_t size() const { return size_; }
  size_t clearThreshold() const { return clearThreshold_; }
  size_t highWaterMark() const { return highWaterMark_; }
  uint32_t overrunCount() const { return overrunCount_; }
  uint32_t clearCount() const { return clearCount_; }
  bool empty() const { return size_ == 0; }
  bool allocated() const { return storage_.get() != NULL; }

  bool setCapacity(size_t capacity, size_t clearThreshold) {
    if (capacity == 0 || clearThreshold == 0 || clearThreshold > capacity)
      return false;
    std::unique_ptr<uint8_t[]> newStorage(new (std::nothrow) uint8_t[capacity]);
    if (!newStorage)
      return false;

    size_t bytesToKeep = minSize(size_, clearThreshold);
    if (bytesToKeep < size_)
      clearCount_++;
    if (bytesToKeep != 0)
      copy(0, newStorage.get(), bytesToKeep);
    storage_ = std::move(newStorage);
    capacity_ = capacity;
    clearThreshold_ = clearThreshold;
    size_ = bytesToKeep;
    head_ = 0;
    tail_ = bytesToKeep % capacity_;
    return true;
  }

  bool setClearThreshold(size_t clearThreshold) {
    if (clearThreshold == 0)
      return false;
    if (clearThreshold > capacity_)
      return setCapacity(clearThreshold, clearThreshold);
    clearThreshold_ = clearThreshold;
    if (size_ > clearThreshold_) {
      clear();
      clearCount_++;
    }
    return true;
  }

  bool append(const uint8_t *data, size_t length) {
    if (length == 0)
      return true;
    if (!allocated() || data == NULL || length > clearThreshold_ ||
        size_ > clearThreshold_ - length) {
      clear();
      overrunCount_++;
      clearCount_++;
      return false;
    }

    size_t first = minSize(length, capacity_ - tail_);
    memcpy(storage_.get() + tail_, data, first);
    if (length > first)
      memcpy(storage_.get(), data + first, length - first);
    tail_ = (tail_ + length) % capacity_;
    size_ += length;
    if (size_ > highWaterMark_)
      highWaterMark_ = size_;
    return true;
  }

  bool peek(size_t offset, uint8_t &value) const {
    if (offset >= size_)
      return false;
    value = storage_[(head_ + offset) % capacity_];
    return true;
  }

  size_t find(uint8_t value, size_t start = 0) const {
    for (size_t i = start; i < size_; ++i) {
      if (storage_[(head_ + i) % capacity_] == value)
        return i;
    }
    return size_;
  }

  size_t copy(size_t offset, uint8_t *out, size_t length) const {
    if (!allocated() || out == NULL || offset >= size_)
      return 0;
    length = minSize(length, size_ - offset);
    size_t start = (head_ + offset) % capacity_;
    size_t first = minSize(length, capacity_ - start);
    memcpy(out, storage_.get() + start, first);
    if (length > first)
      memcpy(out + first, storage_.get(), length - first);
    return length;
  }

  size_t read(uint8_t *out, size_t length) {
    size_t copied = copy(0, out, length);
    consume(copied);
    return copied;
  }

  void consume(size_t length) {
    length = minSize(length, size_);
    if (capacity_ != 0)
      head_ = (head_ + length) % capacity_;
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

  std::unique_ptr<uint8_t[]> storage_;
  size_t capacity_ = 0;
  size_t head_ = 0;
  size_t tail_ = 0;
  size_t size_ = 0;
  size_t clearThreshold_ = 0;
  size_t highWaterMark_ = 0;
  uint32_t overrunCount_ = 0;
  uint32_t clearCount_ = 0;
};

#endif
