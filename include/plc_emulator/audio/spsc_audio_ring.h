#ifndef PLC_EMULATOR_AUDIO_SPSC_AUDIO_RING_H_
#define PLC_EMULATOR_AUDIO_SPSC_AUDIO_RING_H_

#include <algorithm>
#include <atomic>
#include <cstddef>
#include <cstring>
#include <vector>

namespace plc::audio {

// A fixed-storage single-producer/single-consumer ring. Construction may
// allocate; Read/Write/WriteSilence never allocate or lock.
class SpscAudioRing {
 public:
  explicit SpscAudioRing(size_t capacity_samples)
      : storage_(std::max<size_t>(capacity_samples, 2U), 0.0f) {}

  size_t Capacity() const { return storage_.size(); }

  size_t AvailableRead() const {
    const size_t write = write_index_.load(std::memory_order_acquire);
    const size_t read = read_index_.load(std::memory_order_acquire);
    return write - read;
  }

  size_t AvailableWrite() const { return Capacity() - AvailableRead(); }

  size_t Write(const float* source, size_t sample_count) {
    if (!source || sample_count == 0) return 0;
    const size_t read = read_index_.load(std::memory_order_acquire);
    const size_t write = write_index_.load(std::memory_order_relaxed);
    const size_t count = std::min(sample_count, Capacity() - (write - read));
    CopyInto(write, source, count);
    write_index_.store(write + count, std::memory_order_release);
    return count;
  }

  size_t WriteSilence(size_t sample_count) {
    const size_t read = read_index_.load(std::memory_order_acquire);
    const size_t write = write_index_.load(std::memory_order_relaxed);
    const size_t count = std::min(sample_count, Capacity() - (write - read));
    const size_t offset = write % Capacity();
    const size_t first = std::min(count, Capacity() - offset);
    std::fill_n(storage_.data() + offset, first, 0.0f);
    std::fill_n(storage_.data(), count - first, 0.0f);
    write_index_.store(write + count, std::memory_order_release);
    return count;
  }

  size_t Read(float* destination, size_t sample_count) {
    if (!destination || sample_count == 0) return 0;
    const size_t write = write_index_.load(std::memory_order_acquire);
    const size_t read = read_index_.load(std::memory_order_relaxed);
    const size_t count = std::min(sample_count, write - read);
    const size_t offset = read % Capacity();
    const size_t first = std::min(count, Capacity() - offset);
    std::memcpy(destination, storage_.data() + offset,
                first * sizeof(float));
    std::memcpy(destination + first, storage_.data(),
                (count - first) * sizeof(float));
    read_index_.store(read + count, std::memory_order_release);
    return count;
  }

  void Clear() {
    const size_t write = write_index_.load(std::memory_order_acquire);
    read_index_.store(write, std::memory_order_release);
  }

 private:
  void CopyInto(size_t write, const float* source, size_t count) {
    const size_t offset = write % Capacity();
    const size_t first = std::min(count, Capacity() - offset);
    std::memcpy(storage_.data() + offset, source, first * sizeof(float));
    std::memcpy(storage_.data(), source + first,
                (count - first) * sizeof(float));
  }

  std::vector<float> storage_;
  alignas(64) std::atomic<size_t> write_index_{0};
  alignas(64) std::atomic<size_t> read_index_{0};
};

}  // namespace plc::audio

#endif
