#include "plc_emulator/rtl/rtl_runtime_manager.h"
#include "plc_emulator/core/windows_power_utils.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <cctype>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <cstdint>
#include <cstring>
#include <sstream>
#include <thread>
#include <unordered_map>
#include <vector>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <avrt.h>
#endif

// 워커 응답 대기 타임아웃 (ms). 조합 회로는 수μs, 순차 회로도 수ms 이내.
// 초과 시 해당 프레임은 실패로 처리하고 다음 프레임에서 재시도합니다.
static constexpr DWORD kRtlWorkerResponseTimeoutMs = 500;
// Starting a WSL process can take several seconds when the VM is cold.  This is
// deliberately separate from the realtime request timeout: once the worker is
// ready, an audio block still has to complete quickly.
static constexpr DWORD kRtlWorkerStartupTimeoutMs = 30000;
static constexpr DWORD kRtlWorkerEfficiencyModeTimeoutMs = 1200;

namespace plc {
namespace {

#ifdef _WIN32
class AudioMmcssRegistration {
 public:
  AudioMmcssRegistration() {
    DWORD task_index = 0;
    handle_ = AvSetMmThreadCharacteristicsW(L"Pro Audio", &task_index);
    if (handle_) AvSetMmThreadPriority(handle_, AVRT_PRIORITY_CRITICAL);
  }
  ~AudioMmcssRegistration() {
    if (handle_) AvRevertMmThreadCharacteristics(handle_);
  }

 private:
  HANDLE handle_ = nullptr;
};

DWORD GetRtlWorkerResponseTimeoutMs() {
  return ShouldUseWindowsEfficiencyMode()
             ? kRtlWorkerEfficiencyModeTimeoutMs
             : kRtlWorkerResponseTimeoutMs;
}

struct AudioProtocolHeader {
  std::uint32_t magic = 0;
  std::uint16_t version = 2;
  std::uint16_t type = 0;
  std::uint32_t sequence = 0;
  std::uint32_t frames = 0;
};
static_assert(sizeof(AudioProtocolHeader) == 16);
constexpr std::uint32_t kAudioRequestMagic = 0x32524341U;
constexpr std::uint32_t kAudioResponseMagic = 0x32534341U;

bool WritePipeExact(HANDLE pipe, const void* data, size_t byte_count) {
  const auto* cursor = static_cast<const std::uint8_t*>(data);
  while (byte_count > 0) {
    DWORD written = 0;
    const DWORD chunk = static_cast<DWORD>(
        std::min(byte_count, static_cast<size_t>(0x7fffffffU)));
    if (!WriteFile(pipe, cursor, chunk, &written, nullptr) || written == 0) {
      return false;
    }
    cursor += written;
    byte_count -= written;
  }
  return true;
}

bool ReadPipeExact(HANDLE pipe, HANDLE process, void* data, size_t byte_count,
                   DWORD timeout_ms) {
  auto* cursor = static_cast<std::uint8_t*>(data);
  const DWORD started = GetTickCount();
  while (byte_count > 0) {
    if (WaitForSingleObject(process, 0) != WAIT_TIMEOUT) return false;
    DWORD available = 0;
    if (!PeekNamedPipe(pipe, nullptr, 0, nullptr, &available, nullptr)) {
      return false;
    }
    if (available == 0) {
      if (GetTickCount() - started > timeout_ms) return false;
      Sleep(1);
      continue;
    }
    DWORD bytes_read = 0;
    const DWORD wanted = static_cast<DWORD>(
        std::min({byte_count, static_cast<size_t>(available),
                  static_cast<size_t>(0x7fffffffU)}));
    if (!ReadFile(pipe, cursor, wanted, &bytes_read, nullptr) ||
        bytes_read == 0) {
      return false;
    }
    cursor += bytes_read;
    byte_count -= bytes_read;
  }
  return true;
}
#endif

float GetRtlInputHighThreshold(RtlLogicFamily family) {
  switch (family) {
    case RtlLogicFamily::CMOS_5V:
      return 3.5f;
    case RtlLogicFamily::TTL_5V:
      return 2.0f;
    case RtlLogicFamily::INDUSTRIAL_24V:
    default:
      return 12.0f;
  }
}

float GetRtlGroundMaxVoltage(RtlLogicFamily family) {
  switch (family) {
    case RtlLogicFamily::CMOS_5V:
      return 1.0f;
    case RtlLogicFamily::TTL_5V:
      return 0.8f;
    case RtlLogicFamily::INDUSTRIAL_24V:
    default:
      return 2.0f;
  }
}

bool IsVoltageHigh(float voltage, RtlLogicFamily family) {
  return voltage >= GetRtlInputHighThreshold(family);
}

bool TryGetPinVoltage(const PlacedComponent& comp,
                      const std::map<PortRef, float>& voltages,
                      const std::string& pin_name,
                      float* out_voltage) {
  if (!out_voltage || pin_name.empty()) {
    return false;
  }
  for (const auto& binding : comp.rtlPinBindings) {
    if (binding.pinName != pin_name) {
      continue;
    }
    auto it = voltages.find(std::make_pair(comp.instanceId, binding.portId));
    if (it == voltages.end()) {
      return false;
    }
    *out_voltage = it->second;
    return true;
  }
  return false;
}

bool IsRtlComponentPowered(const PlacedComponent& comp,
                          const std::map<PortRef, float>& voltages) {
  if (comp.rtlPowerPinName.empty() && comp.rtlGroundPinName.empty()) {
    return true;
  }
  bool power_ok = true;
  if (!comp.rtlPowerPinName.empty()) {
    float power_voltage = -1.0f;
    power_ok = TryGetPinVoltage(comp, voltages, comp.rtlPowerPinName,
                                &power_voltage) &&
               IsVoltageHigh(power_voltage, comp.rtlLogicFamily);
  }
  bool ground_ok = true;
  if (!comp.rtlGroundPinName.empty()) {
    float ground_voltage = -1.0f;
    ground_ok = TryGetPinVoltage(comp, voltages, comp.rtlGroundPinName,
                                 &ground_voltage) &&
                ground_voltage >= 0.0f &&
                ground_voltage <= GetRtlGroundMaxVoltage(comp.rtlLogicFamily);
  }
  return power_ok && ground_ok;
}

void SetAllRtlOutputs(const PlacedComponent& comp,
                      RtlLogicValue value,
                      std::map<int, RtlLogicValue>* output_values) {
  if (!output_values) {
    return;
  }
  output_values->clear();
  for (const auto& binding : comp.rtlPinBindings) {
    if (!binding.isInput) {
      (*output_values)[binding.portId] = value;
    }
  }
}

const char* kRtlClockLevelKey = "rtl_clock_level";
const char* kRtlResetActiveKey = "rtl_reset_active";

std::string GetBundledToolPathPrefix(const RtlToolchainStatus& status) {
#ifdef _WIN32
  if (status.mingwBinPath.empty()) {
    return "";
  }
  std::string result = status.mingwBinPath;
  // MSYS2 usr/bin: perl, python, sh 등 Verilator 런타임 의존성
  if (!status.msys2Root.empty()) {
    std::string usrBin = status.msys2Root + "\\usr\\bin";
    result += ";" + usrBin;
  }
  return result;
#else
  (void)status;
  return "";
#endif
}


}  // namespace

struct RtlRuntimeManager::WorkerProcess {
  int instanceId = -1;
  std::string moduleId;
  std::unordered_map<std::string, int> outputPortIdsByPinName;
#ifdef _WIN32
  HANDLE processHandle = nullptr;
  HANDLE threadHandle = nullptr;
  HANDLE stdinWrite = nullptr;
  HANDLE stdoutRead = nullptr;

  bool evalPending = false;
  std::string pendingLine;
  std::map<int, RtlLogicValue> lastOutputs;
  DWORD startTick = 0;
  std::uint32_t audioSequence = 0;
  int audioProtocolVersion = 1;
#endif
};

struct RtlRuntimeManager::AudioAsyncState {
  static constexpr size_t kMaximumFrames = 1024;
  static constexpr size_t kSlotCount = 6;
  enum SlotState : int { FREE = 0, PENDING = 1, PROCESSING = 2, READY = 3 };
  struct Slot {
    std::array<float, kMaximumFrames * 2> samples{};
    size_t frames = 0;
    std::uint64_t sequence = 0;
    std::uint64_t generation = 0;
    std::atomic<int> state{FREE};
  };
  std::array<Slot, kSlotCount> slots;
  std::atomic<bool> stop{false};
  std::atomic<std::uint64_t> submit_sequence{0};
  std::atomic<std::uint64_t> generation{1};
  std::atomic<bool> worker_healthy{true};
  std::thread worker;
  std::condition_variable wake;
  std::mutex wake_mutex;
  mutable std::mutex component_mutex;
  PlacedComponent component;
  bool configured = false;
  mutable std::mutex diagnostics_mutex;
  std::string diagnostics;
};

RtlRuntimeManager::RtlRuntimeManager()
    : audio_async_(std::make_unique<AudioAsyncState>()) {}

RtlRuntimeManager::~RtlRuntimeManager() {
  if (audio_async_) {
    audio_async_->stop.store(true, std::memory_order_release);
    audio_async_->wake.notify_all();
    if (audio_async_->worker.joinable()) audio_async_->worker.join();
  }
  ShutdownAll();
}

void RtlRuntimeManager::ConfigureAudioDac(const PlacedComponent& comp) {
  if (!audio_async_) return;
  {
    std::lock_guard<std::mutex> lock(audio_async_->component_mutex);
    audio_async_->component = comp;
    audio_async_->configured =
        comp.type == ComponentType::RTL_MODULE &&
        comp.rtlModuleId == "audio_shell_dac";
  }
  if (audio_async_->worker.joinable()) return;
  audio_async_->worker = std::thread([this]() {
#ifdef _WIN32
    AudioMmcssRegistration mmcss;
#endif
    AudioAsyncState& async = *audio_async_;
    while (!async.stop.load(std::memory_order_acquire)) {
      bool processed_any = false;
      for (AudioAsyncState::Slot& slot : async.slots) {
        int expected = AudioAsyncState::PENDING;
        if (!slot.state.compare_exchange_strong(
                expected, AudioAsyncState::PROCESSING,
                std::memory_order_acq_rel)) {
          continue;
        }
        processed_any = true;
        PlacedComponent component;
        bool configured = false;
        {
          std::lock_guard<std::mutex> lock(async.component_mutex);
          component = async.component;
          configured = async.configured;
        }
        std::string diagnostics;
        const bool success =
            configured &&
            ProcessAudioDacBlock(component, slot.samples.data(), slot.frames,
                                 &diagnostics);
        if (!success) {
          std::fill_n(slot.samples.data(), slot.frames * 2U, 0.0f);
          if (diagnostics.empty()) {
            diagnostics = "Asynchronous Verilator DAC evaluation failed.";
          }
        }
        {
          std::lock_guard<std::mutex> lock(async.diagnostics_mutex);
          async.diagnostics = success ? std::string{} : diagnostics;
        }
        async.worker_healthy.store(success, std::memory_order_release);
        slot.state.store(AudioAsyncState::READY, std::memory_order_release);
      }
      if (!processed_any) {
        std::unique_lock<std::mutex> lock(async.wake_mutex);
        async.wake.wait_for(lock, std::chrono::milliseconds(5), [&]() {
          if (async.stop.load(std::memory_order_acquire)) return true;
          return std::any_of(async.slots.begin(), async.slots.end(),
                             [](const AudioAsyncState::Slot& slot) {
                               return slot.state.load(
                                          std::memory_order_acquire) ==
                                      AudioAsyncState::PENDING;
                             });
        });
      }
    }
  });
}

bool RtlRuntimeManager::SubmitDacFrames(const float* interleaved_stereo,
                                        size_t frame_count) {
  if (!audio_async_) return false;
  const std::uint64_t sequence =
      audio_async_->submit_sequence.fetch_add(1, std::memory_order_relaxed);
  return SubmitDacFrames(sequence, interleaved_stereo, frame_count);
}

bool RtlRuntimeManager::SubmitDacFrames(
    std::uint64_t sequence, const float* interleaved_stereo,
    size_t frame_count) {
  if (!audio_async_ || !interleaved_stereo || frame_count == 0 ||
      frame_count > AudioAsyncState::kMaximumFrames) {
    return false;
  }
  for (AudioAsyncState::Slot& slot : audio_async_->slots) {
    int expected = AudioAsyncState::FREE;
    if (!slot.state.compare_exchange_strong(
            expected, AudioAsyncState::PROCESSING,
            std::memory_order_acq_rel)) {
      continue;
    }
    std::copy_n(interleaved_stereo, frame_count * 2U, slot.samples.data());
    slot.frames = frame_count;
    slot.sequence = sequence;
    slot.generation =
        audio_async_->generation.load(std::memory_order_acquire);
    slot.state.store(AudioAsyncState::PENDING, std::memory_order_release);
    audio_async_->wake.notify_one();
    return true;
  }
  return false;
}

bool RtlRuntimeManager::TryReceiveDacCodes(float* interleaved_stereo,
                                           size_t frame_capacity,
                                           size_t* received_frames) {
  if (!audio_async_) return false;
  AudioAsyncState::Slot* oldest = nullptr;
  for (AudioAsyncState::Slot& slot : audio_async_->slots) {
    if (slot.state.load(std::memory_order_acquire) != AudioAsyncState::READY) {
      continue;
    }
    if (!oldest || slot.sequence < oldest->sequence) oldest = &slot;
  }
  if (!oldest) return false;
  return TryReceiveDacCodes(oldest->sequence, interleaved_stereo,
                            frame_capacity, received_frames);
}

bool RtlRuntimeManager::TryReceiveDacCodes(
    std::uint64_t sequence, float* interleaved_stereo,
    size_t frame_capacity, size_t* received_frames) {
  if (received_frames) *received_frames = 0;
  if (!audio_async_ || !interleaved_stereo) return false;
  AudioAsyncState::Slot* selected = nullptr;
  const std::uint64_t current_generation =
      audio_async_->generation.load(std::memory_order_acquire);
  for (AudioAsyncState::Slot& slot : audio_async_->slots) {
    if (slot.state.load(std::memory_order_acquire) != AudioAsyncState::READY) {
      continue;
    }
    if (slot.generation != current_generation || slot.sequence < sequence) {
      slot.state.store(AudioAsyncState::FREE, std::memory_order_release);
      continue;
    }
    if (slot.sequence == sequence && slot.frames <= frame_capacity) {
      selected = &slot;
      break;
    }
  }
  if (!selected) return false;
  std::copy_n(selected->samples.data(), selected->frames * 2U,
              interleaved_stereo);
  if (received_frames) *received_frames = selected->frames;
  selected->state.store(AudioAsyncState::FREE, std::memory_order_release);
  return true;
}

void RtlRuntimeManager::ResetAudioPipeline() {
  if (!audio_async_) return;
  audio_async_->generation.fetch_add(1, std::memory_order_acq_rel);
  audio_async_->submit_sequence.store(0, std::memory_order_release);
  audio_async_->worker_healthy.store(true, std::memory_order_release);
  for (AudioAsyncState::Slot& slot : audio_async_->slots) {
    const int state = slot.state.load(std::memory_order_acquire);
    if (state != AudioAsyncState::PROCESSING) {
      slot.state.store(AudioAsyncState::FREE, std::memory_order_release);
    }
  }
}

bool RtlRuntimeManager::WarmupAudioDac(std::string* diagnostics) {
  if (!audio_async_) return false;
  PlacedComponent component;
  bool configured = false;
  {
    std::lock_guard<std::mutex> lock(audio_async_->component_mutex);
    component = audio_async_->component;
    configured = audio_async_->configured;
  }
  if (!configured) {
    if (diagnostics) *diagnostics = "Audio DAC is not configured.";
    return false;
  }
  std::array<float, 32 * 2> silence{};
  const bool ready = ProcessAudioDacBlock(
      component, silence.data(), silence.size() / 2U, diagnostics);
  ResetAudioPipeline();
  return ready;
}

std::string RtlRuntimeManager::GetAudioDacDiagnostics() const {
  if (!audio_async_) return {};
  std::lock_guard<std::mutex> lock(audio_async_->diagnostics_mutex);
  return audio_async_->diagnostics;
}

bool RtlRuntimeManager::AudioDacWorkerHealthy() const {
  return !audio_async_ ||
         audio_async_->worker_healthy.load(std::memory_order_acquire);
}

void RtlRuntimeManager::SetProjectManager(
    const RtlProjectManager* project_manager) {
  std::lock_guard<std::mutex> lock(processes_mutex_);
  project_manager_ = project_manager;
}

void RtlRuntimeManager::SyncComponentInstances(
    const std::vector<PlacedComponent>& components) {
  // The audio callback owns this mutex while a Verilator DAC block is in
  // flight.  Component cleanup is maintenance work and can safely wait for a
  // later frame; blocking the UI thread here makes a slow/cold WSL worker
  // freeze the entire application.
  std::unique_lock<std::mutex> lock(processes_mutex_, std::try_to_lock);
  if (!lock.owns_lock()) {
    return;
  }
  std::set<int> live_ids;
  for (const auto& comp : components) {
    if (comp.type == ComponentType::RTL_MODULE) {
      live_ids.insert(comp.instanceId);
    }
  }

  for (auto it = processes_.begin(); it != processes_.end();) {
    if (live_ids.count(it->first) == 0) {
#ifdef _WIN32
      if (it->second.stdinWrite) {
        const char* quit = "QUIT\n";
        DWORD written = 0;
        WriteFile(it->second.stdinWrite, quit, 5, &written, nullptr);
      }
      if (it->second.stdinWrite) {
        CloseHandle(it->second.stdinWrite);
      }
      if (it->second.stdoutRead) {
        CloseHandle(it->second.stdoutRead);
      }
      if (it->second.threadHandle) {
        CloseHandle(it->second.threadHandle);
      }
      if (it->second.processHandle) {
        WaitForSingleObject(it->second.processHandle, 50);
        CloseHandle(it->second.processHandle);
      }
#endif
      it = processes_.erase(it);
    } else {
      ++it;
    }
  }
}

std::string TrimCopy(const std::string& text) {
  size_t start = 0;
  while (start < text.size() &&
         std::isspace(static_cast<unsigned char>(text[start])) != 0) {
    ++start;
  }

  size_t end = text.size();
  while (end > start &&
         std::isspace(static_cast<unsigned char>(text[end - 1])) != 0) {
    --end;
  }

  return text.substr(start, end - start);
}

void RtlRuntimeManager::InvalidateModule(const std::string& module_id) {
  std::lock_guard<std::mutex> lock(processes_mutex_);
  for (auto it = processes_.begin(); it != processes_.end();) {
    if (it->second.moduleId == module_id) {
#ifdef _WIN32
      if (it->second.stdinWrite) {
        const char* quit = "QUIT\n";
        DWORD written = 0;
        WriteFile(it->second.stdinWrite, quit, 5, &written, nullptr);
      }
      if (it->second.stdinWrite) {
        CloseHandle(it->second.stdinWrite);
      }
      if (it->second.stdoutRead) {
        CloseHandle(it->second.stdoutRead);
      }
      if (it->second.threadHandle) {
        CloseHandle(it->second.threadHandle);
      }
      if (it->second.processHandle) {
        WaitForSingleObject(it->second.processHandle, 50);
        CloseHandle(it->second.processHandle);
      }
#endif
      it = processes_.erase(it);
    } else {
      ++it;
    }
  }
}

void RtlRuntimeManager::ShutdownAll() {
  std::lock_guard<std::mutex> lock(processes_mutex_);
#ifdef _WIN32
  for (auto& pair : processes_) {
    if (pair.second.stdinWrite) {
      const char* quit = "QUIT\n";
      DWORD written = 0;
      WriteFile(pair.second.stdinWrite, quit, 5, &written, nullptr);
      CloseHandle(pair.second.stdinWrite);
    }
    if (pair.second.stdoutRead) {
      CloseHandle(pair.second.stdoutRead);
    }
    if (pair.second.threadHandle) {
      CloseHandle(pair.second.threadHandle);
    }
    if (pair.second.processHandle) {
      WaitForSingleObject(pair.second.processHandle, 50);
      CloseHandle(pair.second.processHandle);
    }
  }
#endif
  processes_.clear();
}

bool RtlRuntimeManager::EvaluateComponent(
    const PlacedComponent& comp,
    const std::map<PortRef, float>& voltages,
    std::map<int, RtlLogicValue>* output_values,
    std::string* diagnostics) {
  std::lock_guard<std::mutex> lock(processes_mutex_);
  if (output_values) {
    output_values->clear();
  }
  if (diagnostics) {
    diagnostics->clear();
  }
  if (comp.type != ComponentType::RTL_MODULE || comp.rtlModuleId.empty()) {
    return false;
  }
  if (!IsRtlComponentPowered(comp, voltages)) {
    SetAllRtlOutputs(comp, RtlLogicValue::HIGH_Z, output_values);
    if (diagnostics) {
      diagnostics->clear();
    }
    return true;
  }
  if (!EnsureProcessStarted(comp, diagnostics)) {
    return false;
  }
  auto it = processes_.find(comp.instanceId);
  if (it == processes_.end()) {
    if (diagnostics) {
      *diagnostics = "RTL worker process was not created.";
    }
    return false;
  }
  return SendEvalRequest(&it->second, comp, voltages, output_values,
                         diagnostics);
}

bool RtlRuntimeManager::ProcessAudioDacBlock(
    const PlacedComponent& comp, float* interleaved_stereo,
    size_t frame_count, std::string* diagnostics) {
  std::lock_guard<std::mutex> lock(processes_mutex_);
  if (diagnostics) diagnostics->clear();
  if (!interleaved_stereo || frame_count == 0 ||
      comp.rtlModuleId != "audio_shell_dac") {
    if (diagnostics) *diagnostics = "No audio DAC block was provided.";
    return false;
  }
  if (!EnsureProcessStarted(comp, diagnostics)) return false;
  auto process_it = processes_.find(comp.instanceId);
  if (process_it == processes_.end()) {
    if (diagnostics) *diagnostics = "Audio DAC worker was not created.";
    return false;
  }
#ifdef _WIN32
  WorkerProcess& process = process_it->second;
  constexpr size_t kFramesPerRequest = 1024;
  if (process.audioProtocolVersion < 2) {
    // Projects created before worker protocol v2 may contain a perfectly
    // valid bundled Verilator artifact whose manifest still says protocol 1.
    // This executes on the dedicated RTL worker thread (never the WASAPI or
    // MNA realtime thread), so retaining the text transport here is a safe
    // compatibility bridge.  The next successful RTL build records v2 and
    // automatically takes the binary path below.
    for (size_t offset = 0; offset < frame_count;
         offset += kFramesPerRequest) {
      const size_t count =
          (std::min)(kFramesPerRequest, frame_count - offset);
      std::ostringstream request;
      request << "AUDIO " << count;
      for (size_t frame = 0; frame < count; ++frame) {
        for (size_t channel = 0; channel < 2; ++channel) {
          const float value = std::clamp(
              interleaved_stereo[(offset + frame) * 2U + channel],
              -1.0f, 1.0f);
          request << ' ' << static_cast<int>(std::lround(value * 32767.0f));
        }
      }
      request << '\n';
      const std::string payload = request.str();
      if (!WritePipeExact(process.stdinWrite, payload.data(),
                          payload.size())) {
        if (diagnostics) {
          *diagnostics = "Failed to send legacy PCM to the Verilator DAC.";
        }
        return false;
      }

      std::string response_line;
      const DWORD started = GetTickCount();
      bool response_complete = false;
      while (!response_complete) {
        if (WaitForSingleObject(process.processHandle, 0) != WAIT_TIMEOUT) {
          if (diagnostics) {
            *diagnostics = "Legacy Verilator DAC worker terminated.";
          }
          return false;
        }
        DWORD available = 0;
        if (!PeekNamedPipe(process.stdoutRead, nullptr, 0, nullptr,
                           &available, nullptr)) {
          if (diagnostics) {
            *diagnostics = "Legacy Verilator DAC pipe failed.";
          }
          return false;
        }
        if (available == 0) {
          if (GetTickCount() - started > GetRtlWorkerResponseTimeoutMs()) {
            if (diagnostics) {
              *diagnostics = "Legacy Verilator DAC response timed out.";
            }
            return false;
          }
          Sleep(1);
          continue;
        }
        char ch = '\0';
        DWORD bytes_read = 0;
        if (!ReadFile(process.stdoutRead, &ch, 1, &bytes_read, nullptr) ||
            bytes_read == 0) {
          if (diagnostics) {
            *diagnostics = "Legacy Verilator DAC response was truncated.";
          }
          return false;
        }
        if (ch == '\n') {
          response_complete = true;
        } else if (ch != '\r') {
          response_line.push_back(ch);
        }
      }

      std::istringstream response(TrimCopy(response_line));
      std::string tag;
      size_t returned_count = 0;
      if (!(response >> tag >> returned_count) || tag != "AUDIO_OUT" ||
          returned_count != count) {
        if (diagnostics) {
          *diagnostics = "Legacy Verilator DAC returned an invalid block.";
        }
        return false;
      }
      for (size_t frame = 0; frame < count; ++frame) {
        float left = 0.0f;
        float right = 0.0f;
        if (!(response >> left >> right)) {
          if (diagnostics) {
            *diagnostics = "Legacy Verilator DAC block was incomplete.";
          }
          return false;
        }
        interleaved_stereo[(offset + frame) * 2U] =
            std::clamp(left, -1.0f, 1.0f);
        interleaved_stereo[(offset + frame) * 2U + 1U] =
            std::clamp(right, -1.0f, 1.0f);
      }
    }
    return true;
  }
  std::array<std::int16_t, kFramesPerRequest * 2> request_pcm{};
  std::array<std::uint16_t, kFramesPerRequest * 2> response_codes{};
  for (size_t offset = 0; offset < frame_count; offset += kFramesPerRequest) {
    const size_t count =
        (std::min)(kFramesPerRequest, frame_count - offset);
    for (size_t frame = 0; frame < count; ++frame) {
      for (size_t channel = 0; channel < 2; ++channel) {
        const float value = std::clamp(
            interleaved_stereo[(offset + frame) * 2 + channel], -1.0f, 1.0f);
        request_pcm[frame * 2 + channel] = static_cast<std::int16_t>(
            std::lround(value * 32767.0f));
      }
    }
    const AudioProtocolHeader request{kAudioRequestMagic, 2U, 1U,
                                      ++process.audioSequence,
                                      static_cast<std::uint32_t>(count)};
    if (!WritePipeExact(process.stdinWrite, &request, sizeof(request)) ||
        !WritePipeExact(process.stdinWrite, request_pcm.data(),
                        count * 2U * sizeof(std::int16_t))) {
      if (diagnostics) *diagnostics = "Failed to send PCM to the Verilator DAC.";
      return false;
    }
    AudioProtocolHeader response;
    if (!ReadPipeExact(process.stdoutRead, process.processHandle, &response,
                       sizeof(response), GetRtlWorkerResponseTimeoutMs()) ||
        response.magic != kAudioResponseMagic || response.version != 2U ||
        response.type != 2U || response.sequence != request.sequence ||
        response.frames != count) {
      if (diagnostics) *diagnostics = "Verilator DAC returned an invalid audio block.";
      return false;
    }
    if (!ReadPipeExact(process.stdoutRead, process.processHandle,
                       response_codes.data(),
                       count * 2U * sizeof(std::uint16_t),
                       GetRtlWorkerResponseTimeoutMs())) {
      if (diagnostics) *diagnostics = "Verilator DAC audio block was truncated.";
      return false;
    }
    for (size_t frame = 0; frame < count; ++frame) {
      interleaved_stereo[(offset + frame) * 2] =
          (static_cast<float>(response_codes[frame * 2]) - 32768.0f) /
          32768.0f;
      interleaved_stereo[(offset + frame) * 2 + 1] =
          (static_cast<float>(response_codes[frame * 2 + 1]) - 32768.0f) /
          32768.0f;
    }
  }
  return true;
#else
  (void)comp;
  if (diagnostics) *diagnostics = "Verilator DAC audio requires Windows + WSL.";
  return false;
#endif
}

bool RtlRuntimeManager::EnsureProcessStarted(const PlacedComponent& comp,
                                             std::string* diagnostics) {
  if (processes_.count(comp.instanceId) != 0) {
    return true;
  }
  if (!project_manager_) {
    if (diagnostics) {
      *diagnostics = "RTL project manager is not attached.";
    }
    return false;
  }
  const RtlLibraryEntry* entry = project_manager_->FindEntryById(comp.rtlModuleId);
  if (!entry) {
    if (diagnostics) {
      *diagnostics = "RTL module entry was not found.";
    }
    return false;
  }
  // The launch command is entirely described by the cached artifact and WSL
  // path. Running four WSL probes here makes the first audio block stall while
  // WSL is cold, so detection belongs in the Toolchain panel, not realtime
  // worker startup.
  RtlToolchainStatus status;
  std::string command = project_manager_->GetWorkerLaunchCommand(*entry, status);
  if (command.empty()) {
    if (diagnostics) {
      *diagnostics = "RTL worker launch command is not available. Build the module first.";
    }
    return false;
  }
  const bool is_audio_dac = comp.rtlModuleId == "audio_shell_dac";
  const bool uses_binary_audio =
      is_audio_dac && entry->workerProtocolVersion >= 2;
  if (uses_binary_audio) command += " --audio-binary";

#ifdef _WIN32
  SECURITY_ATTRIBUTES sa;
  std::memset(&sa, 0, sizeof(sa));
  sa.nLength = sizeof(sa);
  sa.bInheritHandle = TRUE;

  HANDLE child_stdout_read = nullptr;
  HANDLE child_stdout_write = nullptr;
  HANDLE child_stdin_read = nullptr;
  HANDLE child_stdin_write = nullptr;
  if (!CreatePipe(&child_stdout_read, &child_stdout_write, &sa, 0)) {
    if (diagnostics) {
      *diagnostics = "Failed to create RTL worker stdout pipe.";
    }
    return false;
  }
  if (!SetHandleInformation(child_stdout_read, HANDLE_FLAG_INHERIT, 0)) {
    CloseHandle(child_stdout_read);
    CloseHandle(child_stdout_write);
    return false;
  }
  if (!CreatePipe(&child_stdin_read, &child_stdin_write, &sa, 0)) {
    CloseHandle(child_stdout_read);
    CloseHandle(child_stdout_write);
    if (diagnostics) {
      *diagnostics = "Failed to create RTL worker stdin pipe.";
    }
    return false;
  }
  if (!SetHandleInformation(child_stdin_write, HANDLE_FLAG_INHERIT, 0)) {
    CloseHandle(child_stdout_read);
    CloseHandle(child_stdout_write);
    CloseHandle(child_stdin_read);
    CloseHandle(child_stdin_write);
    return false;
  }

  STARTUPINFOA startup_info;
  std::memset(&startup_info, 0, sizeof(startup_info));
  startup_info.cb = sizeof(startup_info);
  startup_info.dwFlags = STARTF_USESHOWWINDOW | STARTF_USESTDHANDLES;
  startup_info.wShowWindow = SW_HIDE;
  startup_info.hStdInput = child_stdin_read;
  startup_info.hStdOutput = child_stdout_write;
  startup_info.hStdError = child_stdout_write;

  PROCESS_INFORMATION process_info;
  std::memset(&process_info, 0, sizeof(process_info));

  std::vector<char> command_buffer(command.begin(), command.end());
  command_buffer.push_back('\0');

  char originalPath[32767] = {0};
  DWORD originalPathLen = GetEnvironmentVariableA("PATH", originalPath,
                                                  static_cast<DWORD>(sizeof(originalPath)));
  std::string previous_path =
      (originalPathLen > 0 && originalPathLen < sizeof(originalPath))
          ? std::string(originalPath, originalPathLen)
          : std::string();
  std::string effective_path = GetBundledToolPathPrefix(status);
  if (!previous_path.empty()) {
    effective_path += ";" + previous_path;
  }
  SetEnvironmentVariableA("PATH", effective_path.c_str());
  if (!status.verilatorRoot.empty()) {
    SetEnvironmentVariableA("VERILATOR_ROOT", status.verilatorRoot.c_str());
  }

  BOOL created = CreateProcessA(nullptr, command_buffer.data(), nullptr, nullptr,
                                TRUE, CREATE_NO_WINDOW, nullptr, nullptr,
                                &startup_info, &process_info);

  if (originalPathLen > 0 && originalPathLen < sizeof(originalPath)) {
    SetEnvironmentVariableA("PATH", previous_path.c_str());
  } else {
    SetEnvironmentVariableA("PATH", nullptr);
  }
  SetEnvironmentVariableA("VERILATOR_ROOT", nullptr);
  CloseHandle(child_stdin_read);
  CloseHandle(child_stdout_write);
  if (!created) {
    CloseHandle(child_stdout_read);
    CloseHandle(child_stdin_write);
    if (diagnostics) {
      *diagnostics = "Failed to launch RTL worker process.";
    }
    return false;
  }

  ApplyWindowsEfficiencyModeCompatibility(process_info.hProcess,
                                          process_info.hThread);

  WorkerProcess process;
  process.instanceId = comp.instanceId;
  process.moduleId = comp.rtlModuleId;
  process.audioProtocolVersion = entry->workerProtocolVersion;
  for (const auto& binding : comp.rtlPinBindings) {
    if (!binding.isInput) {
      process.outputPortIdsByPinName[binding.pinName] = binding.portId;
    }
  }
  process.processHandle = process_info.hProcess;
  process.threadHandle = process_info.hThread;
  process.stdinWrite = child_stdin_write;
  process.stdoutRead = child_stdout_read;

  // Synchronize with the worker before exposing it to either the non-blocking
  // EVAL path or the realtime AUDIO path.  CreateProcess only tells us that
  // wsl.exe was created; on a cold WSL VM the Linux worker may not read stdin
  // for many seconds.  Without this handshake the first request times out and
  // every later AUDIO response becomes paired with the wrong input block.
  const char* startup_probe =
      is_audio_dac && !uses_binary_audio ? "AUDIO 0\n" : "EVAL\n";
  const DWORD startup_probe_size =
      static_cast<DWORD>(std::strlen(startup_probe));
  DWORD probe_written = 0;
  AudioProtocolHeader audio_probe{kAudioRequestMagic, 2U, 1U, 0U, 0U};
  bool worker_ready =
      uses_binary_audio
          ? WritePipeExact(process.stdinWrite, &audio_probe,
                           sizeof(audio_probe))
          : (WriteFile(process.stdinWrite, startup_probe,
                       startup_probe_size, &probe_written, nullptr) &&
             probe_written == startup_probe_size);
  if (uses_binary_audio) {
    AudioProtocolHeader response;
    worker_ready =
        worker_ready &&
        ReadPipeExact(process.stdoutRead, process.processHandle, &response,
                      sizeof(response), kRtlWorkerStartupTimeoutMs) &&
        response.magic == kAudioResponseMagic && response.version == 2U &&
        response.type == 2U && response.sequence == 0U &&
        response.frames == 0U;
  }
  std::string startup_line;
  const DWORD startup_tick = GetTickCount();
  while (worker_ready && !uses_binary_audio) {
    if (WaitForSingleObject(process.processHandle, 0) != WAIT_TIMEOUT) {
      worker_ready = false;
      break;
    }
    DWORD available = 0;
    if (!PeekNamedPipe(process.stdoutRead, nullptr, 0, nullptr, &available,
                       nullptr)) {
      worker_ready = false;
      break;
    }
    if (available == 0) {
      if (GetTickCount() - startup_tick > kRtlWorkerStartupTimeoutMs) {
        worker_ready = false;
        break;
      }
      Sleep(2);
      continue;
    }
    char ch = '\0';
    DWORD bytes_read = 0;
    if (!ReadFile(process.stdoutRead, &ch, 1, &bytes_read, nullptr) ||
        bytes_read == 0) {
      worker_ready = false;
      break;
    }
    if (ch == '\n') {
      const std::string line = TrimCopy(startup_line);
      startup_line.clear();
      const bool valid_response =
          is_audio_dac ? line.rfind("AUDIO_OUT 0", 0) == 0
                       : line.rfind("OUT", 0) == 0;
      if (valid_response) {
        break;
      }
      // stderr is intentionally connected to the same pipe.  Ignore launcher
      // chatter and keep reading until the probe's OUT response arrives.
      continue;
    }
    if (ch != '\r') startup_line.push_back(ch);
  }
  if (!worker_ready) {
    TerminateProcess(process.processHandle, 1);
    CloseHandle(process.stdinWrite);
    CloseHandle(process.stdoutRead);
    CloseHandle(process.threadHandle);
    CloseHandle(process.processHandle);
    if (diagnostics) {
      *diagnostics = "RTL worker did not become ready within 30 seconds.";
    }
    return false;
  }
  processes_.emplace(comp.instanceId, process);
  return true;
#else
  (void)comp;
  (void)command;
  if (diagnostics) {
    *diagnostics = "RTL runtime requires native Windows.";
  }
  return false;
#endif
}

bool RtlRuntimeManager::SendEvalRequest(
    WorkerProcess* process,
    const PlacedComponent& comp,
    const std::map<PortRef, float>& voltages,
    std::map<int, RtlLogicValue>* output_values,
    std::string* diagnostics) {
  if (!process) {
    return false;
  }
#ifdef _WIN32
  ApplyWindowsEfficiencyModeCompatibility(process->processHandle,
                                          process->threadHandle);
  if (!process->evalPending) {
    std::string command = BuildEvalCommand(comp, voltages);
    command.push_back('\n');
    DWORD written = 0;
    if (!WriteFile(process->stdinWrite, command.data(),
                   static_cast<DWORD>(command.size()), &written, nullptr)) {
      if (diagnostics) {
        *diagnostics = "Failed to send input values to the RTL worker.";
      }
      return false;
    }
    process->evalPending = true;
    process->startTick = GetTickCount();
    process->pendingLine.clear();
  }

  char ch = '\0';
  DWORD bytes_read = 0;
  while (true) {
    if (WaitForSingleObject(process->processHandle, 0) != WAIT_TIMEOUT) {
      if (diagnostics) {
        *diagnostics = "RTL worker terminated unexpectedly.";
      }
      return false;
    }
    DWORD available = 0;
    if (!PeekNamedPipe(process->stdoutRead, nullptr, 0, nullptr, &available,
                       nullptr)) {
      if (diagnostics) {
        *diagnostics = "RTL worker pipe error.";
      }
      return false;
    }
    if (available == 0) {
      if (GetTickCount() - process->startTick > GetRtlWorkerResponseTimeoutMs()) {
        if (diagnostics) {
          *diagnostics = "RTL worker response timed out.";
        }
        process->evalPending = false;
        return false;
      }
      if (output_values) {
        *output_values = process->lastOutputs;
      }
      return true; // Non-blocking exit
    }
    if (!ReadFile(process->stdoutRead, &ch, 1, &bytes_read, nullptr) ||
        bytes_read == 0) {
      if (diagnostics) {
        *diagnostics = "RTL worker terminated unexpectedly.";
      }
      return false;
    }
    if (ch == '\n') {
      process->evalPending = false;
      break;
    }
    if (ch != '\r') {
      process->pendingLine.push_back(ch);
    }
  }

  std::string line = process->pendingLine;
  line = TrimCopy(line);
  if (line.rfind("OUT", 0) != 0) {
    if (diagnostics) {
      *diagnostics = line.empty() ? "RTL worker returned no output."
                                  : ("RTL worker error: " + line);
    }
    return false;
  }

  std::map<int, RtlLogicValue> newOutputs;
  if (output_values || true) {
    // Bounds check: ensure line has at least 3 characters before calling substr(3)
    std::istringstream stream(line.size() >= 3 ? line.substr(3) : "");
    std::string token;
    while (stream >> token) {
      size_t eq = token.find('=');
      if (eq == std::string::npos) {
        continue;
      }
      std::string pin_name = token.substr(0, eq);
      std::string value_text = token.substr(eq + 1);
      RtlLogicValue value = RtlLogicValue::UNKNOWN;
      if (value_text == "0") {
        value = RtlLogicValue::ZERO;
      } else if (value_text == "1") {
        value = RtlLogicValue::ONE;
      } else if (value_text == "Z") {
        value = RtlLogicValue::HIGH_Z;
      }
      auto port_it = process->outputPortIdsByPinName.find(pin_name);
      if (port_it != process->outputPortIdsByPinName.end()) {
        newOutputs[port_it->second] = value;
        if (output_values) {
          (*output_values)[port_it->second] = value;
        }
      }
    }
  }
  process->lastOutputs = newOutputs;
  return true;
#else
  (void)comp;
  (void)voltages;
  (void)output_values;
  if (diagnostics) {
    *diagnostics = "RTL runtime requires native Windows.";
  }
  return false;
#endif
}

std::string RtlRuntimeManager::BuildEvalCommand(
    const PlacedComponent& comp,
    const std::map<PortRef, float>& voltages) {
  std::ostringstream command;
  command << "EVAL";
  for (const auto& binding : comp.rtlPinBindings) {
    if (!binding.isInput) {
      continue;
    }
    int bit_value = 0;
    if (comp.rtlUseInternalClock && binding.pinName == comp.rtlClockPinName) {
      auto level_it = comp.internalStates.find(kRtlClockLevelKey);
      bit_value =
          (level_it != comp.internalStates.end() && level_it->second > 0.5f)
              ? 1
              : 0;
    } else if (comp.rtlUseStartupReset &&
               binding.pinName == comp.rtlResetPinName) {
      auto reset_it = comp.internalStates.find(kRtlResetActiveKey);
      bool active =
          reset_it != comp.internalStates.end() && reset_it->second > 0.5f;
      bit_value = active ? (comp.rtlResetActiveLow ? 0 : 1)
                         : (comp.rtlResetActiveLow ? 1 : 0);
    } else {
      PortRef port_ref = std::make_pair(comp.instanceId, binding.portId);
      float voltage = -1.0f;
      auto it = voltages.find(port_ref);
      if (it != voltages.end()) {
        voltage = it->second;
      }
      bit_value = IsVoltageHigh(voltage, comp.rtlLogicFamily) ? 1 : 0;
    }
    command << " " << binding.pinName << "=" << bit_value;
  }
  return command.str();
}

}  // namespace plc
