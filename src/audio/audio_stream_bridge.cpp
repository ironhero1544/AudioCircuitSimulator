#include "plc_emulator/audio/audio_stream_bridge.h"

#include "plc_emulator/audio/audio_circuit_runtime.h"
#include "plc_emulator/audio/spsc_audio_ring.h"
#include "plc_emulator/core/windows_power_utils.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <audioclient.h>
#include <endpointvolume.h>
#include <mmdeviceapi.h>
#include <propidl.h>
#include <windows.h>
#include <avrt.h>
#endif

namespace plc {
namespace {

std::mutex g_bridge_mutex;
std::thread g_bridge_thread;
std::atomic<bool> g_bridge_stop{false};
AudioStreamBridgeStatus g_bridge_status;
std::string g_bridge_input_id;
std::string g_bridge_output_id;
int g_bridge_buffer_ms = 80;
// The circuit runtime normally applies the configured volume range. This is
// an independent final-device safety ceiling so startup/failure/concealment
// paths can never bypass Maximum.
std::atomic<float> g_bridge_maximum_scalar{0.5f};

struct AtomicBridgeStatus {
  std::atomic<std::uint64_t> captured_frames{0};
  std::atomic<std::uint64_t> processed_frames{0};
  std::atomic<std::uint64_t> underrun_frames{0};
  std::atomic<std::uint64_t> dropped_frames{0};
  std::atomic<std::uint64_t> rtl_valid_frames{0};
  std::atomic<std::uint64_t> mna_valid_frames{0};
  std::atomic<std::uint64_t> rendered_frames{0};
  std::atomic<std::uint64_t> concealed_frames{0};
  std::atomic<std::uint64_t> capture_overflow_frames{0};
  std::atomic<std::uint64_t> capture_discontinuity_frames{0};
  std::atomic<std::uint64_t> rtl_sequence_loss_frames{0};
  std::atomic<std::uint32_t> capture_queued_frames{0};
  std::atomic<std::uint32_t> queued_frames{0};
  std::atomic<float> rtl_time_ms{0.0f};
  std::atomic<float> rtl_p99_ms{0.0f};
  std::atomic<float> mna_time_ms{0.0f};
  std::atomic<float> mna_p99_ms{0.0f};
  std::atomic<float> processing_time_ms{0.0f};
  std::atomic<float> processing_p99_ms{0.0f};
  std::atomic<float> render_time_ms{0.0f};
  std::atomic<float> render_p99_ms{0.0f};
  std::atomic<float> input_rms{0.0f};
  std::atomic<float> output_rms{0.0f};
  std::atomic<float> render_pcm_rms{0.0f};
  std::atomic<float> output_endpoint_volume{0.0f};
  std::atomic<bool> output_endpoint_muted{false};
  std::atomic<int> silence_reason{
      static_cast<int>(AudioBridgeSilenceReason::NONE)};
};

AtomicBridgeStatus g_atomic_bridge_status;

void ResetAtomicBridgeStatus() {
  g_atomic_bridge_status.captured_frames.store(0);
  g_atomic_bridge_status.processed_frames.store(0);
  g_atomic_bridge_status.underrun_frames.store(0);
  g_atomic_bridge_status.dropped_frames.store(0);
  g_atomic_bridge_status.rtl_valid_frames.store(0);
  g_atomic_bridge_status.mna_valid_frames.store(0);
  g_atomic_bridge_status.rendered_frames.store(0);
  g_atomic_bridge_status.concealed_frames.store(0);
  g_atomic_bridge_status.capture_overflow_frames.store(0);
  g_atomic_bridge_status.capture_discontinuity_frames.store(0);
  g_atomic_bridge_status.rtl_sequence_loss_frames.store(0);
  g_atomic_bridge_status.capture_queued_frames.store(0);
  g_atomic_bridge_status.queued_frames.store(0);
  g_atomic_bridge_status.rtl_time_ms.store(0.0f);
  g_atomic_bridge_status.rtl_p99_ms.store(0.0f);
  g_atomic_bridge_status.mna_time_ms.store(0.0f);
  g_atomic_bridge_status.mna_p99_ms.store(0.0f);
  g_atomic_bridge_status.processing_time_ms.store(0.0f);
  g_atomic_bridge_status.processing_p99_ms.store(0.0f);
  g_atomic_bridge_status.render_time_ms.store(0.0f);
  g_atomic_bridge_status.render_p99_ms.store(0.0f);
  g_atomic_bridge_status.input_rms.store(0.0f);
  g_atomic_bridge_status.output_rms.store(0.0f);
  g_atomic_bridge_status.render_pcm_rms.store(0.0f);
  g_atomic_bridge_status.output_endpoint_volume.store(0.0f);
  g_atomic_bridge_status.output_endpoint_muted.store(false);
  g_atomic_bridge_status.silence_reason.store(
      static_cast<int>(AudioBridgeSilenceReason::NONE));
}

class TimingWindow {
 public:
  float Add(float value) {
    values_[cursor_] = value;
    cursor_ = (cursor_ + 1U) % values_.size();
    count_ = (std::min)(count_ + 1U, values_.size());
    if ((updates_++ & 0x0fU) == 0U || count_ < 16U) {
      std::array<float, 256> ordered = values_;
      const size_t index = count_ > 0
          ? static_cast<size_t>(std::ceil(static_cast<double>(count_) * 0.99)) - 1U
          : 0U;
      std::nth_element(ordered.begin(), ordered.begin() + index,
                       ordered.begin() + count_);
      p99_ = ordered[index];
    }
    return p99_;
  }
  void Reset() { *this = {}; }

 private:
  std::array<float, 256> values_{};
  size_t cursor_ = 0;
  size_t count_ = 0;
  size_t updates_ = 0;
  float p99_ = 0.0f;
};

TimingWindow g_rtl_timing;
TimingWindow g_mna_timing;
TimingWindow g_processing_timing;
TimingWindow g_render_timing;

#ifdef _WIN32
class MmcssRegistration {
 public:
  explicit MmcssRegistration(const wchar_t* task_name) {
    DWORD task_index = 0;
    handle_ = AvSetMmThreadCharacteristicsW(task_name, &task_index);
    if (handle_) AvSetMmThreadPriority(handle_, AVRT_PRIORITY_CRITICAL);
  }
  ~MmcssRegistration() {
    if (handle_) AvRevertMmThreadCharacteristics(handle_);
  }
  MmcssRegistration(const MmcssRegistration&) = delete;
  MmcssRegistration& operator=(const MmcssRegistration&) = delete;

 private:
  HANDLE handle_ = nullptr;
};

enum class AudioEndpointRole : int {
  Console = 0,
  Multimedia = 1,
  Communications = 2,
};
struct DeviceShareMode;
struct PolicyConfig;
// This undocumented Windows COM interface must be represented as its binary
// vtable, not as a pure-virtual C++ class in this anonymous namespace. GCC can
// otherwise conclude that no external implementation of the internal-linkage
// class exists and devirtualize the call into an invalid pure-virtual target
// under -O2. The explicit COM ABI also makes the implicit `this` parameter
// and the exact Windows method order unambiguous.
struct PolicyConfigVtable {
  HRESULT(STDMETHODCALLTYPE* QueryInterface)(PolicyConfig*, REFIID, void**);
  ULONG(STDMETHODCALLTYPE* AddRef)(PolicyConfig*);
  ULONG(STDMETHODCALLTYPE* Release)(PolicyConfig*);
  HRESULT(STDMETHODCALLTYPE* GetMixFormat)(PolicyConfig*, PCWSTR,
                                           WAVEFORMATEX**);
  HRESULT(STDMETHODCALLTYPE* GetDeviceFormat)(PolicyConfig*, PCWSTR, INT,
                                              WAVEFORMATEX**);
  HRESULT(STDMETHODCALLTYPE* ResetDeviceFormat)(PolicyConfig*, PCWSTR);
  HRESULT(STDMETHODCALLTYPE* SetDeviceFormat)(PolicyConfig*, PCWSTR,
                                              WAVEFORMATEX*, WAVEFORMATEX*);
  HRESULT(STDMETHODCALLTYPE* GetProcessingPeriod)(PolicyConfig*, PCWSTR, INT,
                                                  PINT64, PINT64);
  HRESULT(STDMETHODCALLTYPE* SetProcessingPeriod)(PolicyConfig*, PCWSTR,
                                                  PINT64);
  HRESULT(STDMETHODCALLTYPE* GetShareMode)(PolicyConfig*, PCWSTR,
                                           DeviceShareMode*);
  HRESULT(STDMETHODCALLTYPE* SetShareMode)(PolicyConfig*, PCWSTR,
                                           DeviceShareMode*);
  HRESULT(STDMETHODCALLTYPE* GetPropertyValue)(PolicyConfig*, PCWSTR,
                                               const PROPERTYKEY&,
                                               PROPVARIANT*);
  HRESULT(STDMETHODCALLTYPE* SetPropertyValue)(PolicyConfig*, PCWSTR,
                                               const PROPERTYKEY&,
                                               PROPVARIANT*);
  HRESULT(STDMETHODCALLTYPE* SetDefaultEndpoint)(PolicyConfig*, PCWSTR,
                                                 AudioEndpointRole);
  HRESULT(STDMETHODCALLTYPE* SetEndpointVisibility)(PolicyConfig*, PCWSTR,
                                                    INT);
};
struct PolicyConfig {
  const PolicyConfigVtable* vtable;
};

const CLSID kPolicyConfigClient = {
    0x870af99c, 0x171d, 0x4f9e,
    {0xaf, 0x0d, 0xe6, 0x3d, 0xf4, 0x0c, 0x2b, 0xc9}};
const IID kPolicyConfigIid = {
    0xf8679f50, 0x850a, 0x41cf,
    {0x9c, 0x72, 0x43, 0x0f, 0x29, 0x02, 0x90, 0xc8}};

bool SetDefaultRenderDevice(const std::wstring& id) {
  PolicyConfig* policy = nullptr;
  if (id.empty() ||
      FAILED(CoCreateInstance(kPolicyConfigClient, nullptr, CLSCTX_ALL,
                              kPolicyConfigIid,
                              reinterpret_cast<void**>(&policy))) ||
      !policy) {
    return false;
  }
  policy->vtable->SetDefaultEndpoint(policy, id.c_str(),
                                     AudioEndpointRole::Console);
  const HRESULT result = policy->vtable->SetDefaultEndpoint(
      policy, id.c_str(), AudioEndpointRole::Multimedia);
  policy->vtable->SetDefaultEndpoint(policy, id.c_str(),
                                     AudioEndpointRole::Communications);
  policy->vtable->Release(policy);
  return SUCCEEDED(result);
}

std::wstring Utf8ToWide(const std::string& text) {
  if (text.empty()) return {};
  const int size = MultiByteToWideChar(CP_UTF8, 0, text.c_str(), -1, nullptr, 0);
  if (size <= 1) return {};
  std::wstring result(static_cast<size_t>(size), L'\0');
  MultiByteToWideChar(CP_UTF8, 0, text.c_str(), -1, result.data(), size);
  result.pop_back();
  return result;
}

template <typename T>
void ReleaseCom(T** object) {
  if (object && *object) {
    (*object)->Release();
    *object = nullptr;
  }
}

std::wstring GetDeviceId(IMMDevice* device) {
  LPWSTR id = nullptr;
  std::wstring result;
  if (device && SUCCEEDED(device->GetId(&id)) && id) result = id;
  if (id) CoTaskMemFree(id);
  return result;
}

IMMDevice* ResolveRenderDevice(IMMDeviceEnumerator* enumerator,
                               const std::string& id) {
  if (!enumerator) return nullptr;
  IMMDevice* device = nullptr;
  if (!id.empty()) {
    const std::wstring wide = Utf8ToWide(id);
    if (!wide.empty()) enumerator->GetDevice(wide.c_str(), &device);
  } else {
    enumerator->GetDefaultAudioEndpoint(eRender, eMultimedia, &device);
  }
  return device;
}

void SetBridgeStatus(bool running, const std::string& error) {
  std::lock_guard<std::mutex> lock(g_bridge_mutex);
  g_bridge_status.starting = false;
  g_bridge_status.running = running;
  g_bridge_status.error = error;
}

void SetRoutingActive(bool active) {
  std::lock_guard<std::mutex> lock(g_bridge_mutex);
  g_bridge_status.windows_routing_active = active;
}

void AddBridgeFrames(std::uint64_t captured, std::uint64_t processed) {
  g_atomic_bridge_status.captured_frames.fetch_add(
      captured, std::memory_order_relaxed);
  g_atomic_bridge_status.processed_frames.fetch_add(
      processed, std::memory_order_relaxed);
}

AudioBridgeSilenceReason ToBridgeSilenceReason(AudioSilenceReason reason) {
  switch (reason) {
    case AudioSilenceReason::RTL_WAIT:
      return AudioBridgeSilenceReason::RTL_WAIT;
    case AudioSilenceReason::MNA_FAILURE:
      return AudioBridgeSilenceReason::MNA_FAILURE;
    case AudioSilenceReason::RENDER_STARVATION:
      return AudioBridgeSilenceReason::RENDER_STARVATION;
    case AudioSilenceReason::PHYSICAL_ZERO_OUTPUT:
      return AudioBridgeSilenceReason::PHYSICAL_ZERO_OUTPUT;
    case AudioSilenceReason::NONE:
    default:
      return AudioBridgeSilenceReason::NONE;
  }
}

void AddProcessedBlock(const AudioBlockProcessResult& result,
                       std::uint64_t frames, float processing_time_ms) {
  const float rtl_p99 = g_rtl_timing.Add(result.rtl_time_ms);
  const float mna_p99 = g_mna_timing.Add(result.mna_time_ms);
  const float processing_p99 = g_processing_timing.Add(processing_time_ms);
  g_atomic_bridge_status.processed_frames.fetch_add(
      frames, std::memory_order_relaxed);
  if (result.rtl_valid) {
    g_atomic_bridge_status.rtl_valid_frames.fetch_add(
        frames, std::memory_order_relaxed);
  }
  if (result.mna_valid) {
    g_atomic_bridge_status.mna_valid_frames.fetch_add(
        frames, std::memory_order_relaxed);
  }
  if (result.concealed) {
    g_atomic_bridge_status.concealed_frames.fetch_add(
        frames, std::memory_order_relaxed);
  }
  if (result.rtl_sequence_lost) {
    g_atomic_bridge_status.rtl_sequence_loss_frames.fetch_add(
        frames, std::memory_order_relaxed);
  }
  g_atomic_bridge_status.rtl_time_ms.store(result.rtl_time_ms,
                                            std::memory_order_relaxed);
  g_atomic_bridge_status.rtl_p99_ms.store(rtl_p99,
                                           std::memory_order_relaxed);
  g_atomic_bridge_status.mna_time_ms.store(result.mna_time_ms,
                                            std::memory_order_relaxed);
  g_atomic_bridge_status.mna_p99_ms.store(mna_p99,
                                           std::memory_order_relaxed);
  g_atomic_bridge_status.processing_time_ms.store(
      processing_time_ms, std::memory_order_relaxed);
  g_atomic_bridge_status.processing_p99_ms.store(
      processing_p99, std::memory_order_relaxed);
  g_atomic_bridge_status.input_rms.store(result.input_rms,
                                          std::memory_order_relaxed);
  g_atomic_bridge_status.output_rms.store(result.output_rms,
                                           std::memory_order_relaxed);
  g_atomic_bridge_status.silence_reason.store(
      static_cast<int>(ToBridgeSilenceReason(result.silence_reason)),
      std::memory_order_relaxed);
}

void AddCaptureLoss(std::uint64_t overflow, std::uint64_t discontinuity) {
  g_atomic_bridge_status.capture_overflow_frames.fetch_add(
      overflow, std::memory_order_relaxed);
  g_atomic_bridge_status.capture_discontinuity_frames.fetch_add(
      discontinuity, std::memory_order_relaxed);
  // DATA_DISCONTINUITY marks a timing discontinuity around the packet; it
  // does not mean that every frame in the packet was discarded. Count it in
  // its dedicated diagnostic only. dropped_frames is reserved for samples
  // that this process actually failed to enqueue.
  g_atomic_bridge_status.dropped_frames.fetch_add(
      overflow, std::memory_order_relaxed);
}

void AddRenderedFrames(std::uint64_t rendered, std::uint64_t concealed,
                       float render_time_ms, float render_pcm_rms) {
  const float render_p99 = g_render_timing.Add(render_time_ms);
  g_atomic_bridge_status.rendered_frames.fetch_add(
      rendered, std::memory_order_relaxed);
  // Pipeline concealment and output starvation are different failures.
  // Keep render gaps in underrun_frames so RTL/MNA concealment remains useful.
  g_atomic_bridge_status.underrun_frames.fetch_add(
      concealed, std::memory_order_relaxed);
  g_atomic_bridge_status.render_time_ms.store(render_time_ms,
                                               std::memory_order_relaxed);
  g_atomic_bridge_status.render_p99_ms.store(render_p99,
                                              std::memory_order_relaxed);
  g_atomic_bridge_status.render_pcm_rms.store(render_pcm_rms,
                                               std::memory_order_relaxed);
  if (concealed > 0) {
    g_atomic_bridge_status.silence_reason.store(
        static_cast<int>(AudioBridgeSilenceReason::RENDER_STARVATION),
        std::memory_order_relaxed);
  }
}

void UpdateBufferStatus(std::uint32_t capture_queued_frames,
                        std::uint32_t queued_frames,
                        std::uint64_t underrun_frames,
                        std::uint64_t dropped_frames) {
  g_atomic_bridge_status.capture_queued_frames.store(
      capture_queued_frames, std::memory_order_relaxed);
  g_atomic_bridge_status.queued_frames.store(queued_frames,
                                              std::memory_order_relaxed);
  g_atomic_bridge_status.underrun_frames.fetch_add(
      underrun_frames, std::memory_order_relaxed);
  g_atomic_bridge_status.dropped_frames.fetch_add(
      dropped_frames, std::memory_order_relaxed);
}

void RunBridge(std::string input_id, std::string output_id, int buffer_ms) {
  constexpr UINT32 kSampleRate = 48000;
  constexpr UINT32 kChannels = 2;
  SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
  MmcssRegistration bridge_mmcss(L"Pro Audio");
  buffer_ms = std::clamp(buffer_ms, 10, 500);
  const REFERENCE_TIME requested_buffer_duration =
      static_cast<REFERENCE_TIME>(buffer_ms) * 10000;
  const size_t pipeline_capacity_frames =
      (std::max)(static_cast<size_t>(1),
                 static_cast<size_t>(kSampleRate) *
                     static_cast<size_t>(buffer_ms) / 1000U);
  // buffer_ms is the requested startup reservoir, not the hard queue limit.
  // Using the same value for both meant that filling the startup reservoir
  // left no room for the next capture packet, so a short scheduling spike
  // immediately appeared as thousands of dropped frames.
  constexpr size_t kProcessingBatchFrames = 480;
  // Capture needs enough headroom for occasional UI/driver scheduling stalls.
  // This is capacity only: normal latency is still controlled by the selected
  // startup reservoir. Keep the render queue smaller so it cannot accumulate
  // seconds of stale audio while the processor catches up.
  const size_t capture_capacity_frames = (std::max)(
      static_cast<size_t>(kSampleRate) * 2U,
      pipeline_capacity_frames * 4U);
  const size_t render_capacity_frames = (std::max)(
      pipeline_capacity_frames * 4U, kProcessingBatchFrames * 8U);
  const HRESULT com_result = CoInitializeEx(nullptr, COINIT_MULTITHREADED);
  IMMDeviceEnumerator* enumerator = nullptr;
  IMMDevice* input_device = nullptr;
  IMMDevice* output_device = nullptr;
  IMMDevice* previous_default_device = nullptr;
  IAudioClient* capture_client = nullptr;
  IAudioClient* render_client = nullptr;
  IAudioClient3* capture_client3 = nullptr;
  IAudioClient3* render_client3 = nullptr;
  IAudioCaptureClient* capture = nullptr;
  IAudioRenderClient* render = nullptr;
  IAudioEndpointVolume* output_endpoint_volume = nullptr;
  BOOL output_was_muted = FALSE;
  float output_original_volume = 0.0f;
  bool output_mute_changed = false;
  bool output_volume_changed = false;
  bool capture_started = false;
  bool render_started = false;
  WAVEFORMATEX format{};
  DWORD conversion_flags = 0;
  UINT32 render_buffer_frames = 0;
  UINT32 shared_period_frames = 0;
  HANDLE capture_event = nullptr;
  HANDLE render_event = nullptr;
  bool failed = false;
  bool default_device_changed = false;
  std::wstring previous_default_id;
  std::mutex pipeline_mutex;
  std::condition_variable pipeline_ready;
  audio::SpscAudioRing capture_queue(capture_capacity_frames * kChannels);
  audio::SpscAudioRing processed_queue(render_capacity_frames * kChannels);
  std::vector<float> render_input_scratch;
  std::atomic<bool> processor_stop{false};
  std::atomic<bool> engine_output_ready{false};
  std::thread processor_thread;

  auto fail = [&](const std::string& message) {
    failed = true;
    SetBridgeStatus(false, message);
  };

  if (input_id.empty()) {
    fail("Select Line 1 (Virtual Audio Cable) as Engine Sink.");
    goto cleanup;
  }

  if (FAILED(CoCreateInstance(__uuidof(MMDeviceEnumerator), nullptr,
                              CLSCTX_ALL, __uuidof(IMMDeviceEnumerator),
                              reinterpret_cast<void**>(&enumerator)))) {
    fail("Cannot create the Windows audio device enumerator.");
    goto cleanup;
  }
  input_device = ResolveRenderDevice(enumerator, input_id);
  output_device = ResolveRenderDevice(enumerator, output_id);
  if (!input_device || !output_device ||
      GetDeviceId(input_device) == GetDeviceId(output_device)) {
    fail("Engine Sink and Speaker Output must be different devices.");
    goto cleanup;
  }
  previous_default_device = ResolveRenderDevice(enumerator, {});
  previous_default_id = GetDeviceId(previous_default_device);
  if (!SetDefaultRenderDevice(GetDeviceId(input_device))) {
    fail("Cannot route Windows playback to Virtual Audio Cable.");
    goto cleanup;
  }
  default_device_changed = true;
  if (FAILED(input_device->Activate(__uuidof(IAudioClient), CLSCTX_ALL,
                                    nullptr,
                                    reinterpret_cast<void**>(&capture_client))) ||
      FAILED(output_device->Activate(__uuidof(IAudioClient), CLSCTX_ALL,
                                     nullptr,
                                     reinterpret_cast<void**>(&render_client)))) {
    fail("Cannot activate the selected VAC loopback or speaker output.");
    goto cleanup;
  }
  if (SUCCEEDED(output_device->Activate(
          __uuidof(IAudioEndpointVolume), CLSCTX_ALL, nullptr,
          reinterpret_cast<void**>(&output_endpoint_volume))) &&
      output_endpoint_volume) {
    output_endpoint_volume->GetMasterVolumeLevelScalar(
        &output_original_volume);
    output_endpoint_volume->GetMute(&output_was_muted);
    if (output_was_muted != FALSE &&
        SUCCEEDED(output_endpoint_volume->SetMute(FALSE, nullptr))) {
      output_mute_changed = true;
    }
    // Do NOT override the user's physical Windows master volume level to 100%.
    // Respect the user's configured system volume and scale within the simulation.
    BOOL current_mute = FALSE;
    float current_scalar = output_original_volume;
    output_endpoint_volume->GetMute(&current_mute);
    output_endpoint_volume->GetMasterVolumeLevelScalar(&current_scalar);
    g_atomic_bridge_status.output_endpoint_volume.store(
        current_scalar, std::memory_order_relaxed);
    g_atomic_bridge_status.output_endpoint_muted.store(
        current_mute != FALSE, std::memory_order_relaxed);
  }
  SetRoutingActive(true);
  SetSimulationRunningPowerState(true);

  format.wFormatTag = WAVE_FORMAT_IEEE_FLOAT;
  format.nChannels = static_cast<WORD>(kChannels);
  format.nSamplesPerSec = kSampleRate;
  format.wBitsPerSample = 32;
  format.nBlockAlign = static_cast<WORD>(kChannels * sizeof(float));
  format.nAvgBytesPerSec = format.nSamplesPerSec * format.nBlockAlign;

  conversion_flags = AUDCLNT_STREAMFLAGS_AUTOCONVERTPCM |
                     AUDCLNT_STREAMFLAGS_SRC_DEFAULT_QUALITY |
                     AUDCLNT_STREAMFLAGS_EVENTCALLBACK;
  capture_client->QueryInterface(__uuidof(IAudioClient3),
                                 reinterpret_cast<void**>(&capture_client3));
  render_client->QueryInterface(__uuidof(IAudioClient3),
                                reinterpret_cast<void**>(&render_client3));
  if (capture_client3) {
    UINT32 default_period = 0;
    UINT32 fundamental_period = 0;
    UINT32 minimum_period = 0;
    UINT32 maximum_period = 0;
    if (SUCCEEDED(capture_client3->GetSharedModeEnginePeriod(
            &format, &default_period, &fundamental_period, &minimum_period,
            &maximum_period))) {
      shared_period_frames = default_period;
    }
  }
  capture_event = CreateEventW(nullptr, FALSE, FALSE, nullptr);
  render_event = CreateEventW(nullptr, FALSE, FALSE, nullptr);
  if (!capture_event || !render_event) {
    fail("Cannot create WASAPI event handles.");
    goto cleanup;
  }
  if (FAILED(capture_client->Initialize(
          AUDCLNT_SHAREMODE_SHARED,
          conversion_flags | AUDCLNT_STREAMFLAGS_LOOPBACK,
          requested_buffer_duration, 0, &format, nullptr)) ||
      FAILED(render_client->Initialize(AUDCLNT_SHAREMODE_SHARED,
                                       conversion_flags,
                                       requested_buffer_duration, 0,
                                       &format, nullptr))) {
    fail("Cannot initialize 48 kHz stereo float loopback/render streams.");
    goto cleanup;
  }
  if (FAILED(capture_client->SetEventHandle(capture_event)) ||
      FAILED(render_client->SetEventHandle(render_event))) {
    fail("Cannot enable event-driven WASAPI buffering.");
    goto cleanup;
  }
  if (FAILED(capture_client->GetService(__uuidof(IAudioCaptureClient),
                                        reinterpret_cast<void**>(&capture))) ||
      FAILED(render_client->GetService(__uuidof(IAudioRenderClient),
                                       reinterpret_cast<void**>(&render)))) {
    fail("Cannot acquire the Windows audio stream services.");
    goto cleanup;
  }

  render_client->GetBufferSize(&render_buffer_frames);
  render_input_scratch.resize(
      (static_cast<size_t>(render_buffer_frames) + 64U) * kChannels);

  // Cold-start Verilator outside the sequenced realtime queue. The previous
  // one-frame ProcessAudioCircuitBlock warmup became sequence zero and made
  // the first real 480/1024-frame receive fail its size check.
  {
    std::string warmup_error;
    if (!WarmupAudioCircuitRuntime(&warmup_error)) {
      fail(warmup_error.empty() ? "Cannot warm up the Verilator audio DAC."
                                : warmup_error);
      goto cleanup;
    }
  }
  ResetAudioCircuitPipeline();

  // Capture first and hold the processed PCM until the configured amount has
  // accumulated.  Starting the renderer here used to make it consume every
  // small processed packet immediately and fill the rest of the endpoint with
  // zeroes, producing periodic holes even though the software queue never
  // overflowed.
  if (FAILED(capture_client->Start())) {
    fail("Cannot start the circuit audio capture stream.");
    goto cleanup;
  }
  capture_started = true;
  processor_thread = std::thread([&]() {
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_HIGHEST);
    MmcssRegistration processor_mmcss(L"Pro Audio");
    std::array<float, kProcessingBatchFrames * kChannels> block{};
    std::uint64_t block_sequence = 0;
    const size_t target_batch_frames =
        (std::min)(kProcessingBatchFrames, pipeline_capacity_frames);
    while (!processor_stop.load()) {
      {
        std::unique_lock<std::mutex> lock(pipeline_mutex);
        pipeline_ready.wait(lock, [&]() {
          return processor_stop.load() ||
                 capture_queue.AvailableRead() / kChannels >=
                     target_batch_frames;
        });
        if (processor_stop.load()) break;
      }
      const size_t frames = (std::min)(
          kProcessingBatchFrames, capture_queue.AvailableRead() / kChannels);
      const size_t samples = frames * kChannels;
      if (capture_queue.Read(block.data(), samples) != samples) continue;
      const auto processing_started_at = std::chrono::steady_clock::now();
      const AudioBlockProcessResult process_result =
          ProcessAudioCircuitBlockSequenced(
              block.data(), frames, kSampleRate, block_sequence++);
      const float processing_time_ms =
          std::chrono::duration<float, std::milli>(
              std::chrono::steady_clock::now() - processing_started_at)
              .count();
      const bool block_fully_initialized =
          process_result.rtl_valid && process_result.mna_valid &&
          !process_result.concealed;
      if (block_fully_initialized) {
        // Only now is the complete capture -> RTL -> MNA chain known to be
        // producing valid PCM. Until this transition the render client is not
        // started at all, so no startup transient can reach the speaker.
        engine_output_ready.store(true, std::memory_order_release);
      }
      const size_t written = engine_output_ready.load(std::memory_order_acquire)
          ? processed_queue.Write(block.data(), samples)
          : 0U;
      const std::uint64_t processed_frames = frames;
      const std::uint64_t dropped_frames =
          engine_output_ready.load(std::memory_order_relaxed)
              ? static_cast<std::uint64_t>((samples - written) / kChannels)
              : 0U;
      const std::uint32_t queued_frames = static_cast<std::uint32_t>(
          processed_queue.AvailableRead() / kChannels);
      AddProcessedBlock(process_result, processed_frames, processing_time_ms);
      const std::uint32_t capture_queued_frames =
          static_cast<std::uint32_t>(
              capture_queue.AvailableRead() / kChannels);
      UpdateBufferStatus(capture_queued_frames, queued_frames, 0,
                         dropped_frames);
    }
  });
  {
    std::lock_guard<std::mutex> lock(g_bridge_mutex);
    g_bridge_status.buffer_ms = buffer_ms;
    g_bridge_status.shared_period_frames = shared_period_frames;
  }

  {
    // The endpoint's own shared-mode buffer is commonly only about 10 ms.
    // Keep the full user-selected reservoir in the software pipeline instead
    // of silently reducing an 80/120 ms setting to the endpoint buffer size.
    const UINT32 startup_prefill_frames =
        static_cast<UINT32>(pipeline_capacity_frames);
    bool render_starved = false;
    float last_render_left = 0.0f;
    float last_render_right = 0.0f;
    size_t resume_fade_frames = 0;
    constexpr size_t kStartupFadeFrames = 2400U;  // 50 ms at 48 kHz.
    constexpr float kLimiterReleaseStep = 1.0f / 4800.0f;  // About 100 ms.
    size_t startup_fade_position = 0;
    float safety_limiter_gain = 1.0f;
    while (!g_bridge_stop.load()) {
      UINT32 packet_frames = 0;
      // RTL work now runs on the processor thread, so draining every currently
      // available VAC packet here is cheap and prevents the Windows loopback
      // endpoint itself from overflowing while a render iteration is pending.
      while (!g_bridge_stop.load() &&
             SUCCEEDED(capture->GetNextPacketSize(&packet_frames)) &&
             packet_frames > 0) {
        BYTE* bytes = nullptr;
        DWORD flags = 0;
        UINT32 frames = 0;
        const HRESULT capture_hr =
            capture->GetBuffer(&bytes, &frames, &flags, nullptr, nullptr);
        if (FAILED(capture_hr)) {
          // AUDCLNT_E_DEVICE_INVALIDATED: device was lost (sleep/wake or
          // unplugged). Do NOT permanently stop; signal cleanup and let
          // UpdateAudioStreamBridge auto-reconnect after 1 s.
          const bool device_lost =
              capture_hr == AUDCLNT_E_DEVICE_INVALIDATED ||
              capture_hr == static_cast<HRESULT>(0x88890004UL) ||
              capture_hr == static_cast<HRESULT>(0x88890006UL);
          if (!device_lost) {
            fail("Audio capture stream failed.");
          }
          g_bridge_stop.store(true);
          break;
        }
        const size_t packet_samples = static_cast<size_t>(frames) * kChannels;
        size_t written_samples = 0;
        if ((flags & AUDCLNT_BUFFERFLAGS_SILENT) == 0 && bytes) {
          written_samples = capture_queue.Write(
              reinterpret_cast<const float*>(bytes), packet_samples);
        } else {
          written_samples = capture_queue.WriteSilence(packet_samples);
        }
        capture->ReleaseBuffer(frames);
        AddBridgeFrames(frames, 0);
        const std::uint64_t discontinuity_frames =
            (flags & AUDCLNT_BUFFERFLAGS_DATA_DISCONTINUITY) != 0 ? frames : 0;
        const std::uint64_t overflow_frames = static_cast<std::uint64_t>(
            (packet_samples - written_samples) / kChannels);
        AddCaptureLoss(overflow_frames, discontinuity_frames);
        const std::uint32_t queued_frames = static_cast<std::uint32_t>(
            processed_queue.AvailableRead() / kChannels);
        pipeline_ready.notify_one();
        const std::uint32_t capture_queued_frames =
            static_cast<std::uint32_t>(
                capture_queue.AvailableRead() / kChannels);
        UpdateBufferStatus(capture_queued_frames, queued_frames, 0, 0);
      }

      UINT32 padding = 0;
      if (SUCCEEDED(render_client->GetCurrentPadding(&padding))) {
        const UINT32 available = render_buffer_frames - padding;
        const UINT32 queued_frames = static_cast<UINT32>(
            processed_queue.AvailableRead() / kChannels);
        const bool buffer_ready = queued_frames >= startup_prefill_frames;
        const bool ready_to_start =
            !render_started && buffer_ready &&
            engine_output_ready.load(std::memory_order_acquire);
        // Once started, keep the WASAPI clock running continuously. Waiting
        // for the complete 128 ms startup reservoir after every starvation
        // turned a short deadline miss into a long interval of total silence.
        const UINT32 frames = (render_started || ready_to_start) ? available
                                                                 : 0U;
        if (frames > 0) {
          const auto render_started_at = std::chrono::steady_clock::now();
          BYTE* bytes = nullptr;
          if (SUCCEEDED(render->GetBuffer(frames, &bytes)) && bytes) {
            float* output = reinterpret_cast<float*>(bytes);
            const float queue_error = startup_prefill_frames > 0
                ? (static_cast<float>(queued_frames) -
                   static_cast<float>(startup_prefill_frames)) /
                      static_cast<float>(startup_prefill_frames)
                : 0.0f;
            const double input_ratio = 1.0 + std::clamp(
                static_cast<double>(queue_error) * 0.001, -0.001, 0.001);
            const size_t desired_input_frames = (std::max)(
                size_t{1}, static_cast<size_t>(std::llround(
                               static_cast<double>(frames) * input_ratio)));
            const size_t readable_frames = (std::min)(
                desired_input_frames,
                processed_queue.AvailableRead() / kChannels);
            const size_t copied_samples = processed_queue.Read(
                render_input_scratch.data(), readable_frames * kChannels);
            const size_t input_frames = copied_samples / kChannels;
            const size_t real_output_frames = input_frames > 0
                ? (std::min)(static_cast<size_t>(frames),
                             static_cast<size_t>(std::floor(
                                 static_cast<double>(input_frames) /
                                 input_ratio)))
                : 0U;
            if (render_starved && real_output_frames > 0) {
              resume_fade_frames = 64;
            }
            for (size_t frame = 0; frame < real_output_frames; ++frame) {
              const double source_position =
                  static_cast<double>(frame) * input_ratio;
              const size_t first = (std::min)(
                  static_cast<size_t>(source_position), input_frames - 1U);
              const size_t second = (std::min)(first + 1U,
                                                input_frames - 1U);
              const float blend = static_cast<float>(
                  source_position - static_cast<double>(first));
              float left = render_input_scratch[first * 2U] +
                  (render_input_scratch[second * 2U] -
                   render_input_scratch[first * 2U]) * blend;
              float right = render_input_scratch[first * 2U + 1U] +
                  (render_input_scratch[second * 2U + 1U] -
                   render_input_scratch[first * 2U + 1U]) * blend;
              if (resume_fade_frames > 0) {
                const float fade = 1.0f -
                    static_cast<float>(resume_fade_frames) / 64.0f;
                left = last_render_left + (left - last_render_left) * fade;
                right = last_render_right + (right - last_render_right) * fade;
                --resume_fade_frames;
              }
              output[frame * 2U] = left;
              output[frame * 2U + 1U] = right;
            }
            const size_t missing_frames =
                static_cast<size_t>(frames) - real_output_frames;
            for (size_t offset = 0; offset < missing_frames; ++offset) {
              const float gain = offset < 64U
                  ? 1.0f - static_cast<float>(offset + 1U) / 64.0f
                  : 0.0f;
              output[(real_output_frames + offset) * 2U] =
                  last_render_left * gain;
              output[(real_output_frames + offset) * 2U + 1U] =
                  last_render_right * gain;
            }
            // This is the last operation before WASAPI sees the PCM. Apply a
            // linked-stereo limiter (to preserve the stereo image), sanitize
            // invalid solver output, and fade in a newly started simulation.
            // The final clamp makes Maximum an absolute invariant even if a
            // transient or an error-recovery path bypassed normal gain logic.
            const float safety_ceiling = std::clamp(
                g_bridge_maximum_scalar.load(std::memory_order_relaxed),
                0.0f, 1.0f);
            double render_energy = 0.0;
            for (size_t frame = 0; frame < static_cast<size_t>(frames);
                 ++frame) {
              float left = output[frame * kChannels];
              float right = output[frame * kChannels + 1U];
              if (!std::isfinite(left)) left = 0.0f;
              if (!std::isfinite(right)) right = 0.0f;

              const float peak = (std::max)(std::abs(left), std::abs(right));
              const float target_gain =
                  safety_ceiling > 0.0f && peak > safety_ceiling
                      ? safety_ceiling / peak
                      : (safety_ceiling > 0.0f ? 1.0f : 0.0f);
              if (target_gain < safety_limiter_gain) {
                safety_limiter_gain = target_gain;
              } else {
                safety_limiter_gain = (std::min)(
                    target_gain, safety_limiter_gain + kLimiterReleaseStep);
              }

              const float fade_t = (std::min)(
                  1.0f, static_cast<float>(startup_fade_position) /
                            static_cast<float>(kStartupFadeFrames));
              const float startup_gain = fade_t * fade_t * (3.0f - 2.0f * fade_t);
              if (startup_fade_position < kStartupFadeFrames) {
                ++startup_fade_position;
              }
              const float final_gain = safety_limiter_gain * startup_gain;
              left = std::clamp(left * final_gain, -safety_ceiling,
                                safety_ceiling);
              right = std::clamp(right * final_gain, -safety_ceiling,
                                 safety_ceiling);
              output[frame * kChannels] = left;
              output[frame * kChannels + 1U] = right;
              render_energy += static_cast<double>(left) * left;
              render_energy += static_cast<double>(right) * right;
            }
            if (frames > 0) {
              last_render_left = output[(static_cast<size_t>(frames) - 1U) *
                                        kChannels];
              last_render_right = output[(static_cast<size_t>(frames) - 1U) *
                                         kChannels + 1U];
            }
            const float render_pcm_rms = frames > 0
                ? static_cast<float>(std::sqrt(
                      render_energy /
                      static_cast<double>(frames * kChannels)))
                : 0.0f;
            const std::uint32_t remaining_frames =
                static_cast<std::uint32_t>(
                    processed_queue.AvailableRead() / kChannels);
            render->ReleaseBuffer(frames, 0);
            if (!render_started) {
              if (FAILED(render_client->Start())) {
                fail("Cannot start the circuit audio render stream.");
                g_bridge_stop.store(true);
                break;
              }
              render_started = true;
              // Keep the UI in "loading" until a valid, fully prefetched PCM
              // buffer has actually started the physical WASAPI render clock.
              SetBridgeStatus(true, {});
            }
            render_starved = missing_frames > 0;
            AddRenderedFrames(
                frames, missing_frames,
                std::chrono::duration<float, std::milli>(
                    std::chrono::steady_clock::now() - render_started_at)
                    .count(),
                render_pcm_rms);
            const std::uint32_t capture_queued_frames =
                static_cast<std::uint32_t>(
                    capture_queue.AvailableRead() / kChannels);
            UpdateBufferStatus(capture_queued_frames, remaining_frames, 0, 0);
          }
        }
      }
      HANDLE wait_handles[2] = {capture_event, render_event};
      const DWORD handle_count = render_started ? 2U : 1U;
      WaitForMultipleObjects(handle_count, wait_handles, FALSE, 50);
    }
  }

cleanup:
  if (capture_started) capture_client->Stop();
  processor_stop.store(true);
  pipeline_ready.notify_all();
  if (processor_thread.joinable()) processor_thread.join();
  if (render_started) render_client->Stop();
  ReleaseCom(&capture);
  ReleaseCom(&render);
  ReleaseCom(&capture_client3);
  ReleaseCom(&render_client3);
  ReleaseCom(&capture_client);
  ReleaseCom(&render_client);
  if (output_endpoint_volume && output_mute_changed) {
    output_endpoint_volume->SetMute(output_was_muted, nullptr);
  }
  if (output_endpoint_volume && output_volume_changed) {
    output_endpoint_volume->SetMasterVolumeLevelScalar(
        output_original_volume, nullptr);
  }
  ReleaseCom(&output_endpoint_volume);
  ReleaseCom(&input_device);
  ReleaseCom(&output_device);
  ReleaseCom(&previous_default_device);
  ReleaseCom(&enumerator);
  if (capture_event) CloseHandle(capture_event);
  if (render_event) CloseHandle(render_event);
  if (default_device_changed && !previous_default_id.empty()) {
    SetDefaultRenderDevice(previous_default_id);
  }
  SetRoutingActive(false);
  SetSimulationRunningPowerState(false);
  if (SUCCEEDED(com_result)) CoUninitialize();
  if (!failed) SetBridgeStatus(false, {});
}
#endif

void StopBridgeThread() {
  g_bridge_stop.store(true);
  if (g_bridge_thread.joinable()) g_bridge_thread.join();
  g_bridge_stop.store(false);
  ResetAudioSignalAnalysis();
}

}  // namespace

void UpdateAudioStreamBridge(bool enabled,
                             const std::string& input_device_id,
                             const std::string& output_device_id,
                             int buffer_ms,
                             float maximum_volume_percent) {
#ifdef _WIN32
  SetSimulationRunningPowerState(enabled);
  g_bridge_maximum_scalar.store(
      std::clamp(maximum_volume_percent, 0.0f, 100.0f) / 100.0f,
      std::memory_order_relaxed);
  buffer_ms = std::clamp(buffer_ms, 10, 500);
  if (!enabled) {
    StopBridgeThread();
    g_bridge_input_id.clear();
    g_bridge_output_id.clear();
    return;
  }
  static auto last_reconnect_time = std::chrono::steady_clock::now();
  const auto now = std::chrono::steady_clock::now();
  const bool thread_running =
      g_bridge_thread.joinable() && !g_bridge_stop.load();
  if (thread_running &&
      input_device_id == g_bridge_input_id &&
      output_device_id == g_bridge_output_id &&
      buffer_ms == g_bridge_buffer_ms) {
    return;
  }
  if (!thread_running && g_bridge_thread.joinable()) {
    // If the bridge stopped unexpectedly due to sleep/device wake, wait at least 1000ms before auto-restarting.
    if (std::chrono::duration_cast<std::chrono::milliseconds>(
            now - last_reconnect_time)
            .count() < 1000) {
      return;
    }
    last_reconnect_time = now;
  }
  StopBridgeThread();
  g_bridge_input_id = input_device_id;
  g_bridge_output_id = output_device_id;
  g_bridge_buffer_ms = buffer_ms;
  {
    std::lock_guard<std::mutex> lock(g_bridge_mutex);
    g_bridge_status = {};
    g_bridge_status.starting = true;
  }
  g_rtl_timing.Reset();
  g_mna_timing.Reset();
  g_processing_timing.Reset();
  g_render_timing.Reset();
  ResetAtomicBridgeStatus();
  g_bridge_thread =
      std::thread(RunBridge, input_device_id, output_device_id, buffer_ms);
#else
  (void)enabled;
  (void)input_device_id;
  (void)output_device_id;
  (void)buffer_ms;
  (void)maximum_volume_percent;
#endif
}

AudioStreamBridgeStatus GetAudioStreamBridgeStatus() {
  std::lock_guard<std::mutex> lock(g_bridge_mutex);
  AudioStreamBridgeStatus snapshot = g_bridge_status;
  snapshot.captured_frames =
      g_atomic_bridge_status.captured_frames.load(std::memory_order_relaxed);
  snapshot.processed_frames =
      g_atomic_bridge_status.processed_frames.load(std::memory_order_relaxed);
  snapshot.underrun_frames =
      g_atomic_bridge_status.underrun_frames.load(std::memory_order_relaxed);
  snapshot.dropped_frames =
      g_atomic_bridge_status.dropped_frames.load(std::memory_order_relaxed);
  snapshot.rtl_valid_frames =
      g_atomic_bridge_status.rtl_valid_frames.load(std::memory_order_relaxed);
  snapshot.mna_valid_frames =
      g_atomic_bridge_status.mna_valid_frames.load(std::memory_order_relaxed);
  snapshot.rendered_frames =
      g_atomic_bridge_status.rendered_frames.load(std::memory_order_relaxed);
  snapshot.concealed_frames =
      g_atomic_bridge_status.concealed_frames.load(std::memory_order_relaxed);
  snapshot.capture_overflow_frames =
      g_atomic_bridge_status.capture_overflow_frames.load(
          std::memory_order_relaxed);
  snapshot.capture_discontinuity_frames =
      g_atomic_bridge_status.capture_discontinuity_frames.load(
          std::memory_order_relaxed);
  snapshot.rtl_sequence_loss_frames =
      g_atomic_bridge_status.rtl_sequence_loss_frames.load(
          std::memory_order_relaxed);
  snapshot.capture_queued_frames =
      g_atomic_bridge_status.capture_queued_frames.load(
          std::memory_order_relaxed);
  snapshot.queued_frames =
      g_atomic_bridge_status.queued_frames.load(std::memory_order_relaxed);
  snapshot.rtl_time_ms =
      g_atomic_bridge_status.rtl_time_ms.load(std::memory_order_relaxed);
  snapshot.rtl_p99_ms =
      g_atomic_bridge_status.rtl_p99_ms.load(std::memory_order_relaxed);
  snapshot.mna_time_ms =
      g_atomic_bridge_status.mna_time_ms.load(std::memory_order_relaxed);
  snapshot.mna_p99_ms =
      g_atomic_bridge_status.mna_p99_ms.load(std::memory_order_relaxed);
  snapshot.processing_time_ms =
      g_atomic_bridge_status.processing_time_ms.load(
          std::memory_order_relaxed);
  snapshot.processing_p99_ms =
      g_atomic_bridge_status.processing_p99_ms.load(
          std::memory_order_relaxed);
  snapshot.render_time_ms =
      g_atomic_bridge_status.render_time_ms.load(std::memory_order_relaxed);
  snapshot.render_p99_ms =
      g_atomic_bridge_status.render_p99_ms.load(std::memory_order_relaxed);
  snapshot.input_rms =
      g_atomic_bridge_status.input_rms.load(std::memory_order_relaxed);
  snapshot.output_rms =
      g_atomic_bridge_status.output_rms.load(std::memory_order_relaxed);
  snapshot.render_pcm_rms =
      g_atomic_bridge_status.render_pcm_rms.load(std::memory_order_relaxed);
  snapshot.output_endpoint_volume =
      g_atomic_bridge_status.output_endpoint_volume.load(
          std::memory_order_relaxed);
  snapshot.output_endpoint_muted =
      g_atomic_bridge_status.output_endpoint_muted.load(
          std::memory_order_relaxed);
  snapshot.silence_reason = static_cast<AudioBridgeSilenceReason>(
      g_atomic_bridge_status.silence_reason.load(std::memory_order_relaxed));
  return snapshot;
}

void ShutdownAudioStreamBridge() { StopBridgeThread(); }

}  // namespace plc
