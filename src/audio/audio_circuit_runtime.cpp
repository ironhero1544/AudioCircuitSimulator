#include "plc_emulator/audio/audio_circuit_runtime.h"

#include "plc_emulator/audio/compiled_audio_circuit.h"
#include "plc_emulator/audio/spsc_audio_ring.h"
#include "plc_emulator/components/state_keys.h"
#include "plc_emulator/rtl/rtl_runtime_manager.h"

#include <algorithm>
#include <array>
#include <atomic>
#include <bit>
#include <cctype>
#include <cmath>
#include <complex>
#include <condition_variable>
#include <chrono>
#include <cstdlib>
#include <cstdint>
#include <cwchar>
#include <limits>
#include <map>
#include <memory>
#include <mutex>
#include <queue>
#include <set>
#include <span>
#include <sstream>
#include <tuple>
#include <thread>
#include <utility>

#ifdef _WIN32
#include <windows.h>
#include <endpointvolume.h>
#include <mmdeviceapi.h>
#include <propidl.h>
#endif

namespace plc {
namespace {

using PortNode = std::pair<int, int>;
constexpr float kSilenceDb = -120.0f;
constexpr float kPi = 3.14159265358979323846f;

struct Edge {
  PortNode destination;
  const PlacedComponent* through_component = nullptr;
};

enum class SignalDomain {
  DIGITAL = 0,
  ANALOG = 1,
};

struct PathState {
  PortNode node;
  SignalDomain domain = SignalDomain::DIGITAL;
  std::array<float, 10> magnitude{};
  float noise_floor_db = kSilenceDb;
  float thd_percent = 0.0f;
  bool has_powered_amplifier = false;
  bool has_dc_blocking_capacitor = false;
  bool has_emitter_ballast = false;
  bool has_amplifier_bias_fault = false;
  bool has_amplifier_thermal_tracking_fault = false;
  bool has_amplifier_bandwidth_limit = false;
  float amplifier_bias_error_volts = 0.0f;
  float specialized_distortion_percent = 0.0f;
  float amplifier_supply_voltage = 0.0f;
  float amplifier_current_limit_amps = 0.0f;
  float maximum_speaker_peak_voltage =
      (std::numeric_limits<float>::max)();
  std::vector<AudioFilterStage> filter_stages;
  std::set<std::tuple<int, int, int>> visited;
};

struct PassiveFilterState {
  float x1 = 0.0f;
  float x2 = 0.0f;
  float y1 = 0.0f;
  float y2 = 0.0f;
  unsigned int coefficient_sample_rate = 0;
  float coefficient_cutoff_hz = 0.0f;
  float coefficient_gain_db = 0.0f;
  float coefficient_q = 0.0f;
  float b0 = 1.0f;
  float b1 = 0.0f;
  float b2 = 0.0f;
  float a1 = 0.0f;
  float a2 = 0.0f;
  float first_order_alpha = 0.0f;
};

struct PitchShiftState {
  std::vector<float> delay;
  std::vector<float> window;
  size_t write_index = 0;
  float phase = 0.0f;
  float active_ratio = 1.0f;
};

struct AmplifierPowerInfo {
  bool powered = false;
  float supply_voltage = 0.0f;
  float current_limit_amps = 0.0f;
  int bias_diode_count = 0;
  int thermally_coupled_bias_diode_count = 0;
  float bias_voltage = 0.0f;
  bool emitter_ballast_present = false;
  float emitter_ballast_ohms = 0.0f;
};

struct AudioTopologyIndex {
  std::map<int, const PlacedComponent*> component_by_id;
  std::multimap<PortNode, const Wire*> wires_by_port;
};

struct DacBusResponse {
  std::uint16_t connected_mask = 0;
  std::array<float, 16> bit_weights{};
  size_t connected_bits = 0;
  bool complete = false;
};

AudioCircuitStatus g_status;
std::mutex g_status_mutex;
AudioSignalAnalysis g_signal_analysis;
std::mutex g_signal_analysis_mutex;
constexpr size_t kAnalysisWindowFrames = 8192;
std::array<float, kAnalysisWindowFrames * 2> g_analysis_sample_ring{};
size_t g_analysis_ring_write = 0;
size_t g_analysis_ring_size = 0;
std::uint32_t g_analysis_update_counter = 0;
bool g_analysis_has_snapshot = false;
audio::SpscAudioRing g_analysis_queue(kAnalysisWindowFrames * 16U);
std::atomic<bool> g_analysis_worker_stop{false};
std::atomic<bool> g_analysis_worker_started{false};
std::thread g_analysis_worker;
std::condition_variable g_analysis_worker_wake;
std::mutex g_analysis_worker_wake_mutex;
std::array<std::vector<PassiveFilterState>, 2> g_passive_filter_states;
std::array<std::vector<AudioFilterStage>, 2> g_active_filter_stages;
constexpr size_t kMaximumAcousticFilterStages = 8;
std::array<std::array<AudioFilterStage, kMaximumAcousticFilterStages>, 2>
    g_mna_acoustic_filter_stages{};
std::array<std::array<PassiveFilterState, kMaximumAcousticFilterStages>, 2>
    g_mna_acoustic_filter_states{};
std::array<size_t, 2> g_mna_acoustic_filter_count{};
std::array<PitchShiftState, 2> g_pitch_shift_states;
PlacedComponent g_audio_dac_component;
bool g_has_audio_dac_component = false;
RtlRuntimeManager* g_audio_rtl_runtime_manager = nullptr;
struct RealtimeAudioSnapshot {
  AudioCircuitStatus status;
  PlacedComponent dac_component;
  bool has_dac_component = false;
  RtlRuntimeManager* rtl_runtime_manager = nullptr;
};
std::atomic<std::shared_ptr<const RealtimeAudioSnapshot>>
    g_realtime_audio_snapshot;
std::string g_audio_worker_error;
std::atomic<bool> g_realtime_worker_unavailable{false};
float g_last_realtime_left = 0.0f;
float g_last_realtime_right = 0.0f;
struct AsyncSampleClockState {
  static constexpr size_t kCapacityFrames = 16384;
  std::array<float, kCapacityFrames * 2> samples{};
  std::uint64_t oldest_sequence = 0;
  std::uint64_t write_sequence = 0;
  double read_position = 0.0;
  float ratio = 1.0f;
  float held_left = 0.0f;
  float held_right = 0.0f;
  bool initialized = false;
};
AsyncSampleClockState g_async_sample_clock;
std::uint32_t g_noise_state = 0x8f3a91c5U;
std::vector<AudioOutputDevice> g_output_devices;
AudioCircuitStatus g_cached_circuit_evaluation;
std::uint64_t g_cached_circuit_signature = 0;
bool g_has_cached_circuit_evaluation = false;
std::array<std::array<float, 10>, 2> g_cached_mna_response_db{};
std::array<float, kAudioCircuitResponsePoints>
    g_mna_response_frequencies_hz = [] {
      std::array<float, kAudioCircuitResponsePoints> result{};
      const float ratio = std::pow(
          20000.0f / 20.0f,
          1.0f / static_cast<float>(kAudioCircuitResponsePoints - 1));
      result[0] = 20.0f;
      for (size_t index = 1; index < result.size(); ++index) {
        result[index] = index + 1 == result.size()
                            ? 20000.0f
                            : result[index - 1] * ratio;
      }
      return result;
    }();
std::array<std::array<float, kAudioCircuitResponsePoints>, 2>
    g_cached_mna_high_resolution_db{};
std::atomic<std::shared_ptr<audio::CompiledAudioCircuit>>
    g_compiled_audio_circuit;
std::atomic<float> g_mna_block_time_ms = 0.0f;
std::atomic<float> g_mna_deadline_percent = 0.0f;
bool g_audio_output_was_enabled = false;
std::array<std::uint64_t, 6> g_rtl_pending_sequences{};
size_t g_rtl_pending_head = 0;
size_t g_rtl_pending_count = 0;
float g_last_dac_code_left = 0.0f;
float g_last_dac_code_right = 0.0f;
bool g_has_last_dac_code = false;

float BlockRms(const float* samples, size_t frame_count) {
  if (!samples || frame_count == 0) return 0.0f;
  double energy = 0.0;
  for (size_t index = 0; index < frame_count * 2U; ++index) {
    energy += static_cast<double>(samples[index]) * samples[index];
  }
  return static_cast<float>(
      std::sqrt(energy / static_cast<double>(frame_count * 2U)));
}

void FadeRealtimeOutput(float* samples, size_t frame_count) {
  constexpr size_t kFadeFrames = 64;
  for (size_t frame = 0; frame < frame_count; ++frame) {
    const float gain = frame < kFadeFrames
                           ? 1.0f - static_cast<float>(frame + 1) /
                                        static_cast<float>(kFadeFrames)
                           : 0.0f;
    samples[frame * 2] = g_last_realtime_left * gain;
    samples[frame * 2 + 1] = g_last_realtime_right * gain;
  }
  g_last_realtime_left = 0.0f;
  g_last_realtime_right = 0.0f;
  g_async_sample_clock.oldest_sequence = 0;
  g_async_sample_clock.write_sequence = 0;
  g_async_sample_clock.read_position = 0.0;
  g_async_sample_clock.initialized = false;
}

void RememberRealtimeOutput(const float* samples, size_t frame_count) {
  if (!samples || frame_count == 0) return;
  g_last_realtime_left = samples[(frame_count - 1) * 2];
  g_last_realtime_right = samples[(frame_count - 1) * 2 + 1];
}

void ConvertAsynchronousSampleClock(const float* input, float* output,
                                    size_t frame_count, float ratio) {
  AsyncSampleClockState& state = g_async_sample_clock;
  ratio = std::clamp(ratio, 0.25f, 4.0f);
  if (std::abs(ratio - 1.0f) <= 0.001f) {
    std::copy_n(input, frame_count * 2, output);
    state.oldest_sequence = 0;
    state.write_sequence = 0;
    state.read_position = 0.0;
    state.held_left = 0.0f;
    state.held_right = 0.0f;
    state.initialized = false;
    return;
  }
  if (!state.initialized || std::abs(state.ratio - ratio) > 0.0001f) {
    state.oldest_sequence = 0;
    state.write_sequence = 0;
    state.read_position = 0.0;
    state.held_left = 0.0f;
    state.held_right = 0.0f;
    state.ratio = ratio;
    state.initialized = true;
  }
  for (size_t frame = 0; frame < frame_count; ++frame) {
    if (state.write_sequence - state.oldest_sequence >=
        AsyncSampleClockState::kCapacityFrames) {
      ++state.oldest_sequence;
    }
    const size_t slot = static_cast<size_t>(
        state.write_sequence % AsyncSampleClockState::kCapacityFrames);
    state.samples[slot * 2] = input[frame * 2];
    state.samples[slot * 2 + 1] = input[frame * 2 + 1];
    ++state.write_sequence;
  }
  if (state.read_position < static_cast<double>(state.oldest_sequence)) {
    // A slower consumer eventually overflows a finite hardware FIFO. Jumping
    // to the oldest surviving sample models the corresponding time skip.
    state.read_position = static_cast<double>(state.oldest_sequence);
  }
  for (size_t frame = 0; frame < frame_count; ++frame) {
    const std::uint64_t first =
        static_cast<std::uint64_t>(std::floor(state.read_position));
    const std::uint64_t second = first + 1U;
    if (second < state.write_sequence && first >= state.oldest_sequence) {
      const float blend = static_cast<float>(
          state.read_position - static_cast<double>(first));
      const size_t first_slot = static_cast<size_t>(
          first % AsyncSampleClockState::kCapacityFrames);
      const size_t second_slot = static_cast<size_t>(
          second % AsyncSampleClockState::kCapacityFrames);
      state.held_left =
          state.samples[first_slot * 2] +
          (state.samples[second_slot * 2] - state.samples[first_slot * 2]) *
              blend;
      state.held_right =
          state.samples[first_slot * 2 + 1] +
          (state.samples[second_slot * 2 + 1] -
           state.samples[first_slot * 2 + 1]) *
              blend;
      state.read_position += ratio;
    }
    // A faster consumer eventually underruns. A real DAC holds its most
    // recent code until another LRCLK sample arrives, so do the same here.
    output[frame * 2] = state.held_left;
    output[frame * 2 + 1] = state.held_right;
  }
}

#ifdef _WIN32
const PROPERTYKEY kDeviceFriendlyNameKey = {
    {0xa45c254e, 0xdf1c, 0x4efd,
     {0x80, 0x20, 0x67, 0xd1, 0x46, 0xa8, 0x50, 0xe0}},
    14};
#endif

#ifdef _WIN32
IAudioEndpointVolume* g_endpoint = nullptr;
float g_original_volume = 0.0f;
BOOL g_original_mute = FALSE;
std::vector<float> g_original_channels;
bool g_com_initialized = false;
bool g_endpoint_changed = false;
std::string g_active_device_id;

std::string WideToUtf8(const wchar_t* text) {
  if (!text || text[0] == L'\0') return {};
  const int size = WideCharToMultiByte(CP_UTF8, 0, text, -1, nullptr, 0,
                                       nullptr, nullptr);
  if (size <= 1) return {};
  std::string result(static_cast<size_t>(size), '\0');
  WideCharToMultiByte(CP_UTF8, 0, text, -1, result.data(), size, nullptr,
                      nullptr);
  result.pop_back();
  return result;
}

std::wstring Utf8ToWide(const std::string& text) {
  if (text.empty()) return {};
  const int size = MultiByteToWideChar(CP_UTF8, 0, text.c_str(), -1, nullptr,
                                       0);
  if (size <= 1) return {};
  std::wstring result(static_cast<size_t>(size), L'\0');
  MultiByteToWideChar(CP_UTF8, 0, text.c_str(), -1, result.data(), size);
  result.pop_back();
  return result;
}

void EnsureEndpoint(const std::string& requested_device_id) {
  if (g_endpoint && g_active_device_id == requested_device_id) return;
  if (g_endpoint) {
    if (g_endpoint_changed) {
      for (UINT channel = 0; channel < g_original_channels.size(); ++channel) {
        g_endpoint->SetChannelVolumeLevelScalar(
            channel, g_original_channels[static_cast<size_t>(channel)],
            nullptr);
      }
      // Channel writes can change the endpoint's effective master level.
      // Restore the saved master last so the final value is deterministic.
      g_endpoint->SetMasterVolumeLevelScalar(g_original_volume, nullptr);
      g_endpoint->SetMute(g_original_mute, nullptr);
    }
    g_endpoint->Release();
    g_endpoint = nullptr;
    g_endpoint_changed = false;
    g_original_channels.clear();
  }
  if (!g_com_initialized) {
    const HRESULT com_result =
        CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
    if (SUCCEEDED(com_result)) g_com_initialized = true;
  }
  IMMDeviceEnumerator* enumerator = nullptr;
  IMMDevice* device = nullptr;
  const HRESULT enumerator_result = CoCreateInstance(
      __uuidof(MMDeviceEnumerator), nullptr, CLSCTX_ALL,
      __uuidof(IMMDeviceEnumerator), reinterpret_cast<void**>(&enumerator));
  if (SUCCEEDED(enumerator_result) && !requested_device_id.empty()) {
    const std::wstring wide_id = Utf8ToWide(requested_device_id);
    if (!wide_id.empty()) enumerator->GetDevice(wide_id.c_str(), &device);
  }
  if (SUCCEEDED(enumerator_result) && !device) {
    enumerator->GetDefaultAudioEndpoint(eRender, eMultimedia, &device);
  }
  g_active_device_id = requested_device_id;
  if (device &&
      SUCCEEDED(device->Activate(__uuidof(IAudioEndpointVolume), CLSCTX_ALL,
                                nullptr,
                                reinterpret_cast<void**>(&g_endpoint)))) {
    g_endpoint->GetMasterVolumeLevelScalar(&g_original_volume);
    g_endpoint->GetMute(&g_original_mute);
    UINT channel_count = 0;
    if (SUCCEEDED(g_endpoint->GetChannelCount(&channel_count))) {
      g_original_channels.resize(channel_count, 1.0f);
      for (UINT channel = 0; channel < channel_count; ++channel) {
        g_endpoint->GetChannelVolumeLevelScalar(
            channel, &g_original_channels[static_cast<size_t>(channel)]);
      }
    }
  }
  if (device) device->Release();
  if (enumerator) enumerator->Release();
}

[[maybe_unused]] void ApplyEndpointStereo(float left, float right,
                         float maximum_scalar,
                         bool hard_mute,
                         const std::string& output_device_id) {
  EnsureEndpoint(output_device_id);
  if (!g_endpoint) return;
  g_endpoint_changed = true;
  const float safety_cap =
      std::isfinite(maximum_scalar)
          ? std::clamp(maximum_scalar, 0.0f, 1.0f)
          : 0.0f;
  const float safe_left =
      std::clamp(std::isfinite(left) ? left : 0.0f, 0.0f, safety_cap);
  const float safe_right =
      std::clamp(std::isfinite(right) ? right : 0.0f, 0.0f, safety_cap);
  if (hard_mute) {
    g_endpoint->SetMute(TRUE, nullptr);
    g_endpoint->SetMasterVolumeLevelScalar(0.0f, nullptr);
    return;
  }
  const float master = std::max(safe_left, safe_right);
  g_endpoint->SetMute(FALSE, nullptr);
  UINT channel_count = 0;
  if (SUCCEEDED(g_endpoint->GetChannelCount(&channel_count))) {
    if (channel_count > 0) {
      g_endpoint->SetChannelVolumeLevelScalar(0, safe_left, nullptr);
    }
    if (channel_count > 1) {
      g_endpoint->SetChannelVolumeLevelScalar(1, safe_right, nullptr);
    }
  }
  // IAudioEndpointVolume channel writes can alter the effective endpoint
  // level. Apply the user-defined ceiling last; no channel may raise it.
  g_endpoint->SetMasterVolumeLevelScalar(master, nullptr);
}

void RestoreEndpointVolume() {
  if (!g_endpoint || !g_endpoint_changed) return;
  for (UINT channel = 0; channel < g_original_channels.size(); ++channel) {
    g_endpoint->SetChannelVolumeLevelScalar(
        channel, g_original_channels[static_cast<size_t>(channel)], nullptr);
  }
  g_endpoint->SetMasterVolumeLevelScalar(g_original_volume, nullptr);
  g_endpoint->SetMute(g_original_mute, nullptr);
  g_endpoint_changed = false;
}
#else
void ApplyEndpointStereo(float, float, float, bool, const std::string&) {}
void RestoreEndpointVolume() {}
#endif

float State(const PlacedComponent& component, const char* key, float fallback) {
  const auto found = component.internalStates.find(key);
  return found == component.internalStates.end() ? fallback : found->second;
}

void HashMix(std::uint64_t* hash, std::uint64_t value) {
  if (!hash) return;
  *hash ^= value + 0x9e3779b97f4a7c15ULL + (*hash << 6U) + (*hash >> 2U);
}

bool IsCircuitParameterKey(const std::string& key) {
  return key == state_keys::kBitDepth ||
         key == state_keys::kResistanceOhms ||
         key == state_keys::kTolerancePercent ||
         key == state_keys::kWiperPosition ||
         key == state_keys::kCapacitanceUf ||
         key == state_keys::kEsrOhms ||
         key == state_keys::kLeakageResistanceOhms ||
         key == state_keys::kVoltageRatingVolts ||
         key == state_keys::kInductanceMh ||
         key == state_keys::kDcrOhms ||
         key == state_keys::kSaturationCurrentAmps ||
         key == state_keys::kGain ||
         key == state_keys::kOpenLoopGain ||
         key == state_keys::kGainBandwidthHz ||
         key == state_keys::kSlewRateVoltsPerUs ||
         key == state_keys::kInputOffsetMillivolts ||
         key == state_keys::kOutputCurrentLimitAmps ||
         key == state_keys::kOutputResistanceOhms ||
         key == state_keys::kOutputHeadroomVolts ||
         key == state_keys::kForwardVoltageVolts ||
         key == state_keys::kDynamicResistanceOhms ||
         key == state_keys::kThermalCoupling ||
         key == state_keys::kCurrentGainBeta ||
         key == state_keys::kBaseEmitterVoltageVolts ||
         key == state_keys::kSaturationVoltageVolts ||
         key == state_keys::kMaximumCollectorCurrentAmps ||
         key == state_keys::kMaximumPowerWatts ||
         key == state_keys::kThermalResistanceCPerW ||
         key == state_keys::kImpedanceOhms ||
         key == state_keys::kTransducerEq1FrequencyHz ||
         key == state_keys::kTransducerEq1GainDb ||
         key == state_keys::kTransducerEq1Q ||
         key == state_keys::kTransducerEq2FrequencyHz ||
         key == state_keys::kTransducerEq2GainDb ||
         key == state_keys::kTransducerEq2Q ||
         key == state_keys::kTransducerEq3FrequencyHz ||
         key == state_keys::kTransducerEq3GainDb ||
         key == state_keys::kTransducerEq3Q ||
         key == state_keys::kTransducerEq4FrequencyHz ||
         key == state_keys::kTransducerEq4GainDb ||
         key == state_keys::kTransducerEq4Q ||
         key == state_keys::kVoltageVolts ||
         key == state_keys::kCurrentLimitAmps ||
         key == state_keys::kInternalResistanceOhms ||
         key == state_keys::kSpeakerReOhms ||
         key == state_keys::kSpeakerLeMh ||
         key == state_keys::kSpeakerBlTeslaMeters ||
         key == state_keys::kSpeakerMmsGrams ||
         key == state_keys::kSpeakerCmsMmPerNewton ||
         key == state_keys::kSpeakerRmsNewtonSecondsPerMeter ||
         key == state_keys::kFrequencyHz ||
         key == state_keys::kFrequencyMinHz ||
         key == state_keys::kFrequencyMaxHz ||
         key == state_keys::kPhaseDegrees ||
         key == state_keys::kDutyCyclePercent ||
         key == state_keys::kPulseLowVolts ||
         key == state_keys::kPulseHighVolts;
}

std::uint64_t BuildCircuitSignature(
    const std::vector<PlacedComponent>* components,
    const std::vector<Wire>& wires) {
  std::uint64_t hash = 0xcbf29ce484222325ULL;
  if (components) {
    HashMix(&hash, components->size());
    for (const PlacedComponent& component : *components) {
      HashMix(&hash, static_cast<std::uint64_t>(component.instanceId));
      HashMix(&hash, static_cast<std::uint64_t>(component.type));
      for (const auto& state : component.internalStates) {
        // Verilator pin values and runtime meters can change every audio/frame
        // tick. Only user-editable circuit parameters invalidate topology.
        if (!IsCircuitParameterKey(state.first)) continue;
        for (unsigned char character : state.first) HashMix(&hash, character);
        HashMix(&hash, std::bit_cast<std::uint32_t>(state.second));
      }
      for (unsigned char character : component.rtlModuleId) {
        HashMix(&hash, character);
      }
      HashMix(&hash, std::bit_cast<std::uint32_t>(
                         component.rtlClockFrequencyHz));
      for (const RtlPinBinding& binding : component.rtlPinBindings) {
        HashMix(&hash, static_cast<std::uint64_t>(binding.portId));
        HashMix(&hash, binding.isInput ? 1U : 0U);
        for (unsigned char character : binding.pinName) HashMix(&hash, character);
      }
    }
  }
  HashMix(&hash, wires.size());
  for (const Wire& wire : wires) {
    HashMix(&hash, static_cast<std::uint64_t>(wire.fromComponentId));
    HashMix(&hash, static_cast<std::uint64_t>(wire.fromPortId));
    HashMix(&hash, static_cast<std::uint64_t>(wire.toComponentId));
    HashMix(&hash, static_cast<std::uint64_t>(wire.toPortId));
    HashMix(&hash, wire.isElectric ? 1U : 0U);
  }
  return hash;
}

float LinearToDb(float value) {
  return value > 0.000001f ? 20.0f * std::log10(value) : kSilenceDb;
}

struct BiquadCoefficients {
  float b0 = 1.0f;
  float b1 = 0.0f;
  float b2 = 0.0f;
  float a1 = 0.0f;
  float a2 = 0.0f;
};

BiquadCoefficients BuildPeakingEq(float center_hz, float gain_db, float q,
                                  float sample_rate) {
  const float safe_rate = std::max(sample_rate, 1.0f);
  const float frequency =
      std::clamp(center_hz, 1.0f, safe_rate * 0.49f);
  const float safe_q = std::clamp(q, 0.1f, 20.0f);
  const float a = std::pow(10.0f, gain_db / 40.0f);
  const float omega = 2.0f * kPi * frequency / safe_rate;
  const float cosine = std::cos(omega);
  const float alpha = std::sin(omega) / (2.0f * safe_q);
  const float a0 = 1.0f + alpha / a;
  return {(1.0f + alpha * a) / a0,
          (-2.0f * cosine) / a0,
          (1.0f - alpha * a) / a0,
          (-2.0f * cosine) / a0,
          (1.0f - alpha / a) / a0};
}

float PeakingEqMagnitude(float frequency_hz, float center_hz, float gain_db,
                         float q, float sample_rate) {
  const BiquadCoefficients coefficients =
      BuildPeakingEq(center_hz, gain_db, q, sample_rate);
  const float omega = 2.0f * kPi * frequency_hz / sample_rate;
  const std::complex<float> z1 = std::polar(1.0f, -omega);
  const std::complex<float> z2 = z1 * z1;
  const std::complex<float> numerator =
      coefficients.b0 + coefficients.b1 * z1 + coefficients.b2 * z2;
  const std::complex<float> denominator =
      1.0f + coefficients.a1 * z1 + coefficients.a2 * z2;
  return std::abs(numerator / denominator);
}

void ApplyTransducerResponse(const PlacedComponent& speaker,
                             const std::array<float, 10>& frequencies,
                             AudioChannelStatus* channel) {
  if (!channel || !channel->route_complete) return;
  constexpr std::array<const char*, 4> kFrequencyKeys = {
      state_keys::kTransducerEq1FrequencyHz,
      state_keys::kTransducerEq2FrequencyHz,
      state_keys::kTransducerEq3FrequencyHz,
      state_keys::kTransducerEq4FrequencyHz};
  constexpr std::array<const char*, 4> kGainKeys = {
      state_keys::kTransducerEq1GainDb, state_keys::kTransducerEq2GainDb,
      state_keys::kTransducerEq3GainDb, state_keys::kTransducerEq4GainDb};
  constexpr std::array<const char*, 4> kQKeys = {
      state_keys::kTransducerEq1Q, state_keys::kTransducerEq2Q,
      state_keys::kTransducerEq3Q, state_keys::kTransducerEq4Q};
  constexpr std::array<float, 4> kDefaultFrequencies = {
      125.0f, 500.0f, 4000.0f, 10000.0f};
  for (size_t band = 0; band < kFrequencyKeys.size(); ++band) {
    const float center = std::clamp(
        State(speaker, kFrequencyKeys[band], kDefaultFrequencies[band]),
        10.0f, 22000.0f);
    const float gain =
        std::clamp(State(speaker, kGainKeys[band], 0.0f), -24.0f, 24.0f);
    const float q =
        std::clamp(State(speaker, kQKeys[band], 1.0f), 0.1f, 20.0f);
    if (std::abs(gain) < 0.001f) continue;
    channel->filter_stages.push_back(
        {AudioFilterType::PEAKING_EQ, center, 1.0f,
         speaker.instanceId, gain, q});
    for (size_t index = 0; index < frequencies.size(); ++index) {
      const float magnitude = PeakingEqMagnitude(
          frequencies[index], center, gain, q, 48000.0f);
      channel->eq_db[index] += LinearToDb(magnitude);
    }
  }
}

void TransformFft(std::span<std::complex<float>> values) {
  if (values.empty()) return;
  const size_t count = values.size();
  for (size_t index = 1, reversed = 0; index < count; ++index) {
    size_t bit = count >> 1;
    for (; reversed & bit; bit >>= 1) reversed ^= bit;
    reversed ^= bit;
    if (index < reversed) std::swap(values[index], values[reversed]);
  }
  for (size_t length = 2; length <= count; length <<= 1) {
    const std::complex<float> step = std::polar(
        1.0f, -2.0f * kPi / static_cast<float>(length));
    for (size_t offset = 0; offset < count; offset += length) {
      std::complex<float> rotation{1.0f, 0.0f};
      for (size_t index = 0; index < length / 2; ++index) {
        const std::complex<float> even = values[offset + index];
        const std::complex<float> odd =
            values[offset + index + length / 2] * rotation;
        values[offset + index] = even + odd;
        values[offset + index + length / 2] = even - odd;
        rotation *= step;
      }
    }
  }
}

void AnalyzeSignalSamples(const float* interleaved_stereo, size_t frame_count,
                          unsigned int sample_rate) {
  if (!interleaved_stereo || frame_count == 0 || sample_rate == 0) return;
  for (size_t sample = 0; sample < frame_count * 2; ++sample) {
    g_analysis_sample_ring[g_analysis_ring_write] =
        interleaved_stereo[sample];
    g_analysis_ring_write =
        (g_analysis_ring_write + 1U) % g_analysis_sample_ring.size();
    g_analysis_ring_size =
        std::min(g_analysis_ring_size + 1U, g_analysis_sample_ring.size());
  }
  ++g_analysis_update_counter;
  if (g_analysis_ring_size < kAnalysisWindowFrames * 2) return;
  if (g_analysis_has_snapshot && g_analysis_update_counter % 4 != 0) return;
  std::array<float, kAnalysisWindowFrames * 2> analysis_samples{};
  for (size_t sample = 0; sample < analysis_samples.size(); ++sample) {
    analysis_samples[sample] =
        g_analysis_sample_ring[(g_analysis_ring_write + sample) %
                               g_analysis_sample_ring.size()];
  }
  interleaved_stereo = analysis_samples.data();
  frame_count = kAnalysisWindowFrames;

  AudioSignalAnalysis next;
  next.active = true;
  next.sample_rate = sample_rate;
  next.analyzed_frames = frame_count;

  for (size_t channel = 0; channel < 2; ++channel) {
    double square_sum = 0.0;
    float peak = 0.0f;
    size_t clipped = 0;
    for (size_t frame = 0; frame < frame_count; ++frame) {
      const float sample = interleaved_stereo[frame * 2 + channel];
      square_sum += static_cast<double>(sample) * sample;
      peak = std::max(peak, std::abs(sample));
      if (std::abs(sample) >= 0.999f) ++clipped;
    }
    const float rms = static_cast<float>(
        std::sqrt(square_sum / static_cast<double>(frame_count)));
    next.rms_dbfs[channel] = LinearToDb(rms);
    next.peak_dbfs[channel] = LinearToDb(peak);
    next.clipped_percent[channel] =
        static_cast<float>(clipped) * 100.0f / static_cast<float>(frame_count);

    for (size_t point = 0; point < kAudioAnalyzerWaveformPoints; ++point) {
      const size_t frame = kAudioAnalyzerWaveformPoints > 1
                               ? point * (frame_count - 1) /
                                     (kAudioAnalyzerWaveformPoints - 1)
                               : 0;
      next.waveform[channel][point] =
          interleaved_stereo[frame * 2 + channel];
    }
  }

  static const std::array<float, kAnalysisWindowFrames> window = [] {
    std::array<float, kAnalysisWindowFrames> result{};
    for (size_t frame = 0; frame < result.size(); ++frame) {
      result[frame] =
          0.5f - 0.5f *
                     std::cos(2.0f * kPi * static_cast<float>(frame) /
                              static_cast<float>(result.size() - 1));
    }
    return result;
  }();
  std::array<std::array<std::complex<float>, kAnalysisWindowFrames>, 2>
      spectra{};
  for (size_t channel = 0; channel < 2; ++channel) {
    for (size_t frame = 0; frame < frame_count; ++frame) {
      spectra[channel][frame] =
          interleaved_stereo[frame * 2 + channel] * window[frame];
    }
    TransformFft(spectra[channel]);

    size_t fundamental_bin = 1;
    float fundamental_power = 0.0f;
    for (size_t bin = 1; bin < frame_count / 2; ++bin) {
      const float power = std::norm(spectra[channel][bin]);
      if (power > fundamental_power) {
        fundamental_power = power;
        fundamental_bin = bin;
      }
    }
    next.fundamental_hz[channel] =
        static_cast<float>(fundamental_bin) *
        static_cast<float>(sample_rate) / static_cast<float>(frame_count);
    double harmonic_power = 0.0;
    for (size_t harmonic = 2; harmonic <= 10; ++harmonic) {
      const size_t center = fundamental_bin * harmonic;
      if (center >= frame_count / 2) break;
      for (int offset = -1; offset <= 1; ++offset) {
        const size_t bin = static_cast<size_t>(
            static_cast<std::ptrdiff_t>(center) + offset);
        harmonic_power += std::norm(spectra[channel][bin]);
      }
    }
    double total_ac_power = 0.0;
    for (size_t bin = 1; bin < frame_count / 2; ++bin) {
      total_ac_power += std::norm(spectra[channel][bin]);
    }
    const double fundamental_band_power =
        std::norm(spectra[channel][fundamental_bin]) +
        (fundamental_bin > 1
             ? std::norm(spectra[channel][fundamental_bin - 1])
             : 0.0f) +
        std::norm(spectra[channel][fundamental_bin + 1]);
    const double noise_and_distortion =
        std::max(0.0, total_ac_power - fundamental_band_power);
    if (fundamental_band_power > 1.0e-20) {
      next.thd_percent[channel] = static_cast<float>(
          std::sqrt(harmonic_power / fundamental_band_power) * 100.0);
      next.thd_plus_noise_percent[channel] = static_cast<float>(
          std::sqrt(noise_and_distortion / fundamental_band_power) * 100.0);
      next.snr_db[channel] = noise_and_distortion > 1.0e-20
                                 ? static_cast<float>(
                                       10.0 * std::log10(
                                                  fundamental_band_power /
                                                  noise_and_distortion))
                                 : 120.0f;
    }
  }

  constexpr float kSpectrumStartHz = 20.0f;
  constexpr float kSpectrumEndHz = 20000.0f;
  const float spectrum_ratio = std::pow(
      kSpectrumEndHz / kSpectrumStartHz,
      1.0f / static_cast<float>(kAudioAnalyzerSpectrumBins - 1));
  float frequency = kSpectrumStartHz;
  for (size_t bin = 0; bin < kAudioAnalyzerSpectrumBins; ++bin) {
    frequency = bin + 1 == kAudioAnalyzerSpectrumBins
                    ? kSpectrumEndHz
                    : frequency;
    next.spectrum_frequencies_hz[bin] = frequency;
    const size_t fft_bin = (std::min)(
        frame_count / 2,
        static_cast<size_t>(std::lround(
            frequency * static_cast<float>(frame_count) /
            static_cast<float>(sample_rate))));
    for (size_t channel = 0; channel < 2; ++channel) {
      const float amplitude =
          4.0f * std::abs(spectra[channel][fft_bin]) /
          static_cast<float>(frame_count);
      next.spectrum_dbfs[channel][bin] = LinearToDb(amplitude);
    }
    frequency *= spectrum_ratio;
  }

  const std::shared_ptr<const RealtimeAudioSnapshot> realtime =
      g_realtime_audio_snapshot.load(std::memory_order_acquire);
  if (realtime) {
    next.model_response_db = realtime->status.mna_ac_response_db;
  }

  std::unique_lock<std::mutex> lock(g_signal_analysis_mutex,
                                    std::try_to_lock);
  if (!lock.owns_lock()) return;
  next.sequence = g_signal_analysis.sequence + 1;
  g_signal_analysis = std::move(next);
  g_analysis_has_snapshot = true;
}

void EnsureSignalAnalysisWorker() {
  bool expected = false;
  if (!g_analysis_worker_started.compare_exchange_strong(
          expected, true, std::memory_order_acq_rel)) {
    return;
  }
  g_analysis_worker_stop.store(false, std::memory_order_release);
  g_analysis_worker = std::thread([]() {
#ifdef _WIN32
    // FFT/THD visualization is best-effort work. It must never compete at the
    // same priority as the MMCSS circuit and WASAPI threads.
    SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
#endif
    std::array<float, 1024 * 2> samples{};
    while (!g_analysis_worker_stop.load(std::memory_order_acquire)) {
      const size_t available_frames =
          g_analysis_queue.AvailableRead() / 2U;
      if (available_frames == 0) {
        std::unique_lock<std::mutex> lock(g_analysis_worker_wake_mutex);
        g_analysis_worker_wake.wait_for(
            lock, std::chrono::milliseconds(10), []() {
              return g_analysis_worker_stop.load(std::memory_order_acquire) ||
                     g_analysis_queue.AvailableRead() >= 2U;
            });
        continue;
      }
      const size_t frames =
          (std::min)(available_frames, samples.size() / 2U);
      const size_t read = g_analysis_queue.Read(samples.data(), frames * 2U);
      if (read > 0) {
        AnalyzeSignalSamples(samples.data(), read / 2U, 48000U);
      }
    }
  });
}

void UpdateSignalAnalysis(const float* interleaved_stereo, size_t frame_count,
                          unsigned int sample_rate) {
  (void)sample_rate;
  if (!interleaved_stereo || frame_count == 0) return;
  const size_t samples = frame_count * 2U;
  if (g_analysis_queue.AvailableWrite() < samples) return;
  g_analysis_queue.Write(interleaved_stereo, samples);
  g_analysis_worker_wake.notify_one();
}

void AddInternalEdge(std::multimap<PortNode, Edge>* graph,
                     const PlacedComponent& component, int a, int b) {
  graph->emplace(PortNode{component.instanceId, a},
                 Edge{PortNode{component.instanceId, b}, &component});
  graph->emplace(PortNode{component.instanceId, b},
                 Edge{PortNode{component.instanceId, a}, &component});
}

bool HasRtlPin(const PlacedComponent& component, const char* pin_name,
               bool input) {
  const std::string bit_prefix = std::string(pin_name) + "[";
  return std::any_of(
      component.rtlPinBindings.begin(), component.rtlPinBindings.end(),
      [&](const RtlPinBinding& binding) {
        return binding.isInput == input &&
               (binding.pinName == pin_name ||
                binding.pinName.rfind(bit_prefix, 0) == 0);
      });
}

size_t CountRtlBusBits(const PlacedComponent& component,
                       const char* base_name,
                       bool input) {
  const std::string bit_prefix = std::string(base_name) + "[";
  return static_cast<size_t>(std::count_if(
      component.rtlPinBindings.begin(), component.rtlPinBindings.end(),
      [&](const RtlPinBinding& binding) {
        return binding.isInput == input &&
               binding.pinName.rfind(bit_prefix, 0) == 0;
      }));
}

bool HasFunctionalStereoDacInterface(const PlacedComponent& component) {
  if (component.type != ComponentType::RTL_MODULE ||
      component.rtlModuleId != "audio_shell_dac") {
    return false;
  }
  // The example DAC uses an I2S receiver to drive the two signed 16-bit codes
  // of an R-2R ladder.  Verilator evaluates the digital receiver/latches; the
  // audio circuit runtime converts those codes through the ladder transfer.
  return HasRtlPin(component, "bclk", true) &&
         HasRtlPin(component, "lrclk", true) &&
         HasRtlPin(component, "sdata", true) &&
         HasRtlPin(component, "rst_n", true) &&
         CountRtlBusBits(component, "dac_l", false) == 16 &&
         CountRtlBusBits(component, "dac_r", false) == 16;
}

AudioTopologyIndex BuildTopologyIndex(
    const std::vector<PlacedComponent>& components,
    const std::vector<Wire>& wires) {
  AudioTopologyIndex index;
  for (const PlacedComponent& component : components) {
    index.component_by_id.emplace(component.instanceId, &component);
  }
  for (const Wire& wire : wires) {
    if (!wire.isElectric) continue;
    index.wires_by_port.emplace(
        PortNode{wire.fromComponentId, wire.fromPortId}, &wire);
    index.wires_by_port.emplace(
        PortNode{wire.toComponentId, wire.toPortId}, &wire);
  }
  return index;
}

const PlacedComponent* FindComponentById(const AudioTopologyIndex& index,
                                         int component_id) {
  const auto found = index.component_by_id.find(component_id);
  return found == index.component_by_id.end() ? nullptr : found->second;
}

const PlacedComponent* FindDacBitResistor(
    const PlacedComponent& dac, const RtlPinBinding& bit,
    const AudioTopologyIndex& index) {
  const auto range = index.wires_by_port.equal_range(
      PortNode{dac.instanceId, bit.portId});
  for (auto entry = range.first; entry != range.second; ++entry) {
    const Wire& wire = *entry->second;
    int other_component_id = -1;
    if (wire.fromComponentId == dac.instanceId &&
        wire.fromPortId == bit.portId) {
      other_component_id = wire.toComponentId;
    } else if (wire.toComponentId == dac.instanceId &&
               wire.toPortId == bit.portId) {
      other_component_id = wire.fromComponentId;
    }
    if (other_component_id < 0) continue;
    const PlacedComponent* other = FindComponentById(index, other_component_id);
    if (other && other->type == ComponentType::AUDIO_RESISTOR) return other;
  }
  return nullptr;
}

int ParseBusBitIndex(const std::string& pin_name,
                     const std::string& prefix) {
  if (pin_name.empty() || pin_name.rfind(prefix, 0) != 0 ||
      pin_name.back() != ']') {
    return -1;
  }
  const std::string number =
      pin_name.substr(prefix.size(), pin_name.size() - prefix.size() - 1);
  char* end = nullptr;
  const long parsed = std::strtol(number.c_str(), &end, 10);
  return end && *end == '\0' && parsed >= 0 && parsed < 16
             ? static_cast<int>(parsed)
             : -1;
}

DacBusResponse BuildDacBusResponse(
    const PlacedComponent& dac, const char* base_name,
    const AudioTopologyIndex& index) {
  DacBusResponse response;
  const std::string prefix = std::string(base_name) + "[";
  std::array<float, 16> resistances{};
  std::vector<float> connected_resistances;
  for (const RtlPinBinding& binding : dac.rtlPinBindings) {
    if (binding.isInput) continue;
    const int bit_index = ParseBusBitIndex(binding.pinName, prefix);
    if (bit_index < 0) continue;
    const PlacedComponent* resistor =
        FindDacBitResistor(dac, binding, index);
    if (!resistor) continue;
    const float nominal_resistance = std::max(
        0.001f, State(*resistor, state_keys::kResistanceOhms, 1000.0f));
    const float tolerance = std::clamp(
        State(*resistor, state_keys::kTolerancePercent, 1.0f), 0.0f, 20.0f) /
        100.0f;
    const std::uint32_t spread =
        static_cast<std::uint32_t>(resistor->instanceId) * 2654435761U;
    const float signed_error =
        static_cast<float>(spread & 0xffffU) / 32767.5f - 1.0f;
    const float resistance = std::max(
        0.001f, nominal_resistance * (1.0f + tolerance * signed_error));
    resistances[static_cast<size_t>(bit_index)] = resistance;
    connected_resistances.push_back(resistance);
    response.connected_mask |=
        static_cast<std::uint16_t>(1U << static_cast<unsigned int>(bit_index));
  }
  response.connected_bits = connected_resistances.size();
  response.complete = response.connected_mask == 0xffffU;
  if (connected_resistances.empty()) return response;

  std::sort(connected_resistances.begin(), connected_resistances.end());
  const float reference_resistance =
      connected_resistances[connected_resistances.size() / 2];
  for (size_t bit = 0; bit < response.bit_weights.size(); ++bit) {
    if (resistances[bit] <= 0.0f) continue;
    // A wrong branch resistance changes that bit's contribution instead of
    // turning the whole converter off.  Limit extreme shorts so malformed
    // circuits remain audible without producing unbounded samples.
    response.bit_weights[bit] =
        std::clamp(reference_resistance / resistances[bit], 0.0f, 8.0f);
  }
  return response;
}

std::set<int> BuildDacNetworkResistorIds(
    const std::vector<PlacedComponent>& components,
    const std::vector<Wire>& wires, const AudioTopologyIndex& index) {
  std::set<int> resistor_ids;
  for (const PlacedComponent& component : components) {
    if (!HasFunctionalStereoDacInterface(component)) continue;
    for (const RtlPinBinding& binding : component.rtlPinBindings) {
      if (binding.isInput) continue;
      if (binding.pinName.rfind("dac_l[", 0) != 0 &&
          binding.pinName.rfind("dac_r[", 0) != 0) {
        continue;
      }
      const PlacedComponent* resistor =
          FindDacBitResistor(component, binding, index);
      if (resistor) resistor_ids.insert(resistor->instanceId);
    }
  }

  bool changed = true;
  while (changed) {
    changed = false;
    for (const Wire& wire : wires) {
      if (!wire.isElectric) continue;
      const bool from_is_ladder =
          resistor_ids.count(wire.fromComponentId) != 0;
      const bool to_is_ladder =
          resistor_ids.count(wire.toComponentId) != 0;
      if (from_is_ladder == to_is_ladder) continue;
      const int candidate_id =
          from_is_ladder ? wire.toComponentId : wire.fromComponentId;
      const PlacedComponent* candidate = FindComponentById(index, candidate_id);
      if (candidate && candidate->type == ComponentType::AUDIO_RESISTOR) {
        // A real ladder may continue through a short chain of R/2R parts, but
        // the reconstruction filter begins at the summing resistor fed by
        // many bit branches. Treating that high-fan-in resistor as part of
        // the DAC made its shunt capacitor disappear from both the modeled
        // response and realtime audio path.
        int ladder_neighbor_count = 0;
        for (const Wire& neighbor_wire : wires) {
          if (!neighbor_wire.isElectric) continue;
          int neighbor_id = -1;
          if (neighbor_wire.fromComponentId == candidate_id) {
            neighbor_id = neighbor_wire.toComponentId;
          } else if (neighbor_wire.toComponentId == candidate_id) {
            neighbor_id = neighbor_wire.fromComponentId;
          }
          if (neighbor_id >= 0 &&
              resistor_ids.count(neighbor_id) != 0) {
            ++ladder_neighbor_count;
          }
        }
        if (ladder_neighbor_count > 3) continue;
        changed = resistor_ids.insert(candidate_id).second || changed;
      }
    }
  }
  return resistor_ids;
}

std::map<int, int> BuildDacDrivenResistorPorts(
    const std::vector<PlacedComponent>& components,
    const AudioTopologyIndex& index) {
  std::map<int, int> driven_ports;
  for (const PlacedComponent& dac : components) {
    if (!HasFunctionalStereoDacInterface(dac)) continue;
    for (const RtlPinBinding& binding : dac.rtlPinBindings) {
      if (binding.isInput ||
          (binding.pinName.rfind("dac_l[", 0) != 0 &&
           binding.pinName.rfind("dac_r[", 0) != 0)) {
        continue;
      }
      const auto range = index.wires_by_port.equal_range(
          PortNode{dac.instanceId, binding.portId});
      for (auto entry = range.first; entry != range.second; ++entry) {
        const Wire& wire = *entry->second;
        int resistor_id = -1;
        int resistor_port = -1;
        if (wire.fromComponentId == dac.instanceId &&
            wire.fromPortId == binding.portId) {
          resistor_id = wire.toComponentId;
          resistor_port = wire.toPortId;
        } else if (wire.toComponentId == dac.instanceId &&
                   wire.toPortId == binding.portId) {
          resistor_id = wire.fromComponentId;
          resistor_port = wire.fromPortId;
        }
        const PlacedComponent* resistor =
            FindComponentById(index, resistor_id);
        if (resistor && resistor->type == ComponentType::AUDIO_RESISTOR) {
          driven_ports[resistor_id] = resistor_port;
        }
      }
    }
  }
  return driven_ports;
}

bool EqualsIgnoreCase(const std::string& left, const std::string& right) {
  if (left.size() != right.size()) return false;
  for (size_t index = 0; index < left.size(); ++index) {
    if (std::tolower(static_cast<unsigned char>(left[index])) !=
        std::tolower(static_cast<unsigned char>(right[index]))) {
      return false;
    }
  }
  return true;
}

bool IsVoltageAmplifier(const PlacedComponent& component) {
  return component.type == ComponentType::AUDIO_OP_AMP ||
         (component.type == ComponentType::RTL_MODULE &&
          component.rtlModuleId == "audio_shell_amplifier");
}

bool IsOutputTransistor(const PlacedComponent& component) {
  return component.type == ComponentType::AUDIO_BJT_NPN ||
         component.type == ComponentType::AUDIO_BJT_PNP;
}

bool IsAudioGainDevice(const PlacedComponent& component) {
  return IsVoltageAmplifier(component) || IsOutputTransistor(component);
}

const RtlPinBinding* FindRtlBinding(const PlacedComponent& component,
                                    const std::string& pin_name) {
  if (pin_name.empty()) return nullptr;
  const auto found = std::find_if(
      component.rtlPinBindings.begin(), component.rtlPinBindings.end(),
      [&](const RtlPinBinding& binding) {
        return EqualsIgnoreCase(binding.pinName, pin_name);
      });
  return found == component.rtlPinBindings.end() ? nullptr : &*found;
}

void BuildPortGraph(const std::vector<PlacedComponent>& components,
                    const std::vector<Wire>& wires,
                    std::multimap<PortNode, Edge>* graph) {
  for (const Wire& wire : wires) {
    if (!wire.isElectric) continue;
    const PortNode from{wire.fromComponentId, wire.fromPortId};
    const PortNode to{wire.toComponentId, wire.toPortId};
    graph->emplace(from, Edge{to, nullptr});
    graph->emplace(to, Edge{from, nullptr});
  }
  const PlacedComponent* common_ground = nullptr;
  for (const PlacedComponent& component : components) {
    if (!common_ground && component.type == ComponentType::AUDIO_GROUND) {
      common_ground = &component;
    }
  }
  for (const PlacedComponent& component : components) {
    switch (component.type) {
      case ComponentType::AUDIO_ADC:
      case ComponentType::AUDIO_DAC:
      case ComponentType::AUDIO_RESISTOR:
      case ComponentType::AUDIO_CAPACITOR:
      case ComponentType::AUDIO_INDUCTOR:
        AddInternalEdge(graph, component, 0, 1);
        break;
      case ComponentType::AUDIO_DIODE:
        // The analog solver applies the forward drop/dynamic resistance; the
        // topology layer keeps both directions so a reversed diode produces
        // severe distortion instead of disappearing from the graph.
        AddInternalEdge(graph, component, 0, 1);
        break;
      case ComponentType::AUDIO_OP_AMP:
        graph->emplace(PortNode{component.instanceId, 0},
                       Edge{PortNode{component.instanceId, 4}, &component});
        graph->emplace(PortNode{component.instanceId, 1},
                       Edge{PortNode{component.instanceId, 4}, &component});
        break;
      case ComponentType::AUDIO_BJT_NPN:
      case ComponentType::AUDIO_BJT_PNP:
        AddInternalEdge(graph, component, 0, 2);
        break;
      case ComponentType::AUDIO_POTENTIOMETER:
        AddInternalEdge(graph, component, 0, 1);
        AddInternalEdge(graph, component, 2, 1);
        break;
      case ComponentType::RTL_MODULE:
        if (component.rtlModuleId == "audio_shell_dac" &&
            !HasFunctionalStereoDacInterface(component)) {
          break;
        }
        for (const RtlPinBinding& input : component.rtlPinBindings) {
          if (!input.isInput) continue;
          if (component.rtlModuleId == "audio_shell_amplifier" &&
              (EqualsIgnoreCase(input.pinName, component.rtlPowerPinName) ||
               EqualsIgnoreCase(input.pinName, component.rtlGroundPinName))) {
            continue;
          }
          for (const RtlPinBinding& output : component.rtlPinBindings) {
            if (output.isInput) continue;
            graph->emplace(PortNode{component.instanceId, input.portId},
                           Edge{PortNode{component.instanceId, output.portId},
                                &component});
          }
        }
        break;
      case ComponentType::AUDIO_GROUND:
        if (common_ground &&
            component.instanceId != common_ground->instanceId) {
          graph->emplace(PortNode{component.instanceId, 0},
                         Edge{PortNode{common_ground->instanceId, 0}, nullptr});
          graph->emplace(PortNode{common_ground->instanceId, 0},
                         Edge{PortNode{component.instanceId, 0}, nullptr});
        }
        break;
      default:
        break;
    }
  }
}

bool TransitionSignalDomain(const PlacedComponent& component,
                            SignalDomain input,
                            SignalDomain* output) {
  if (!output) return false;
  *output = input;
  switch (component.type) {
    case ComponentType::AUDIO_DAC:
      if (input != SignalDomain::DIGITAL) return false;
      *output = SignalDomain::ANALOG;
      return true;
    case ComponentType::AUDIO_ADC:
      if (input != SignalDomain::ANALOG) return false;
      *output = SignalDomain::DIGITAL;
      return true;
    case ComponentType::AUDIO_RESISTOR:
    case ComponentType::AUDIO_POTENTIOMETER:
    case ComponentType::AUDIO_CAPACITOR:
    case ComponentType::AUDIO_INDUCTOR:
    case ComponentType::AUDIO_DIODE:
    case ComponentType::AUDIO_BJT_NPN:
    case ComponentType::AUDIO_BJT_PNP:
    case ComponentType::AUDIO_OP_AMP:
      return input == SignalDomain::ANALOG;
    case ComponentType::RTL_MODULE:
      if (component.rtlModuleId == "audio_shell_dac") {
        if (input != SignalDomain::DIGITAL) return false;
        *output = SignalDomain::ANALOG;
        return true;
      }
      if (component.rtlModuleId == "audio_shell_adc") {
        if (input != SignalDomain::ANALOG) return false;
        *output = SignalDomain::DIGITAL;
        return true;
      }
      if (component.rtlModuleId == "audio_shell_amplifier") {
        return input == SignalDomain::ANALOG;
      }
      // General Verilog components operate in the digital domain.
      return input == SignalDomain::DIGITAL;
    default:
      return true;
  }
}

std::array<float, 10> BuildEqFrequencies(
    const std::vector<PlacedComponent>& components) {
  const PlacedComponent* sweep_source = nullptr;
  for (const PlacedComponent& component : components) {
    if (component.type == ComponentType::AUDIO_AC_SOURCE) {
      sweep_source = &component;
      break;
    }
    if (!sweep_source &&
        component.type == ComponentType::AUDIO_PULSE_SOURCE) {
      sweep_source = &component;
    }
  }
  if (!sweep_source) return kAudioEqFrequenciesHz;
  const float start = std::clamp(
      State(*sweep_source, state_keys::kFrequencyMinHz, 20.0f),
      0.01f, 1000000.0f);
  const float end = std::clamp(
      State(*sweep_source, state_keys::kFrequencyMaxHz, 20000.0f),
      start, 1000000.0f);
  std::array<float, 10> frequencies{};
  const float ratio = std::pow(end / start,
                               1.0f / static_cast<float>(frequencies.size() - 1));
  frequencies[0] = start;
  for (size_t index = 1; index < frequencies.size(); ++index) {
    frequencies[index] = frequencies[index - 1] * ratio;
  }
  frequencies.back() = end;
  return frequencies;
}

void ApplyComponentResponse(const PlacedComponent& component,
                             std::array<float, 10>* magnitude,
                             float* noise_floor_db, float* thd_percent,
                             float load_ohms,
                             const std::array<float, 10>& frequencies,
                             std::vector<AudioFilterStage>* filter_stages) {
  if (filter_stages && component.type == ComponentType::AUDIO_CAPACITOR) {
    const float capacitance =
        std::max(0.000001f,
                 State(component, state_keys::kCapacitanceUf, 10.0f)) *
        0.000001f;
    const float cutoff = 1.0f / (2.0f * kPi * load_ohms * capacitance);
    const float reference_frequency = frequencies[5];
    const float reference_gain = reference_frequency /
        std::sqrt(reference_frequency * reference_frequency + cutoff * cutoff);
    filter_stages->push_back({AudioFilterType::HIGH_PASS, cutoff,
                              reference_gain, component.instanceId});
  } else if (filter_stages &&
             component.type == ComponentType::AUDIO_INDUCTOR) {
    const float inductance =
        std::max(0.000001f,
                 State(component, state_keys::kInductanceMh, 10.0f)) *
        0.001f;
    const float cutoff = load_ohms / (2.0f * kPi * inductance);
    const float reference_frequency = frequencies[5];
    const float reference_gain = cutoff /
        std::sqrt(reference_frequency * reference_frequency + cutoff * cutoff);
    filter_stages->push_back({AudioFilterType::LOW_PASS, cutoff,
                              reference_gain, component.instanceId});
  } else if (filter_stages && IsVoltageAmplifier(component)) {
    const float gain =
        std::max(0.01f, State(component, state_keys::kGain, 4.0f));
    const float gain_bandwidth = std::max(
        1.0f, State(component, state_keys::kGainBandwidthHz, 100000.0f));
    const float cutoff = gain_bandwidth / std::max(gain, 1.0f);
    const float reference_frequency = frequencies[5];
    const float reference_gain = cutoff /
        std::sqrt(reference_frequency * reference_frequency + cutoff * cutoff);
    filter_stages->push_back({AudioFilterType::LOW_PASS, cutoff,
                              reference_gain, component.instanceId});
  }
  for (size_t band = 0; band < magnitude->size(); ++band) {
    const float frequency = frequencies[band];
    float transfer = 1.0f;
    switch (component.type) {
      case ComponentType::AUDIO_RESISTOR: {
        const float resistance = std::max(
            0.0f, State(component, state_keys::kResistanceOhms, 1000.0f));
        transfer = load_ohms / (load_ohms + resistance);
        break;
      }
      case ComponentType::AUDIO_POTENTIOMETER: {
        const float wiper = std::clamp(
            State(component, state_keys::kWiperPosition, 0.5f), 0.0f, 1.0f);
        const float resistance = std::max(
            0.0f, State(component, state_keys::kResistanceOhms, 10000.0f));
        const float output_resistance = resistance * wiper * (1.0f - wiper);
        transfer = wiper * load_ohms / (load_ohms + output_resistance);
        break;
      }
      case ComponentType::AUDIO_CAPACITOR: {
        const float capacitance =
            std::max(0.000001f,
                     State(component, state_keys::kCapacitanceUf, 10.0f)) *
            0.000001f;
        const float reactance = 1.0f / (2.0f * kPi * frequency * capacitance);
        transfer = load_ohms /
                   std::sqrt(load_ohms * load_ohms + reactance * reactance);
        break;
      }
      case ComponentType::AUDIO_INDUCTOR: {
        const float inductance =
            std::max(0.000001f,
                     State(component, state_keys::kInductanceMh, 10.0f)) *
            0.001f;
        const float reactance = 2.0f * kPi * frequency * inductance;
        transfer = load_ohms /
                   std::sqrt(load_ohms * load_ohms + reactance * reactance);
        break;
      }
      case ComponentType::AUDIO_OP_AMP:
      case ComponentType::RTL_MODULE:
        if (IsVoltageAmplifier(component)) {
          const float gain =
              std::max(0.0f, State(component, state_keys::kGain, 4.0f));
          const float gain_bandwidth = std::max(
              1.0f,
              State(component, state_keys::kGainBandwidthHz, 100000.0f));
          const float cutoff = gain_bandwidth / std::max(gain, 1.0f);
          const float output_resistance = std::max(
              0.0f,
              State(component, state_keys::kOutputResistanceOhms, 0.1f));
          const float load_transfer =
              load_ohms / (load_ohms + output_resistance);
          const float bandwidth_transfer =
              cutoff / std::sqrt(frequency * frequency + cutoff * cutoff);
          transfer = gain * load_transfer * bandwidth_transfer;
        }
        break;
      case ComponentType::AUDIO_DIODE: {
        const float dynamic_resistance = std::max(
            0.0f, State(component, state_keys::kDynamicResistanceOhms, 2.0f));
        transfer = load_ohms / (load_ohms + dynamic_resistance);
        break;
      }
      case ComponentType::AUDIO_BJT_NPN:
      case ComponentType::AUDIO_BJT_PNP: {
        const float output_resistance = std::max(
            0.0f, State(component, state_keys::kOutputResistanceOhms, 0.2f));
        transfer = 0.98f * load_ohms / (load_ohms + output_resistance);
        break;
      }
      default:
        break;
    }
    (*magnitude)[band] *= transfer;
  }

  if (component.type == ComponentType::RTL_MODULE ||
      component.type == ComponentType::AUDIO_OP_AMP) {
    if (component.rtlModuleId == "audio_shell_adc" ||
        component.rtlModuleId == "audio_shell_dac") {
      const float bits =
          std::clamp(State(component, state_keys::kBitDepth, 16.0f), 1.0f, 32.0f);
      *noise_floor_db = std::max(*noise_floor_db, -6.02f * bits - 1.76f);
    }
    if (IsVoltageAmplifier(component)) {
      const float gain = std::max(0.0f,
                                  State(component, state_keys::kGain, 1.0f));
      const float open_loop_gain = std::max(
          1.0f, State(component, state_keys::kOpenLoopGain, 100000.0f));
      if (gain > open_loop_gain) *thd_percent += 20.0f;
      const float slew_rate = std::max(
          0.001f, State(component, state_keys::kSlewRateVoltsPerUs, 0.5f));
      if (slew_rate < 0.1f) *thd_percent += (0.1f - slew_rate) * 10.0f;
    }
  }
}

bool HasTopologyPath(const std::multimap<PortNode, Edge>& graph,
                     PortNode start, PortNode target) {
  std::queue<PortNode> queue;
  std::set<PortNode> visited;
  queue.push(start);
  visited.insert(start);
  while (!queue.empty() && visited.size() < 2048) {
    const PortNode node = queue.front();
    queue.pop();
    if (node == target) return true;
    const auto range = graph.equal_range(node);
    for (auto edge = range.first; edge != range.second; ++edge) {
      if (visited.insert(edge->second.destination).second) {
        queue.push(edge->second.destination);
      }
    }
  }
  return false;
}

bool HasWireOnlyPath(const std::multimap<PortNode, Edge>& graph,
                     PortNode start, PortNode target) {
  std::queue<PortNode> queue;
  std::set<PortNode> visited;
  queue.push(start);
  visited.insert(start);
  while (!queue.empty() && visited.size() < 2048) {
    const PortNode node = queue.front();
    queue.pop();
    if (node == target) return true;
    const auto range = graph.equal_range(node);
    for (auto edge = range.first; edge != range.second; ++edge) {
      if (edge->second.through_component != nullptr) continue;
      if (visited.insert(edge->second.destination).second) {
        queue.push(edge->second.destination);
      }
    }
  }
  return false;
}

struct EmitterLeg {
  PortNode output_node;
  const PlacedComponent* ballast_resistor = nullptr;
};

std::vector<EmitterLeg> FindEmitterLegs(
    const std::multimap<PortNode, Edge>& graph,
    const PlacedComponent& transistor) {
  const PortNode emitter{transistor.instanceId, 2};
  std::queue<PortNode> queue;
  std::set<PortNode> emitter_net;
  queue.push(emitter);
  emitter_net.insert(emitter);
  while (!queue.empty() && emitter_net.size() < 256) {
    const PortNode node = queue.front();
    queue.pop();
    const auto range = graph.equal_range(node);
    for (auto edge = range.first; edge != range.second; ++edge) {
      if (edge->second.through_component != nullptr) continue;
      if (emitter_net.insert(edge->second.destination).second) {
        queue.push(edge->second.destination);
      }
    }
  }

  std::vector<EmitterLeg> legs;
  for (const PortNode& node : emitter_net) {
    const auto range = graph.equal_range(node);
    for (auto edge = range.first; edge != range.second; ++edge) {
      const PlacedComponent* component = edge->second.through_component;
      if (!component ||
          component->type != ComponentType::AUDIO_RESISTOR) {
        continue;
      }
      const auto duplicate = std::find_if(
          legs.begin(), legs.end(), [&](const EmitterLeg& leg) {
            return leg.ballast_resistor->instanceId == component->instanceId;
          });
      if (duplicate == legs.end()) {
        legs.push_back({edge->second.destination, component});
      }
    }
  }
  return legs;
}

bool HasLocalDriverBasePath(
    const std::multimap<PortNode, Edge>& graph,
    const std::vector<PlacedComponent>& components, PortNode driver_output,
    PortNode transistor_base) {
  if (HasWireOnlyPath(graph, driver_output, transistor_base)) return true;
  for (const PlacedComponent& component : components) {
    if (component.type != ComponentType::AUDIO_DIODE) continue;
    const PortNode anode{component.instanceId, 0};
    const PortNode cathode{component.instanceId, 1};
    // A reversed diode still identifies the local driver, allowing the bias
    // checker to report the orientation/voltage fault instead of pretending
    // that the entire output stage is absent.
    if ((HasWireOnlyPath(graph, driver_output, anode) &&
         HasWireOnlyPath(graph, cathode, transistor_base)) ||
        (HasWireOnlyPath(graph, driver_output, cathode) &&
         HasWireOnlyPath(graph, anode, transistor_base))) {
      return true;
    }
  }
  return false;
}

std::map<int, float> BuildShuntLowPassMap(
    const std::vector<PlacedComponent>& components,
    const std::multimap<PortNode, Edge>& graph,
    const std::set<int>& ground_component_ids) {
  std::map<int, float> result;
  for (const PlacedComponent& resistor : components) {
    if (resistor.type != ComponentType::AUDIO_RESISTOR) continue;
    for (const PlacedComponent& capacitor : components) {
      if (capacitor.type != ComponentType::AUDIO_CAPACITOR) continue;
      for (int resistor_port = 0; resistor_port < 2; ++resistor_port) {
        for (int capacitor_signal_port = 0; capacitor_signal_port < 2;
             ++capacitor_signal_port) {
          if (!HasWireOnlyPath(graph,
                               {resistor.instanceId, resistor_port},
                               {capacitor.instanceId,
                                capacitor_signal_port})) {
            continue;
          }
          const int capacitor_ground_port = 1 - capacitor_signal_port;
          bool grounded = false;
          for (int ground_id : ground_component_ids) {
            if (HasWireOnlyPath(
                    graph,
                    {capacitor.instanceId, capacitor_ground_port},
                    {ground_id, 0})) {
              grounded = true;
              break;
            }
          }
          if (!grounded) continue;
          const float resistance = std::max(
              0.001f,
              State(resistor, state_keys::kResistanceOhms, 1000.0f));
          const float capacitance = std::max(
              0.000000000001f,
              State(capacitor, state_keys::kCapacitanceUf, 0.0082f) *
                  0.000001f);
          result[resistor.instanceId] =
              1.0f / (2.0f * kPi * resistance * capacitance);
        }
      }
    }
  }
  return result;
}

std::map<int, AmplifierPowerInfo> BuildAmplifierPowerMap(
    const std::vector<PlacedComponent>& components,
    const std::multimap<PortNode, Edge>& graph) {
  std::map<int, AmplifierPowerInfo> result;
  // First validate the low-current voltage amplifier/driver supply.
  for (const PlacedComponent& amplifier : components) {
    if (!IsVoltageAmplifier(amplifier)) continue;
    int power_port = -1;
    int ground_port = -1;
    if (amplifier.type == ComponentType::AUDIO_OP_AMP) {
      power_port = 2;
      ground_port = 3;
    } else {
      const RtlPinBinding* power =
          FindRtlBinding(amplifier, amplifier.rtlPowerPinName);
      const RtlPinBinding* ground =
          FindRtlBinding(amplifier, amplifier.rtlGroundPinName);
      if (power) power_port = power->portId;
      if (ground) ground_port = ground->portId;
    }
    AmplifierPowerInfo best;
    if (power_port >= 0 && ground_port >= 0) {
      for (const PlacedComponent& supply : components) {
        if (supply.type != ComponentType::AUDIO_DC_SOURCE) continue;
        if (!HasWireOnlyPath(graph, {supply.instanceId, 0},
                             {amplifier.instanceId, power_port}) ||
            !HasWireOnlyPath(graph, {supply.instanceId, 1},
                             {amplifier.instanceId, ground_port})) {
          continue;
        }
        AmplifierPowerInfo candidate;
        candidate.supply_voltage =
            std::abs(State(supply, state_keys::kVoltageVolts, 0.0f));
        candidate.current_limit_amps = std::max(
            0.0f, State(supply, state_keys::kCurrentLimitAmps, 0.0f));
        const bool supply_shorted = HasWireOnlyPath(
            graph, {supply.instanceId, 0}, {supply.instanceId, 1});
        candidate.powered = !supply_shorted &&
                            candidate.supply_voltage > 2.0f &&
                            candidate.current_limit_amps > 0.0f;
        if (candidate.supply_voltage * candidate.current_limit_amps >
            best.supply_voltage * best.current_limit_amps) {
          best = candidate;
        }
      }
    }
    result[amplifier.instanceId] = best;
  }

  // A speaker-capable class-AB stage is a complementary emitter follower:
  // powered NPN + PNP collectors, joined emitters, and bases driven by a
  // powered voltage amplifier.  An op-amp alone deliberately never satisfies
  // the power-output requirement.
  for (const PlacedComponent& npn : components) {
    if (npn.type != ComponentType::AUDIO_BJT_NPN) continue;
    for (const PlacedComponent& pnp : components) {
      if (pnp.type != ComponentType::AUDIO_BJT_PNP) continue;
      bool emitters_joined = HasWireOnlyPath(
          graph, {npn.instanceId, 2}, {pnp.instanceId, 2});
      bool emitter_ballast_present = false;
      float emitter_ballast_ohms = 0.0f;
      if (!emitters_joined) {
        const std::vector<EmitterLeg> npn_legs = FindEmitterLegs(graph, npn);
        const std::vector<EmitterLeg> pnp_legs = FindEmitterLegs(graph, pnp);
        for (const EmitterLeg& npn_leg : npn_legs) {
          for (const EmitterLeg& pnp_leg : pnp_legs) {
            if (!HasWireOnlyPath(graph, npn_leg.output_node,
                                 pnp_leg.output_node)) {
              continue;
            }
            emitters_joined = true;
            const float npn_ballast = State(
                *npn_leg.ballast_resistor,
                state_keys::kResistanceOhms, 0.22f);
            const float pnp_ballast = State(
                *pnp_leg.ballast_resistor,
                state_keys::kResistanceOhms, 0.22f);
            emitter_ballast_ohms = std::min(npn_ballast, pnp_ballast);
            emitter_ballast_present =
                npn_ballast >= 0.1f && npn_ballast <= 0.47f &&
                pnp_ballast >= 0.1f && pnp_ballast <= 0.47f;
            break;
          }
          if (emitters_joined) break;
        }
      }
      if (!emitters_joined) continue;
      for (const PlacedComponent& supply : components) {
        if (supply.type != ComponentType::AUDIO_DC_SOURCE ||
            !HasWireOnlyPath(graph, {supply.instanceId, 0},
                             {npn.instanceId, 1}) ||
            !HasWireOnlyPath(graph, {supply.instanceId, 1},
                             {pnp.instanceId, 1})) {
          continue;
        }
        bool driven = false;
        float driver_current_limit = 0.0f;
        PortNode driver_output{};
        for (const PlacedComponent& driver : components) {
          if (!IsVoltageAmplifier(driver)) continue;
          const auto driver_power = result.find(driver.instanceId);
          if (driver_power == result.end() || !driver_power->second.powered) {
            continue;
          }
          int output_port = 4;
          if (driver.type == ComponentType::RTL_MODULE) {
            const auto output = std::find_if(
                driver.rtlPinBindings.begin(), driver.rtlPinBindings.end(),
                [](const RtlPinBinding& binding) { return !binding.isInput; });
            if (output == driver.rtlPinBindings.end()) continue;
            output_port = output->portId;
          }
          driver_output = {driver.instanceId, output_port};
          if (HasLocalDriverBasePath(
                  graph, components, driver_output, {npn.instanceId, 0}) &&
              HasLocalDriverBasePath(
                  graph, components, driver_output, {pnp.instanceId, 0})) {
            driven = true;
            driver_current_limit = std::max(
                0.0f, State(driver, state_keys::kOutputCurrentLimitAmps,
                            0.025f));
            break;
          }
        }
        if (!driven) continue;

        AmplifierPowerInfo stage;
        stage.supply_voltage =
            std::abs(State(supply, state_keys::kVoltageVolts, 0.0f));
        const float transistor_current = std::min(
            std::max(0.0f, State(npn,
                state_keys::kMaximumCollectorCurrentAmps, 1.5f)),
            std::max(0.0f, State(pnp,
                state_keys::kMaximumCollectorCurrentAmps, 1.5f)));
        const float beta = std::min(
            std::max(1.0f, State(npn, state_keys::kCurrentGainBeta, 100.0f)),
            std::max(1.0f, State(pnp, state_keys::kCurrentGainBeta, 100.0f)));
        stage.current_limit_amps = std::min({
            std::max(0.0f,
                     State(supply, state_keys::kCurrentLimitAmps, 0.0f)),
            transistor_current, driver_current_limit * beta});
        stage.powered = !HasWireOnlyPath(
                            graph, {supply.instanceId, 0},
                            {supply.instanceId, 1}) &&
                        stage.supply_voltage > 2.0f &&
                        stage.current_limit_amps > 0.0f;
        stage.emitter_ballast_present = emitter_ballast_present;
        stage.emitter_ballast_ohms = emitter_ballast_ohms;
        for (const PlacedComponent& component : components) {
          if (component.type != ComponentType::AUDIO_DIODE) continue;
          // Bias devices belong to this output pair only when both sides are
          // on the local wire nets. A general topology search can walk back
          // through passive networks, supply/ground, and the other stereo
          // channel, incorrectly counting every bias diode in the project.
          if (HasWireOnlyPath(graph, driver_output,
                              {component.instanceId, 0}) &&
              (HasWireOnlyPath(graph, {component.instanceId, 1},
                               {npn.instanceId, 0}) ||
               HasWireOnlyPath(graph, {component.instanceId, 1},
                               {pnp.instanceId, 0}))) {
            ++stage.bias_diode_count;
            if (State(component, state_keys::kThermalCoupling, 0.0f) >=
                0.5f) {
              ++stage.thermally_coupled_bias_diode_count;
            }
            stage.bias_voltage += std::max(
                0.0f, State(component, state_keys::kForwardVoltageVolts,
                            0.65f));
          }
        }
        result[npn.instanceId] = stage;
        result[pnp.instanceId] = stage;
      }
    }
  }
  return result;
}

AudioChannelStatus EvaluateChannel(
    const std::multimap<PortNode, Edge>& graph, PortNode signal_start,
    PortNode signal_target, PortNode return_start, PortNode return_target,
    float load_ohms, const std::array<float, 10>& frequencies,
    const std::set<int>& ground_component_ids,
    const std::map<int, AmplifierPowerInfo>& amplifier_power,
    const std::set<int>& dac_network_resistor_ids,
    const std::map<int, int>& dac_driven_resistor_ports,
    const std::map<int, float>& shunt_low_pass_cutoffs) {
  AudioChannelStatus result;
  result.eq_db.fill(kSilenceDb);
  result.return_complete =
      HasTopologyPath(graph, return_start, return_target);
  PathState initial;
  initial.node = signal_start;
  initial.magnitude.fill(1.0f);
  initial.visited.insert(
      {signal_start.first, signal_start.second,
       static_cast<int>(initial.domain)});
  std::vector<PathState> queue{initial};
  using SearchStateKey = std::tuple<int, int, int, bool, bool, bool>;
  std::map<SearchStateKey, float> best_expanded_level;
  float best_level = -1.0f;
  for (size_t cursor = 0; cursor < queue.size() && cursor < 2048; ++cursor) {
    PathState state = queue[cursor];
    const SearchStateKey search_key{
        state.node.first, state.node.second, static_cast<int>(state.domain),
        state.has_powered_amplifier, state.has_dc_blocking_capacitor,
        state.has_emitter_ballast};
    const float search_level = state.magnitude[5];
    const auto expanded = best_expanded_level.find(search_key);
    if (expanded != best_expanded_level.end() &&
        search_level <= expanded->second + 0.000001f) {
      continue;
    }
    best_expanded_level[search_key] = search_level;
    if (state.node == signal_target && state.domain == SignalDomain::ANALOG) {
      if (!state.has_powered_amplifier) continue;
      const float reference = state.magnitude[5];
      if (reference > best_level) {
        best_level = reference;
        result.route_complete = true;
        float combined_filter_reference_gain = 1.0f;
        for (const AudioFilterStage& filter : state.filter_stages) {
          combined_filter_reference_gain *=
              std::clamp(filter.reference_gain, 0.000001f, 1.0f);
        }
        const float flat_path_gain =
            reference / std::max(combined_filter_reference_gain, 0.000001f);
        // Preserve the real requested path gain for rail/current clipping.
        // The user safety-volume mapping clamps this value later, but clipping
        // must still see a 24x/40x driver instead of silently treating every
        // high-gain amplifier as only 4x.
        result.output_scalar =
            result.return_complete ? std::clamp(flat_path_gain, 0.0f, 1000.0f)
                                   : 0.0f;
        result.output_db = LinearToDb(result.output_scalar);
        result.noise_floor_db = state.noise_floor_db;
        result.thd_percent = std::clamp(
            state.thd_percent +
                (state.has_dc_blocking_capacitor ? 0.0f : 25.0f),
            0.0f, 100.0f);
        result.filter_stages = state.filter_stages;
        result.amplifier_present = true;
        result.amplifier_powered = true;
        result.dc_blocking_capacitor_present =
            state.has_dc_blocking_capacitor;
        result.emitter_ballast_present = state.has_emitter_ballast;
        result.amplifier_bias_fault = state.has_amplifier_bias_fault;
        result.amplifier_thermal_tracking_fault =
            state.has_amplifier_thermal_tracking_fault;
        result.amplifier_bandwidth_limited =
            state.has_amplifier_bandwidth_limit;
        result.amplifier_bias_error_volts =
            state.amplifier_bias_error_volts;
        result.specialized_distortion_percent =
            state.specialized_distortion_percent;
        result.speaker_damage_risk =
            !state.has_dc_blocking_capacitor || !state.has_emitter_ballast;
        result.estimated_dc_offset_volts =
            state.has_dc_blocking_capacitor
                ? 0.0f
                : state.amplifier_supply_voltage * 0.5f;
        result.amplifier_supply_voltage = state.amplifier_supply_voltage;
        result.amplifier_current_limit_amps =
            state.amplifier_current_limit_amps;
        result.maximum_speaker_peak_voltage =
            state.maximum_speaker_peak_voltage;
        result.maximum_speaker_power_watts =
            state.maximum_speaker_peak_voltage *
            state.maximum_speaker_peak_voltage / (2.0f * load_ohms);
        for (size_t band = 0; band < result.eq_db.size(); ++band) {
          result.eq_db[band] = LinearToDb(state.magnitude[band]);
        }
      }
      continue;
    }
    const auto range = graph.equal_range(state.node);
    for (auto edge = range.first; edge != range.second; ++edge) {
      PathState child = state;
      child.node = edge->second.destination;
      // Ground is valid only for the separately evaluated speaker return
      // path.  Letting the forward signal search enter the common ground net
      // creates impossible reverse paths such as DAC -> unused output ->
      // ground -> potentiometer wiper, bypassing every series C/L component.
      if (ground_component_ids.count(child.node.first) != 0) continue;
      if (edge->second.through_component) {
        const PlacedComponent& traversed = *edge->second.through_component;
        // Walking a discrete DAC branch backwards from the shared bus re-enters all
        // of the other DAC bits and creates an exponential number of equivalent
        // paths.  Apart from being electrically meaningless for a driven DAC
        // output, that fan-out can exhaust the bounded route search before the
        // real amplifier path is visited.
        const auto driven_port =
            dac_driven_resistor_ports.find(traversed.instanceId);
        if (driven_port != dac_driven_resistor_ports.end() &&
            state.node.first == traversed.instanceId &&
            child.node.first == traversed.instanceId &&
            state.node.second != driven_port->second &&
            child.node.second == driven_port->second) {
          continue;
        }
        if (IsAudioGainDevice(*edge->second.through_component)) {
          result.amplifier_present = true;
          const auto power = amplifier_power.find(
              edge->second.through_component->instanceId);
          if (power == amplifier_power.end() || !power->second.powered) {
            continue;
          }
          if (IsOutputTransistor(*edge->second.through_component)) {
            child.has_powered_amplifier = true;
            child.amplifier_supply_voltage = power->second.supply_voltage;
            child.amplifier_current_limit_amps =
                power->second.current_limit_amps;
            child.has_emitter_ballast =
                child.has_emitter_ballast ||
                power->second.emitter_ballast_present;
            if (!power->second.emitter_ballast_present) {
              // Directly joined emitters are thermally unstable. Keep the
              // route audible for diagnosis, but model the rapidly rising
              // quiescent current as severe distortion and derating.
              child.thd_percent += 35.0f;
              child.amplifier_current_limit_amps =
                  std::min(child.amplifier_current_limit_amps, 0.1f);
            }
            const float saturation_voltage = std::max(
                0.0f, State(*edge->second.through_component,
                            state_keys::kSaturationVoltageVolts, 0.2f));
            const float voltage_limited_peak = std::max(
                0.0f, power->second.supply_voltage * 0.5f -
                          saturation_voltage);
            const float current_limited_peak =
                child.amplifier_current_limit_amps * load_ohms;
            const float maximum_power = std::max(
                0.0f, State(*edge->second.through_component,
                            state_keys::kMaximumPowerWatts, 1.0f));
            const float thermal_resistance = std::max(
                0.001f, State(*edge->second.through_component,
                              state_keys::kThermalResistanceCPerW, 62.5f));
            const float thermally_derated_power =
                std::min(maximum_power, 75.0f / thermal_resistance);
            const float thermal_limited_peak =
                std::sqrt(2.0f * thermally_derated_power * load_ohms);
            child.maximum_speaker_peak_voltage =
                std::min(child.maximum_speaker_peak_voltage,
                         std::min({voltage_limited_peak,
                                   current_limited_peak,
                                   thermal_limited_peak}));
            // Under-biased followers expose a dead zone; over-biasing also
            // adds distortion/heating. Two silicon drops is the nominal AB
            // bias point for the complementary pair.
            const float target_bias =
                std::max(0.0f,
                    State(*edge->second.through_component,
                          state_keys::kBaseEmitterVoltageVolts, 0.65f)) *
                2.0f;
            child.amplifier_bias_error_volts =
                power->second.bias_voltage - target_bias;
            if (power->second.bias_diode_count < 2) {
              child.has_amplifier_bias_fault = true;
              const float bias_distortion =
                  static_cast<float>(2 - power->second.bias_diode_count) *
                  3.5f;
              child.thd_percent += bias_distortion;
              child.specialized_distortion_percent += bias_distortion;
            } else if (power->second.bias_diode_count > 2) {
              child.has_amplifier_bias_fault = true;
              const float bias_distortion =
                  static_cast<float>(power->second.bias_diode_count - 2) *
                  1.0f;
              child.thd_percent += bias_distortion;
              child.specialized_distortion_percent += bias_distortion;
            } else {
              if (std::abs(power->second.bias_voltage - target_bias) >
                  0.15f) {
                child.has_amplifier_bias_fault = true;
              }
              const float bias_distortion = 0.15f +
                  std::abs(power->second.bias_voltage - target_bias) * 4.0f;
              child.thd_percent += bias_distortion;
              child.specialized_distortion_percent += bias_distortion;
            }
            if (power->second.thermally_coupled_bias_diode_count < 2) {
              child.has_amplifier_thermal_tracking_fault = true;
              child.thd_percent += 4.0f;
              child.specialized_distortion_percent += 4.0f;
              child.amplifier_current_limit_amps *= 0.75f;
            }
          }
          const float gain = std::max(
              0.01f, State(*edge->second.through_component,
                           state_keys::kGain, 1.0f));
          const float gain_bandwidth = std::max(
              1.0f, State(*edge->second.through_component,
                          state_keys::kGainBandwidthHz, 1000000.0f));
          if (IsVoltageAmplifier(*edge->second.through_component) &&
              gain_bandwidth / std::max(gain, 1.0f) < 20000.0f) {
            child.has_amplifier_bandwidth_limit = true;
          }
        }
        SignalDomain next_domain = child.domain;
        if (!TransitionSignalDomain(*edge->second.through_component,
                                    child.domain, &next_domain)) {
          continue;
        }
        child.domain = next_domain;
      }
      const auto visit_key =
          std::make_tuple(child.node.first, child.node.second,
                          static_cast<int>(child.domain));
      if (state.visited.count(visit_key)) continue;
      child.visited.insert(visit_key);
      if (edge->second.through_component) {
        const PlacedComponent& traversed = *edge->second.through_component;
        if (traversed.type == ComponentType::AUDIO_CAPACITOR &&
            child.has_powered_amplifier &&
            State(traversed, state_keys::kCapacitanceUf, 0.0f) >= 1000.0f) {
          child.has_dc_blocking_capacitor = true;
        }
        const bool is_dac_network_resistor =
            traversed.type == ComponentType::AUDIO_RESISTOR &&
            dac_network_resistor_ids.count(traversed.instanceId) != 0;
        if (!is_dac_network_resistor) {
          const auto shunt_filter =
              shunt_low_pass_cutoffs.find(traversed.instanceId);
          if (shunt_filter != shunt_low_pass_cutoffs.end()) {
            const float cutoff = shunt_filter->second;
            const float reference_frequency = frequencies[5];
            const float reference_gain = cutoff / std::sqrt(
                reference_frequency * reference_frequency +
                cutoff * cutoff);
            child.filter_stages.push_back(
                {AudioFilterType::LOW_PASS, cutoff, reference_gain,
                 traversed.instanceId});
            for (size_t band = 0; band < child.magnitude.size(); ++band) {
              const float frequency = frequencies[band];
              child.magnitude[band] *= cutoff / std::sqrt(
                  frequency * frequency + cutoff * cutoff);
            }
          } else {
            const float effective_load =
                child.has_powered_amplifier ? load_ohms : 10000.0f;
            ApplyComponentResponse(traversed, &child.magnitude,
                                   &child.noise_floor_db,
                                   &child.thd_percent, effective_load,
                                   frequencies, &child.filter_stages);
          }
        }
      }
      queue.push_back(std::move(child));
    }
  }
  if (!result.route_complete || !result.return_complete) {
    result.output_scalar = 0.0f;
    result.output_db = kSilenceDb;
  }
  return result;
}

AudioCircuitStatus Evaluate(std::vector<PlacedComponent>* components,
                            const std::vector<Wire>& wires) {
  AudioCircuitStatus result;
  if (!components) return result;
  const AudioTopologyIndex topology_index =
      BuildTopologyIndex(*components, wires);
  result.eq_frequencies_hz = BuildEqFrequencies(*components);
  const PlacedComponent* source = nullptr;
  const PlacedComponent* speaker = nullptr;
  const PlacedComponent* ground = nullptr;
  bool dac_present = false;
  bool dac_ready = false;
  bool dac_bus_wired = false;
  const PlacedComponent* functional_rtl_dac = nullptr;
  DacBusResponse left_dac_bus;
  DacBusResponse right_dac_bus;
  for (const PlacedComponent& component : *components) {
    if (!source && component.type == ComponentType::AUDIO_SOURCE) {
      source = &component;
    }
    if (!speaker && component.type == ComponentType::AUDIO_SPEAKER) {
      speaker = &component;
    }
    if (!ground && component.type == ComponentType::AUDIO_GROUND) {
      ground = &component;
    }
    if (component.type == ComponentType::AUDIO_DAC ||
        (component.type == ComponentType::RTL_MODULE &&
         component.rtlModuleId == "audio_shell_dac")) {
      dac_present = true;
      dac_ready = dac_ready || component.type == ComponentType::AUDIO_DAC ||
                  HasFunctionalStereoDacInterface(component);
      if (component.type == ComponentType::AUDIO_DAC) {
        dac_bus_wired = true;
        left_dac_bus.connected_mask = 0xffffU;
        left_dac_bus.bit_weights.fill(1.0f);
        left_dac_bus.connected_bits = 16;
        left_dac_bus.complete = true;
        right_dac_bus = left_dac_bus;
      } else if (HasFunctionalStereoDacInterface(component)) {
        if (!functional_rtl_dac) functional_rtl_dac = &component;
        const DacBusResponse candidate_left = BuildDacBusResponse(
            component, "dac_l", topology_index);
        const DacBusResponse candidate_right = BuildDacBusResponse(
            component, "dac_r", topology_index);
        if (candidate_left.connected_bits > left_dac_bus.connected_bits) {
          left_dac_bus = candidate_left;
        }
        if (candidate_right.connected_bits > right_dac_bus.connected_bits) {
          right_dac_bus = candidate_right;
        }
        dac_bus_wired = dac_bus_wired ||
                        candidate_left.connected_mask != 0 ||
                        candidate_right.connected_mask != 0;
      }
    }
  }
  if (!source || !speaker || !ground) {
    result.diagnosis =
        "PC digital source, stereo speaker, and analog ground are required.";
    return result;
  }

  std::multimap<PortNode, Edge> graph;
  BuildPortGraph(*components, wires, &graph);
  std::set<int> ground_component_ids;
  for (const PlacedComponent& component : *components) {
    if (component.type == ComponentType::AUDIO_GROUND) {
      ground_component_ids.insert(component.instanceId);
    }
  }
  const std::map<int, AmplifierPowerInfo> amplifier_power =
      BuildAmplifierPowerMap(*components, graph);
  const bool dc_supply_shorted = std::any_of(
      components->begin(), components->end(),
      [&](const PlacedComponent& component) {
        return component.type == ComponentType::AUDIO_DC_SOURCE &&
               HasWireOnlyPath(graph, {component.instanceId, 0},
                               {component.instanceId, 1});
      });
  const std::set<int> dac_network_resistor_ids =
      BuildDacNetworkResistorIds(*components, wires, topology_index);
  const std::map<int, int> dac_driven_resistor_ports =
      BuildDacDrivenResistorPorts(*components, topology_index);
  const std::map<int, float> shunt_low_pass_cutoffs =
      BuildShuntLowPassMap(*components, graph, ground_component_ids);
  const float load_ohms =
      std::max(0.1f, State(*speaker, state_keys::kImpedanceOhms, 8.0f));
  const float speaker_rated_power_watts = std::max(
      0.1f, State(*speaker, state_keys::kMaximumPowerWatts, 100.0f));
  result.left = EvaluateChannel(
      graph, {source->instanceId, 0}, {speaker->instanceId, 0},
      {speaker->instanceId, 1}, {ground->instanceId, 0}, load_ohms,
      result.eq_frequencies_hz, ground_component_ids, amplifier_power,
      dac_network_resistor_ids, dac_driven_resistor_ports,
      shunt_low_pass_cutoffs);
  result.right = EvaluateChannel(
      graph, {source->instanceId, 1}, {speaker->instanceId, 2},
      {speaker->instanceId, 3}, {ground->instanceId, 0}, load_ohms,
      result.eq_frequencies_hz, ground_component_ids, amplifier_power,
      dac_network_resistor_ids, dac_driven_resistor_ports,
      shunt_low_pass_cutoffs);
  ApplyTransducerResponse(*speaker, result.eq_frequencies_hz, &result.left);
  ApplyTransducerResponse(*speaker, result.eq_frequencies_hz, &result.right);
  // A 16-bit stereo I2S stream with 32-bit slots has a nominal 3.072 MHz
  // bit clock at 48 kHz. Deliberately clocking the DAC at another rate is
  // exposed as an audible pitch ratio instead of silently being normalized.
  if (functional_rtl_dac) {
    constexpr float kNominalI2sBitClockHz = 48000.0f * 2.0f * 32.0f;
    result.pitch_shift_ratio = std::clamp(
        functional_rtl_dac->rtlClockFrequencyHz / kNominalI2sBitClockHz,
        0.5f, 2.5f);
  }
  result.left.speaker_rated_power_watts = speaker_rated_power_watts;
  result.right.speaker_rated_power_watts = speaker_rated_power_watts;
  result.left.amplifier_power_insufficient =
      result.left.route_complete &&
      result.left.maximum_speaker_power_watts <
          speaker_rated_power_watts * 0.5f;
  result.right.amplifier_power_insufficient =
      result.right.route_complete &&
      result.right.maximum_speaker_power_watts <
          speaker_rated_power_watts * 0.5f;
  result.left.dac_connected_mask = left_dac_bus.connected_mask;
  result.left.dac_bus_complete = left_dac_bus.complete;
  result.left.dac_bit_weights = left_dac_bus.bit_weights;
  result.right.dac_connected_mask = right_dac_bus.connected_mask;
  result.right.dac_bus_complete = right_dac_bus.complete;
  result.right.dac_bit_weights = right_dac_bus.bit_weights;
  const auto apply_dac_quality = [](const DacBusResponse& bus,
                                    AudioChannelStatus* channel) {
    if (!channel || bus.connected_bits == 0) return;
    float weighted_error = 0.0f;
    for (size_t bit = 0; bit < bus.bit_weights.size(); ++bit) {
      const float significance =
          static_cast<float>(std::uint32_t{1} << bit) / 65535.0f;
      if ((bus.connected_mask & (std::uint16_t{1} << bit)) == 0) {
        weighted_error += significance;
      } else {
        weighted_error +=
            significance * std::abs(bus.bit_weights[bit] - 1.0f);
      }
    }
    channel->thd_percent = std::clamp(
        channel->thd_percent + weighted_error * 100.0f, 0.0f, 100.0f);
    channel->dac_weight_error_percent = weighted_error * 100.0f;
    channel->noise_floor_db =
        std::max(channel->noise_floor_db,
                 -6.02f * static_cast<float>(bus.connected_bits) - 1.76f);
  };
  apply_dac_quality(left_dac_bus, &result.left);
  apply_dac_quality(right_dac_bus, &result.right);
  result.route_complete =
      result.left.route_complete || result.right.route_complete;
  result.return_complete =
      (!result.left.route_complete || result.left.return_complete) &&
      (!result.right.route_complete || result.right.return_complete);
  result.output_scalar =
      std::max(result.left.output_scalar, result.right.output_scalar);
  result.output_db = LinearToDb(result.output_scalar);
  result.noise_floor_db =
      std::max(result.left.noise_floor_db, result.right.noise_floor_db);
  result.thd_percent =
      std::max(result.left.thd_percent, result.right.thd_percent);
  if (!result.route_complete) {
    if (!dac_present) {
      result.diagnosis =
          "PC output is digital. Add a DAC before the analog circuit.";
    } else if (!dac_ready) {
      result.diagnosis =
          "The DAC has not been analyzed and built with Verilator.";
    } else if (!dac_bus_wired) {
      result.diagnosis =
          "Wire all 16 left and 16 right DAC bus bits through resistors to build the discrete R-2R network.";
    } else if (dc_supply_shorted) {
      result.diagnosis =
          "DC supply short: a +V pin is connected to the return/ground net.";
    } else if (std::none_of(
                   components->begin(), components->end(),
                   [](const PlacedComponent& component) {
                     return IsOutputTransistor(component);
                   })) {
      result.diagnosis =
          "A DAC/op-amp is line-level only. Add complementary NPN and PNP output transistors before the speaker.";
    } else if (std::none_of(
                   amplifier_power.begin(), amplifier_power.end(),
                   [&](const auto& entry) {
                     const PlacedComponent* component =
                         FindComponentById(topology_index, entry.first);
                     return component && IsOutputTransistor(*component) &&
                            entry.second.powered;
                   })) {
      result.diagnosis =
          "Build a complementary class-AB stage: joined emitters, NPN collector to +V, PNP collector to return, and both bases driven by a powered op-amp.";
    } else {
      result.diagnosis =
          "No powered DAC-to-amplifier-to-speaker route reaches the load.";
    }
  } else if (!result.return_complete) {
    result.diagnosis = "A speaker return path is open. That channel is muted.";
  } else if ((result.left.route_complete &&
              result.left.speaker_damage_risk) ||
             (result.right.route_complete &&
              result.right.speaker_damage_risk)) {
    result.diagnosis =
        "Unsafe single-supply output stage: add 0.1-0.47 ohm emitter ballast resistors and a 1000 uF or larger series speaker coupling capacitor per channel.";
  } else if (!result.left.route_complete || !result.right.route_complete) {
    result.diagnosis = "Only one speaker channel is connected.";
  } else if (!result.left.dac_bus_complete ||
             !result.right.dac_bus_complete) {
    const auto count_bits = [](std::uint16_t mask) {
      int count = 0;
      while (mask != 0) {
        count += mask & 1U;
        mask = static_cast<std::uint16_t>(mask >> 1U);
      }
      return count;
    };
    result.diagnosis =
        "Incomplete discrete DAC bus: L " +
        std::to_string(count_bits(result.left.dac_connected_mask)) +
        "/16, R " +
        std::to_string(count_bits(result.right.dac_connected_mask)) +
        "/16. Missing and mis-valued branches are audible.";
  } else if (result.left.dac_weight_error_percent > 0.5f ||
             result.right.dac_weight_error_percent > 0.5f) {
    result.diagnosis =
        "DAC branches are connected, but their resistor ratios corrupt the bit weights. Check ladder values and tolerance.";
  } else if (result.left.amplifier_power_insufficient ||
             result.right.amplifier_power_insufficient) {
    result.diagnosis =
        "Amplifier power is far below the speaker rating. Supply voltage/current limits will cause severe rail clipping.";
  } else if (result.left.amplifier_bias_fault ||
             result.right.amplifier_bias_fault ||
             result.left.amplifier_thermal_tracking_fault ||
             result.right.amplifier_thermal_tracking_fault) {
    result.diagnosis =
        "Class-AB bias is incorrect or not thermally tracked. Expect crossover distortion or rising idle current.";
  } else if (result.left.amplifier_bandwidth_limited ||
             result.right.amplifier_bandwidth_limited) {
    result.diagnosis =
        "A voltage driver has less than 20 kHz closed-loop bandwidth. High frequencies are being removed.";
  } else {
    float stereo_mismatch_db = 0.0f;
    for (size_t band = 0; band < result.left.eq_db.size(); ++band) {
      stereo_mismatch_db = std::max(
          stereo_mismatch_db,
          std::abs(result.left.eq_db[band] - result.right.eq_db[band]));
    }
    if (stereo_mismatch_db > 1.0f) {
      result.diagnosis =
          "Stereo component mismatch exceeds 1 dB; use identical left/right filter values and tolerances.";
    } else if (result.thd_percent > 5.0f) {
      result.diagnosis =
          "The route is complete, but modeled distortion exceeds 5%. Inspect DAC accuracy, bias, slew rate, and output loading.";
    } else {
      result.diagnosis =
          "DC-blocked, emitter-stabilized stereo class-AB circuits are active.";
    }
  }
  return result;
}

float MapOutputRange(float scalar, float minimum_percent,
                     float maximum_percent) {
  if (!std::isfinite(scalar) || scalar <= 0.0f) return 0.0f;
  const float safe_maximum_percent =
      std::isfinite(maximum_percent)
          ? std::clamp(maximum_percent, 0.0f, 100.0f)
          : 0.0f;
  const float safe_minimum_percent =
      std::isfinite(minimum_percent)
          ? std::clamp(minimum_percent, 0.0f, safe_maximum_percent)
          : 0.0f;
  const float minimum = safe_minimum_percent / 100.0f;
  const float maximum = safe_maximum_percent / 100.0f;
  return std::clamp(minimum + std::clamp(scalar, 0.0f, 1.0f) *
                                  (maximum - minimum),
                    0.0f, maximum);
}

bool SameFilterStages(const std::vector<AudioFilterStage>& left,
                      const std::vector<AudioFilterStage>& right) {
  if (left.size() != right.size()) return false;
  for (size_t index = 0; index < left.size(); ++index) {
    if (left[index].type != right[index].type ||
        left[index].component_instance_id !=
            right[index].component_instance_id ||
        std::abs(left[index].cutoff_hz - right[index].cutoff_hz) > 0.001f ||
        std::abs(left[index].reference_gain - right[index].reference_gain) >
            0.000001f ||
        std::abs(left[index].gain_db - right[index].gain_db) > 0.000001f ||
        std::abs(left[index].q - right[index].q) > 0.000001f) {
      return false;
    }
  }
  return true;
}

void PreparePassiveFilter(const AudioFilterStage& stage,
                          unsigned int sample_rate,
                          PassiveFilterState* state) {
  if (!state || sample_rate == 0 || stage.cutoff_hz <= 0.0f) return;
  const float safe_cutoff = std::clamp(
      stage.cutoff_hz, 0.01f, static_cast<float>(sample_rate) * 0.49f);
  if (stage.type != AudioFilterType::PEAKING_EQ) {
    if (state->coefficient_sample_rate == sample_rate &&
        std::abs(state->coefficient_cutoff_hz - safe_cutoff) <= 0.001f) {
      return;
    }
    const float dt = 1.0f / static_cast<float>(sample_rate);
    const float time_constant = 1.0f / (2.0f * kPi * safe_cutoff);
    state->coefficient_sample_rate = sample_rate;
    state->coefficient_cutoff_hz = safe_cutoff;
    state->first_order_alpha =
        stage.type == AudioFilterType::HIGH_PASS
            ? time_constant / (time_constant + dt)
            : dt / (time_constant + dt);
    return;
  }
    // Coefficients are constant for an audio block and normally for the whole
    // simulation. Trigonometry/pow in this per-sample path caused severe CPU
    // load once several transducer EQ bands were active.
  if (state->coefficient_sample_rate == sample_rate &&
      std::abs(state->coefficient_cutoff_hz - safe_cutoff) <= 0.001f &&
      std::abs(state->coefficient_gain_db - stage.gain_db) <= 0.0001f &&
      std::abs(state->coefficient_q - stage.q) <= 0.0001f) {
    return;
  }
  const BiquadCoefficients coefficients = BuildPeakingEq(
      safe_cutoff, stage.gain_db, stage.q,
      static_cast<float>(sample_rate));
  state->coefficient_sample_rate = sample_rate;
  state->coefficient_cutoff_hz = safe_cutoff;
  state->coefficient_gain_db = stage.gain_db;
  state->coefficient_q = stage.q;
  state->b0 = coefficients.b0;
  state->b1 = coefficients.b1;
  state->b2 = coefficients.b2;
  state->a1 = coefficients.a1;
  state->a2 = coefficients.a2;
}

float ProcessPassiveFilter(float sample, const AudioFilterStage& stage,
                           unsigned int sample_rate,
                           PassiveFilterState* state) {
  if (!state || sample_rate == 0 || stage.cutoff_hz <= 0.0f) return sample;
  if (stage.type == AudioFilterType::PEAKING_EQ) {
    const float output = state->b0 * sample + state->b1 * state->x1 +
                         state->b2 * state->x2 - state->a1 * state->y1 -
                         state->a2 * state->y2;
    state->x2 = state->x1;
    state->x1 = sample;
    state->y2 = state->y1;
    state->y1 = output;
    return output;
  }
  float output = sample;
  if (stage.type == AudioFilterType::HIGH_PASS) {
    output = state->first_order_alpha *
             (state->y1 + sample - state->x1);
  } else {
    output = state->y1 +
             state->first_order_alpha * (sample - state->y1);
  }
  state->x1 = sample;
  state->y1 = output;
  // output_scalar stores the flat (non-filter) path gain. Apply the physical
  // transfer directly so changing a cutoff reshapes the spectrum instead of
  // being normalized into an apparent channel-volume boost.
  return output;
}

float ReadPitchDelay(const PitchShiftState& state, float delay_samples) {
  if (state.delay.empty()) return 0.0f;
  const float size = static_cast<float>(state.delay.size());
  float position = static_cast<float>(state.write_index) - delay_samples;
  if (position < 0.0f) position += size;
  if (position >= size) position -= size;
  const size_t first = static_cast<size_t>(position);
  const size_t second = (first + 1U) % state.delay.size();
  const float fraction = position - static_cast<float>(first);
  return state.delay[first] +
         (state.delay[second] - state.delay[first]) * fraction;
}

float ProcessPitchShift(float sample, float ratio, unsigned int sample_rate,
                        PitchShiftState* state) {
  if (!state || sample_rate == 0 || std::abs(ratio - 1.0f) < 0.001f) {
    return sample;
  }
  constexpr float kGrainMilliseconds = 42.0f;
  const size_t grain_samples = static_cast<size_t>(std::clamp(
      static_cast<float>(sample_rate) * kGrainMilliseconds / 1000.0f,
      512.0f, 4096.0f));
  const size_t delay_size = grain_samples * 2U + 4U;
  if (state->delay.size() != delay_size ||
      std::abs(state->active_ratio - ratio) > 0.001f) {
    state->delay.assign(delay_size, 0.0f);
    state->window.resize(grain_samples + 1U);
    for (size_t index = 0; index <= grain_samples; ++index) {
      const float phase = static_cast<float>(index) /
                          static_cast<float>(grain_samples);
      state->window[index] =
          0.5f - 0.5f * std::cos(2.0f * kPi * phase);
    }
    state->write_index = 0;
    state->phase = 0.0f;
    state->active_ratio = ratio;
  }
  state->delay[state->write_index] = sample;
  const float phase_a = state->phase;
  const float phase_b = std::fmod(phase_a + 0.5f, 1.0f);
  const auto delay_for_phase = [&](float phase) {
    return ratio >= 1.0f
               ? (1.0f - phase) * static_cast<float>(grain_samples)
               : phase * static_cast<float>(grain_samples);
  };
  const float window_position =
      phase_a * static_cast<float>(grain_samples);
  const size_t window_index = static_cast<size_t>(window_position);
  const size_t next_window_index =
      std::min(window_index + 1U, grain_samples);
  const float window_fraction =
      window_position - static_cast<float>(window_index);
  const float window_a =
      state->window[window_index] +
      (state->window[next_window_index] - state->window[window_index]) *
          window_fraction;
  // Hann windows half a cycle apart are complementary. Reusing the first
  // lookup removes the second transcendental operation and normalization.
  const float window_b = 1.0f - window_a;
  const float output =
      (ReadPitchDelay(*state, delay_for_phase(phase_a)) * window_a +
       ReadPitchDelay(*state, delay_for_phase(phase_b)) * window_b);
  state->write_index = (state->write_index + 1U) % state->delay.size();
  state->phase += std::abs(ratio - 1.0f) /
                  static_cast<float>(grain_samples);
  if (state->phase >= 1.0f) state->phase -= 1.0f;
  return output;
}

float ProcessDiscreteDacBus(float sample,
                            const AudioChannelStatus& channel) {
  if (channel.dac_connected_mask == 0) return 0.0f;
  // The Verilator worker has already produced 16-bit PCM. A complete,
  // correctly weighted ladder is electrically linear, so walking all 16 bits
  // again for every sample only burns callback time. Faulty/missing ladders
  // still take the detailed nonlinear conversion path below.
  if (channel.dac_bus_complete &&
      channel.dac_weight_error_percent <= 0.01f) {
    return std::clamp(sample, -1.0f, 1.0f);
  }
  const float normalized = std::clamp(sample, -1.0f, 1.0f);
  const std::uint32_t code = static_cast<std::uint32_t>(
      std::lround((normalized + 1.0f) * 32767.5f));
  constexpr std::uint32_t kZeroCode = 0x8000U;
  float converted = 0.0f;
  float zero_level = 0.0f;
  float full_scale_level = 0.0f;
  for (size_t bit = 0; bit < channel.dac_bit_weights.size(); ++bit) {
    if ((channel.dac_connected_mask & (std::uint16_t{1} << bit)) == 0) {
      continue;
    }
    const float contribution =
        static_cast<float>(std::uint32_t{1} << bit) *
        channel.dac_bit_weights[bit];
    if ((code & (std::uint32_t{1} << bit)) != 0) {
      converted += contribution;
    }
    if ((kZeroCode & (std::uint32_t{1} << bit)) != 0) {
      zero_level += contribution;
    }
    full_scale_level += contribution;
  }
  // Calibrate the malformed ladder's positive and negative full-scale spans
  // independently. Wrong bit weights must change code-step linearity, not turn
  // into an arbitrary 8x volume boost that merely slams the power amplifier.
  const float centered = converted - zero_level;
  const float negative_span = std::max(zero_level, 0.000001f);
  const float positive_span =
      std::max(full_scale_level - zero_level, 0.000001f);
  const float normalized_output =
      centered < 0.0f ? centered / negative_span
                      : centered / positive_span;
  // Missing/significantly wrong bits still create discontinuities,
  // wraparound-like jumps, and coarse quantization, but stay at line level.
  return std::clamp(normalized_output, -1.0f, 1.0f);
}

}  // namespace

const std::vector<AudioOutputDevice>& GetAudioOutputDevices(bool refresh) {
  if (!refresh && !g_output_devices.empty()) return g_output_devices;
  g_output_devices.clear();
  g_output_devices.push_back({"", "System Default Output", true});
#ifdef _WIN32
  const HRESULT com_result = CoInitializeEx(nullptr, COINIT_APARTMENTTHREADED);
  IMMDeviceEnumerator* enumerator = nullptr;
  IMMDeviceCollection* collection = nullptr;
  IMMDevice* default_device = nullptr;
  LPWSTR default_id = nullptr;
  if (SUCCEEDED(CoCreateInstance(
          __uuidof(MMDeviceEnumerator), nullptr, CLSCTX_ALL,
          __uuidof(IMMDeviceEnumerator),
          reinterpret_cast<void**>(&enumerator)))) {
    if (SUCCEEDED(enumerator->GetDefaultAudioEndpoint(
            eRender, eMultimedia, &default_device)) &&
        default_device) {
      default_device->GetId(&default_id);
    }
    if (SUCCEEDED(enumerator->EnumAudioEndpoints(
            eRender, DEVICE_STATE_ACTIVE, &collection)) &&
        collection) {
      UINT count = 0;
      collection->GetCount(&count);
      for (UINT index = 0; index < count; ++index) {
        IMMDevice* device = nullptr;
        LPWSTR device_id = nullptr;
        IPropertyStore* properties = nullptr;
        PROPVARIANT friendly_name;
        PropVariantInit(&friendly_name);
        if (SUCCEEDED(collection->Item(index, &device)) && device &&
            SUCCEEDED(device->GetId(&device_id)) && device_id) {
          std::string name = "Audio Output";
          if (SUCCEEDED(device->OpenPropertyStore(STGM_READ, &properties)) &&
              properties &&
              SUCCEEDED(properties->GetValue(kDeviceFriendlyNameKey,
                                             &friendly_name)) &&
              friendly_name.vt == VT_LPWSTR) {
            name = WideToUtf8(friendly_name.pwszVal);
          }
          g_output_devices.push_back(
              {WideToUtf8(device_id), name,
               default_id && std::wcscmp(device_id, default_id) == 0});
        }
        PropVariantClear(&friendly_name);
        if (properties) properties->Release();
        if (device_id) CoTaskMemFree(device_id);
        if (device) device->Release();
      }
    }
  }
  if (default_id) CoTaskMemFree(default_id);
  if (default_device) default_device->Release();
  if (collection) collection->Release();
  if (enumerator) enumerator->Release();
  if (SUCCEEDED(com_result)) CoUninitialize();
#else
  (void)refresh;
#endif
  return g_output_devices;
}

void UpdateAudioCircuitRuntime(std::vector<PlacedComponent>* components,
                               const std::vector<Wire>& wires,
                               bool output_enabled,
                               float minimum_volume_percent,
                               float maximum_volume_percent,
                               const std::string& output_device_id,
                               RtlRuntimeManager* rtl_runtime_manager) {
  EnsureSignalAnalysisWorker();
  const std::uint64_t circuit_signature =
      BuildCircuitSignature(components, wires);
  if (!g_has_cached_circuit_evaluation ||
      circuit_signature != g_cached_circuit_signature) {
    g_cached_circuit_evaluation = Evaluate(components, wires);
    auto compiled = std::make_shared<audio::CompiledAudioCircuit>();
    if (components && compiled->Compile(*components, wires, 48000.0)) {
      g_cached_mna_response_db[0] = compiled->SolveChannelResponse(
          false, kAudioEqFrequenciesHz);
      g_cached_mna_response_db[1] = compiled->SolveChannelResponse(
          true, kAudioEqFrequenciesHz);
      for (size_t channel = 0; channel < 2; ++channel) {
        const std::vector<float> response = compiled->SolveChannelResponse(
            channel == 1, g_mna_response_frequencies_hz);
        std::copy(response.begin(), response.end(),
                  g_cached_mna_high_resolution_db[channel].begin());
      }
      g_compiled_audio_circuit.store(std::move(compiled),
                                     std::memory_order_release);
    } else {
      g_cached_mna_response_db[0].fill(kSilenceDb);
      g_cached_mna_response_db[1].fill(kSilenceDb);
      g_cached_mna_high_resolution_db[0].fill(kSilenceDb);
      g_cached_mna_high_resolution_db[1].fill(kSilenceDb);
      g_compiled_audio_circuit.store(std::move(compiled),
                                     std::memory_order_release);
    }
    g_cached_circuit_signature = circuit_signature;
    g_has_cached_circuit_evaluation = true;
  }
  // Reset immediately on the UI-visible start edge. RunBridge repeats the
  // reset after Verilator warmup, but waiting until then leaves stale power
  // and coil temperature visible throughout a potentially long tool startup.
  if (output_enabled && !g_audio_output_was_enabled) {
    ResetAudioCircuitPipeline();
  }
  g_audio_output_was_enabled = output_enabled;
  AudioCircuitStatus next_status = g_cached_circuit_evaluation;
  const std::shared_ptr<audio::CompiledAudioCircuit> compiled =
      g_compiled_audio_circuit.load(std::memory_order_acquire);
  if (compiled) {
    const audio::CompiledAudioCircuitMetrics& metrics = compiled->Metrics();
    const audio::CompiledAudioRealtimeMetrics realtime_metrics =
        compiled->RealtimeMetrics();
    next_status.physical_solver_active = metrics.valid;
    next_status.mna_node_count = metrics.electrical_nodes;
    next_status.mna_matrix_order = metrics.matrix_order;
    next_status.mna_reduced_matrix_order = metrics.reduced_matrix_order;
    next_status.mna_eliminated_unknowns = metrics.eliminated_unknowns;
    next_status.mna_dynamic_elements = metrics.dynamic_elements;
    next_status.mna_dac_output_bits = metrics.dac_output_bits;
    next_status.mna_dc_left_volts = static_cast<float>(metrics.dc_left_volts);
    next_status.mna_dc_right_volts = static_cast<float>(metrics.dc_right_volts);
    next_status.mna_residual = static_cast<float>(metrics.last_residual);
    next_status.mna_error = metrics.error;
    next_status.mna_speaker_power_left_watts =
        static_cast<float>(realtime_metrics.left_speaker_power_watts);
    next_status.mna_speaker_power_right_watts =
        static_cast<float>(realtime_metrics.right_speaker_power_watts);
    next_status.mna_coil_temperature_left_c =
        static_cast<float>(realtime_metrics.left_coil_temperature_c);
    next_status.mna_coil_temperature_right_c =
        static_cast<float>(realtime_metrics.right_coil_temperature_c);
    next_status.mna_newton_iterations = realtime_metrics.newton_iterations;
    next_status.mna_nonlinear_substeps =
        realtime_metrics.nonlinear_substeps;
    next_status.mna_converged = realtime_metrics.converged;
    next_status.mna_failure_component_instance_id =
        realtime_metrics.failure_component_instance_id;
    next_status.mna_failure_frame = realtime_metrics.failure_frame;
    next_status.mna_residual =
        static_cast<float>(realtime_metrics.residual);
    next_status.mna_ac_response_db = g_cached_mna_response_db;
    next_status.mna_ac_response_frequencies_hz =
        g_mna_response_frequencies_hz;
    next_status.mna_ac_response_high_resolution_db =
        g_cached_mna_high_resolution_db;
    next_status.mna_dc_node_voltages.assign(
        metrics.dc_node_voltages.begin(), metrics.dc_node_voltages.end());
    next_status.mna_component_operating_points.clear();
    next_status.mna_component_operating_points.reserve(
        metrics.component_operating_points.size());
    for (const audio::CompiledComponentOperatingPoint& point :
         metrics.component_operating_points) {
      next_status.mna_component_operating_points.push_back(
          {point.component_instance_id,
           static_cast<float>(point.current_amps),
           static_cast<float>(point.power_watts),
           static_cast<float>(point.junction_temperature_c)});
    }
    if (!metrics.default_model_component_ids.empty()) {
      std::ostringstream default_model_summary;
      default_model_summary << "기본 물리 모델 사용: "
                            << metrics.default_model_component_ids.size()
                            << "개 부품 (ID ";
      constexpr std::size_t kMaximumListedComponentIds = 12;
      const std::size_t listed_count =
          std::min(metrics.default_model_component_ids.size(),
                   kMaximumListedComponentIds);
      for (std::size_t index = 0; index < listed_count; ++index) {
        if (index != 0) default_model_summary << ", ";
        default_model_summary
            << metrics.default_model_component_ids[index];
      }
      if (listed_count < metrics.default_model_component_ids.size()) {
        default_model_summary << ", ...";
      }
      default_model_summary
          << "). 프로젝트에 없는 물리 파라미터만 안전한 "
             "audio_engine_version 2 기본값으로 채웠습니다.";
      next_status.structured_diagnostics.push_back(
          {"DEFAULT_PHYSICAL_MODEL", AudioDiagnosticSeverity::INFO,
           -1, -1, default_model_summary.str()});
    }
    if (metrics.valid) {
      next_status.left.route_complete = metrics.left_signal_route;
      next_status.right.route_complete = metrics.right_signal_route;
      next_status.left.return_complete = metrics.left_return_grounded;
      next_status.right.return_complete = metrics.right_return_grounded;
      next_status.route_complete =
          metrics.left_signal_route || metrics.right_signal_route;
      next_status.return_complete =
          metrics.left_return_grounded && metrics.right_return_grounded;
      next_status.diagnosis = "MNA v2 physical circuit is active.";
    }
    if (!metrics.valid) {
      next_status.structured_diagnostics.push_back(
          {"MNA_COMPILE_FAILED", AudioDiagnosticSeverity::FATAL, -1, -1,
           metrics.error});
    } else if (!realtime_metrics.converged) {
      next_status.structured_diagnostics.push_back(
          {"MNA_NONLINEAR_CONVERGENCE_FAILED",
           AudioDiagnosticSeverity::FATAL,
           realtime_metrics.failure_component_instance_id, -1,
           "Newton-Raphson did not converge within three iterations. "
           "The engine faded from the last stable sample; inspect the "
           "reported nonlinear device and residual."});
    } else if (metrics.dac_output_bits < 32) {
      next_status.structured_diagnostics.push_back(
          {"DAC_BITS_INCOMPLETE", AudioDiagnosticSeverity::WARNING, -1, -1,
           "The physical DAC exposes fewer than 16 bits per stereo channel; "
           "DNL, offset, and distortion are simulated from the connected bits."});
    }
    if (metrics.valid &&
        (!metrics.left_signal_route || !metrics.right_signal_route)) {
      next_status.structured_diagnostics.push_back(
          {"SIGNAL_ROUTE_OPEN", AudioDiagnosticSeverity::ERROR_LEVEL, -1, -1,
           "At least one physical DAC-to-speaker signal route is open."});
      next_status.diagnosis =
          "MNA fault: a DAC-to-speaker signal route is open.";
    }
    if (metrics.valid &&
        (!metrics.left_return_grounded || !metrics.right_return_grounded)) {
      next_status.structured_diagnostics.push_back(
          {"SPEAKER_RETURN_FLOATING", AudioDiagnosticSeverity::ERROR_LEVEL, -1, -1,
           "At least one speaker return is not connected to audio ground."});
      next_status.diagnosis =
          "MNA fault: a speaker return is floating.";
    }
    if (metrics.valid &&
        (std::abs(metrics.dc_left_volts) > 0.05 ||
         std::abs(metrics.dc_right_volts) > 0.05)) {
      next_status.structured_diagnostics.push_back(
          {"SPEAKER_DC_OFFSET", AudioDiagnosticSeverity::FATAL, -1, -1,
           "The DC operating point puts more than 50 mV across a speaker "
           "voice coil."});
      next_status.diagnosis =
          "MNA fault: dangerous speaker DC offset is present.";
    }
    if (next_status.left.dac_weight_error_percent > 0.5f ||
        next_status.right.dac_weight_error_percent > 0.5f) {
      next_status.structured_diagnostics.push_back(
          {"DAC_DNL_INL", AudioDiagnosticSeverity::WARNING, -1, -1,
           "R-2R weight mismatch produces DNL/INL and harmonic distortion."});
      next_status.diagnosis =
          "MNA fault: DAC resistor weighting is non-monotonic or inaccurate.";
    }
    float maximum_stereo_delta = 0.0f;
    for (size_t band = 0; band < kAudioEqFrequenciesHz.size(); ++band) {
      maximum_stereo_delta = std::max(
          maximum_stereo_delta,
          std::abs(g_cached_mna_response_db[0][band] -
                   g_cached_mna_response_db[1][band]));
    }
    if (metrics.valid && maximum_stereo_delta > 3.0f) {
      next_status.structured_diagnostics.push_back(
          {"STEREO_RESPONSE_MISMATCH", AudioDiagnosticSeverity::WARNING, -1,
           -1,
           "Left/right electrical response differs by more than 3 dB."});
      next_status.diagnosis =
          "MNA fault: left/right circuit response is severely mismatched.";
    }
    if (realtime_metrics.left_speaker_power_watts >
            next_status.left.speaker_rated_power_watts ||
        realtime_metrics.right_speaker_power_watts >
            next_status.right.speaker_rated_power_watts) {
      next_status.structured_diagnostics.push_back(
          {"SPEAKER_POWER_EXCEEDED", AudioDiagnosticSeverity::ERROR_LEVEL,
           -1, -1,
           "Calculated voice-coil power exceeds the speaker rating."});
      next_status.diagnosis =
          "MNA fault: speaker electrical power rating is exceeded.";
    }
    if (realtime_metrics.left_coil_temperature_c > 100.0 ||
        realtime_metrics.right_coil_temperature_c > 100.0) {
      next_status.structured_diagnostics.push_back(
          {"SPEAKER_THERMAL_LIMIT", AudioDiagnosticSeverity::FATAL, -1, -1,
           "Voice-coil temperature exceeds the safe thermal limit."});
      next_status.diagnosis =
          "MNA fault: speaker voice coil is overheating.";
    }
  }
  next_status.mna_block_time_ms =
      g_mna_block_time_ms.load(std::memory_order_relaxed);
  next_status.mna_deadline_percent =
      g_mna_deadline_percent.load(std::memory_order_relaxed);
  const float maximum_scalar =
      std::isfinite(maximum_volume_percent)
          ? std::clamp(maximum_volume_percent, 0.0f, 100.0f) / 100.0f
          : 0.0f;
  if (next_status.physical_solver_active) {
    // The MNA solution already contains the circuit's physical attenuation
    // and gain. The application range is only the final hearing-safety cap.
    next_status.applied_left_scalar = maximum_scalar;
    next_status.applied_right_scalar = maximum_scalar;
  } else {
    next_status.applied_left_scalar =
        std::clamp(MapOutputRange(next_status.left.output_scalar,
                                  minimum_volume_percent,
                                  maximum_volume_percent),
                   0.0f, maximum_scalar);
    next_status.applied_right_scalar =
        std::clamp(MapOutputRange(next_status.right.output_scalar,
                                  minimum_volume_percent,
                                  maximum_volume_percent),
                   0.0f, maximum_scalar);
  }
  if (components) {
    for (PlacedComponent& component : *components) {
      if (component.type == ComponentType::AUDIO_SPEAKER) {
        component.internalStates[state_keys::kAudioLevel] =
            std::max(next_status.applied_left_scalar,
                     next_status.applied_right_scalar);
        component.internalStates[state_keys::kStatus] =
            next_status.route_complete && next_status.return_complete
                ? 1.0f
                : 0.0f;
      }
    }
  }
  {
    std::lock_guard<std::mutex> lock(g_status_mutex);
    g_has_audio_dac_component = false;
    if (components) {
      const auto dac_it = std::find_if(
          components->begin(), components->end(),
          [](const PlacedComponent& component) {
            return HasFunctionalStereoDacInterface(component);
          });
      if (dac_it != components->end()) {
        g_audio_dac_component = *dac_it;
        g_has_audio_dac_component = true;
      }
    }
    g_audio_rtl_runtime_manager = rtl_runtime_manager;
    if (g_audio_rtl_runtime_manager && g_has_audio_dac_component) {
      g_audio_rtl_runtime_manager->ConfigureAudioDac(g_audio_dac_component);
      const std::string rtl_diagnostics =
          g_audio_rtl_runtime_manager->GetAudioDacDiagnostics();
      g_audio_worker_error = rtl_diagnostics;
    }
    if (g_realtime_worker_unavailable.exchange(
            false, std::memory_order_acq_rel) &&
        g_audio_worker_error.empty()) {
      g_audio_worker_error = "functional I2S DAC worker is unavailable";
    }
    if (!g_audio_worker_error.empty()) {
      if (!next_status.diagnosis.empty()) next_status.diagnosis += " ";
      next_status.diagnosis += "Verilator DAC: " + g_audio_worker_error;
    }
    g_status = std::move(next_status);
    auto realtime = std::make_shared<RealtimeAudioSnapshot>();
    realtime->status = g_status;
    realtime->dac_component = g_audio_dac_component;
    realtime->has_dac_component = g_has_audio_dac_component;
    realtime->rtl_runtime_manager = g_audio_rtl_runtime_manager;
    g_realtime_audio_snapshot.store(std::move(realtime),
                                    std::memory_order_release);
  }
  // Audio is rendered only by AudioStreamBridge after PCM circuit processing.
  // Never change the Windows endpoint master/channel volume here.
  (void)output_enabled;
  (void)output_device_id;
}

AudioBlockProcessResult ProcessAudioCircuitBlockSequenced(
    float* interleaved_stereo, size_t frame_count, unsigned int sample_rate,
    std::uint64_t sequence) {
  AudioBlockProcessResult block_result;
  if (!interleaved_stereo || sample_rate == 0) return block_result;
  block_result.input_rms = BlockRms(interleaved_stereo, frame_count);
  const std::shared_ptr<const RealtimeAudioSnapshot> realtime =
      g_realtime_audio_snapshot.load(std::memory_order_acquire);
  if (!realtime) {
    FadeRealtimeOutput(interleaved_stereo, frame_count);
    block_result.concealed = true;
    block_result.silence_reason = AudioSilenceReason::PHYSICAL_ZERO_OUTPUT;
    block_result.output_rms = BlockRms(interleaved_stereo, frame_count);
    return block_result;
  }
  const AudioCircuitStatus& status = realtime->status;
  const bool has_dac_component = realtime->has_dac_component;
  RtlRuntimeManager* const rtl_runtime_manager =
      realtime->rtl_runtime_manager;
  if (((status.physical_solver_active &&
        status.mna_dac_output_bits > 0) ||
       status.left.route_complete || status.right.route_complete) &&
      (!has_dac_component || !rtl_runtime_manager)) {
    g_realtime_worker_unavailable.store(true, std::memory_order_release);
    FadeRealtimeOutput(interleaved_stereo, frame_count);
    UpdateSignalAnalysis(interleaved_stereo, frame_count, sample_rate);
    block_result.concealed = true;
    block_result.silence_reason = AudioSilenceReason::RTL_WAIT;
    block_result.output_rms = BlockRms(interleaved_stereo, frame_count);
    return block_result;
  }
  if (has_dac_component && rtl_runtime_manager) {
    constexpr size_t kMaximumAsyncRtlFrames = 1024;
    if (frame_count > kMaximumAsyncRtlFrames) {
      g_realtime_worker_unavailable.store(true, std::memory_order_release);
      FadeRealtimeOutput(interleaved_stereo, frame_count);
      UpdateSignalAnalysis(interleaved_stereo, frame_count, sample_rate);
      block_result.concealed = true;
      block_result.silence_reason = AudioSilenceReason::RTL_WAIT;
      block_result.output_rms = BlockRms(interleaved_stereo, frame_count);
      return block_result;
    }
    std::array<float, kMaximumAsyncRtlFrames * 2> submitted_pcm{};
    ConvertAsynchronousSampleClock(
        interleaved_stereo, submitted_pcm.data(), frame_count,
        status.pitch_shift_ratio);
    const auto rtl_started = std::chrono::steady_clock::now();
    size_t received_frames = 0;
    bool received = false;
    if (g_rtl_pending_count > 0) {
      const std::uint64_t expected =
          g_rtl_pending_sequences[g_rtl_pending_head];
      received = rtl_runtime_manager->TryReceiveDacCodes(
          expected, interleaved_stereo, frame_count, &received_frames);
      if (received) {
        g_rtl_pending_head =
            (g_rtl_pending_head + 1U) % g_rtl_pending_sequences.size();
        --g_rtl_pending_count;
      }
    }
    const bool submitted = rtl_runtime_manager->SubmitDacFrames(
        sequence, submitted_pcm.data(), frame_count);
    block_result.rtl_sequence_lost = !submitted;
    if (submitted && g_rtl_pending_count < g_rtl_pending_sequences.size()) {
      const size_t tail = (g_rtl_pending_head + g_rtl_pending_count) %
                          g_rtl_pending_sequences.size();
      g_rtl_pending_sequences[tail] = sequence;
      ++g_rtl_pending_count;
    }
    block_result.rtl_time_ms = std::chrono::duration<float, std::milli>(
        std::chrono::steady_clock::now() - rtl_started).count();
    if (!rtl_runtime_manager->AudioDacWorkerHealthy()) {
      g_realtime_worker_unavailable.store(true, std::memory_order_release);
      FadeRealtimeOutput(interleaved_stereo, frame_count);
      UpdateSignalAnalysis(interleaved_stereo, frame_count, sample_rate);
      block_result.concealed = true;
      block_result.silence_reason = AudioSilenceReason::RTL_WAIT;
      block_result.output_rms = BlockRms(interleaved_stereo, frame_count);
      return block_result;
    }
    if (received && received_frames == frame_count) {
      block_result.rtl_valid = true;
      g_last_dac_code_left = interleaved_stereo[(frame_count - 1U) * 2U];
      g_last_dac_code_right =
          interleaved_stereo[(frame_count - 1U) * 2U + 1U];
      g_has_last_dac_code = true;
    } else {
      // A hardware DAC holds its last converted code while a serial frame is
      // late. Preserve that behavior instead of turning an ordinary one-block
      // RTL scheduling miss into 1024 frames of hard zero.
      const float held_left = g_has_last_dac_code ? g_last_dac_code_left : 0.0f;
      const float held_right =
          g_has_last_dac_code ? g_last_dac_code_right : 0.0f;
      for (size_t frame = 0; frame < frame_count; ++frame) {
        interleaved_stereo[frame * 2U] = held_left;
        interleaved_stereo[frame * 2U + 1U] = held_right;
      }
      block_result.concealed = true;
      block_result.silence_reason = AudioSilenceReason::RTL_WAIT;
      if (!submitted) {
        g_realtime_worker_unavailable.store(true, std::memory_order_release);
      }
    }
  }
  const std::shared_ptr<audio::CompiledAudioCircuit> compiled =
      g_compiled_audio_circuit.load(std::memory_order_acquire);
  if (compiled && compiled->IsValid()) {
    const auto started = std::chrono::steady_clock::now();
    const bool mna_ok = compiled->ProcessBlock(interleaved_stereo, frame_count);
    const auto elapsed = std::chrono::steady_clock::now() - started;
    const float block_ms = std::chrono::duration<float, std::milli>(elapsed).count();
    const float deadline_ms =
        static_cast<float>(frame_count) * 1000.0f /
        static_cast<float>(sample_rate);
    g_mna_block_time_ms.store(block_ms, std::memory_order_relaxed);
    g_mna_deadline_percent.store(
        deadline_ms > 0.0f ? block_ms / deadline_ms * 100.0f : 0.0f,
        std::memory_order_relaxed);
    block_result.mna_time_ms = block_ms;
    if (!mna_ok) {
      // ProcessBlock has already replaced the unstable tail with a short
      // fade from the last converged sample. Preserve it to avoid a click.
      RememberRealtimeOutput(interleaved_stereo, frame_count);
      UpdateSignalAnalysis(interleaved_stereo, frame_count, sample_rate);
      block_result.silence_reason = AudioSilenceReason::MNA_FAILURE;
      block_result.output_rms = BlockRms(interleaved_stereo, frame_count);
      return block_result;
    }

    const AudioChannelStatus* physical_channels[2] = {&status.left,
                                                       &status.right};
    const float physical_applied[2] = {status.applied_left_scalar,
                                        status.applied_right_scalar};
    for (size_t channel = 0; channel < 2; ++channel) {
      const std::vector<AudioFilterStage>& all_stages =
          physical_channels[channel]->filter_stages;
      size_t acoustic_count = 0;
      for (const AudioFilterStage& stage : all_stages) {
        if (stage.type == AudioFilterType::PEAKING_EQ &&
            acoustic_count < kMaximumAcousticFilterStages) {
          g_mna_acoustic_filter_stages[channel][acoustic_count++] = stage;
        }
      }
      if (g_mna_acoustic_filter_count[channel] != acoustic_count) {
        g_mna_acoustic_filter_count[channel] = acoustic_count;
        g_mna_acoustic_filter_states[channel] = {};
      }
      for (size_t stage_index = 0; stage_index < acoustic_count;
           ++stage_index) {
        PreparePassiveFilter(g_mna_acoustic_filter_stages[channel][stage_index],
                             sample_rate,
                             &g_mna_acoustic_filter_states[channel][stage_index]);
      }
    }
    for (size_t frame = 0; frame < frame_count; ++frame) {
      for (size_t channel = 0; channel < 2; ++channel) {
        float sample = interleaved_stereo[frame * 2 + channel];
        if (!std::isfinite(sample)) sample = 0.0f;
        for (size_t stage_index = 0;
             stage_index < g_mna_acoustic_filter_count[channel];
             ++stage_index) {
          sample = ProcessPassiveFilter(
              sample, g_mna_acoustic_filter_stages[channel][stage_index],
              sample_rate,
              &g_mna_acoustic_filter_states[channel][stage_index]);
        }
        if (!std::isfinite(sample)) sample = 0.0f;
        interleaved_stereo[frame * 2 + channel] = std::clamp(
            sample * physical_applied[channel], -1.0f, 1.0f);
      }
    }
    RememberRealtimeOutput(interleaved_stereo, frame_count);
    UpdateSignalAnalysis(interleaved_stereo, frame_count, sample_rate);
    block_result.mna_valid = true;
    block_result.output_rms = BlockRms(interleaved_stereo, frame_count);
    if (block_result.output_rms < 1.0e-6f &&
        block_result.input_rms > 1.0e-4f &&
        block_result.silence_reason == AudioSilenceReason::NONE) {
      block_result.silence_reason = AudioSilenceReason::PHYSICAL_ZERO_OUTPUT;
    }
    return block_result;
  }
  if (compiled && !compiled->IsValid()) {
    FadeRealtimeOutput(interleaved_stereo, frame_count);
    UpdateSignalAnalysis(interleaved_stereo, frame_count, sample_rate);
    block_result.silence_reason = AudioSilenceReason::MNA_FAILURE;
    block_result.output_rms = BlockRms(interleaved_stereo, frame_count);
    return block_result;
  }
  const AudioChannelStatus* channels[2] = {&status.left, &status.right};
  const float applied[2] = {status.applied_left_scalar,
                            status.applied_right_scalar};
  std::array<float, 2> dac_fault_level_gain = {1.0f, 1.0f};
  for (size_t channel = 0; channel < 2; ++channel) {
    if (channels[channel]->dac_weight_error_percent <= 0.5f ||
        frame_count == 0) {
      continue;
    }
    double input_sum = 0.0;
    double converted_sum = 0.0;
    for (size_t frame = 0; frame < frame_count; ++frame) {
      const float input = interleaved_stereo[frame * 2 + channel];
      input_sum += input;
      converted_sum += ProcessDiscreteDacBus(input, *channels[channel]);
    }
    const double input_mean = input_sum / static_cast<double>(frame_count);
    const double converted_mean =
        converted_sum / static_cast<double>(frame_count);
    double input_energy = 0.0;
    double converted_energy = 0.0;
    for (size_t frame = 0; frame < frame_count; ++frame) {
      const float input = interleaved_stereo[frame * 2 + channel];
      const float converted =
          ProcessDiscreteDacBus(input, *channels[channel]);
      const double centered_input = input - input_mean;
      const double centered_converted = converted - converted_mean;
      input_energy += centered_input * centered_input;
      converted_energy += centered_converted * centered_converted;
    }
    if (input_energy > 0.000000000001 &&
        converted_energy > input_energy) {
      dac_fault_level_gain[channel] = static_cast<float>(std::clamp(
          std::sqrt(input_energy / converted_energy), 0.01, 1.0));
    }
  }
  // Filter topology is block-constant. Comparing and copying the vectors in
  // the inner sample loop made callback cost grow with frames x EQ bands.
  for (size_t channel = 0; channel < 2; ++channel) {
    const std::vector<AudioFilterStage>& stages =
        channels[channel]->filter_stages;
    if (!SameFilterStages(g_active_filter_stages[channel], stages)) {
      g_active_filter_stages[channel] = stages;
      g_passive_filter_states[channel].assign(stages.size(), {});
    }
    for (size_t stage_index = 0; stage_index < stages.size(); ++stage_index) {
      PreparePassiveFilter(stages[stage_index], sample_rate,
                           &g_passive_filter_states[channel][stage_index]);
    }
  }
  for (size_t frame = 0; frame < frame_count; ++frame) {
    for (size_t channel = 0; channel < 2; ++channel) {
      float sample = interleaved_stereo[frame * 2 + channel];
      if (!channels[channel]->route_complete ||
          !channels[channel]->return_complete || applied[channel] <= 0.0f) {
        interleaved_stereo[frame * 2 + channel] = 0.0f;
        continue;
      }
      sample = ProcessDiscreteDacBus(sample, *channels[channel]) *
               dac_fault_level_gain[channel];
      sample = ProcessPitchShift(sample, status.pitch_shift_ratio, sample_rate,
                                 &g_pitch_shift_states[channel]);
      // Treat one normalized DAC full-scale unit as 1 V peak at the
      // amplifier input. Gain changes the requested speaker voltage; the
      // smaller of rail headroom and I_limit * speaker impedance clips it.
      // Dividing back by the requested path gain preserves the existing
      // user safety-volume mapping while making weak supplies quieter and
      // overloaded amplifiers audibly distort both level and spectrum.
      const float requested_path_gain =
          std::max(std::abs(channels[channel]->output_scalar), 0.000001f);
      const float requested_speaker_voltage = sample * requested_path_gain;
      const float available_peak_voltage =
          std::max(0.0f, channels[channel]->maximum_speaker_peak_voltage);
      sample = std::clamp(requested_speaker_voltage,
                          -available_peak_voltage,
                          available_peak_voltage) /
               requested_path_gain;
      const std::vector<AudioFilterStage>& stages =
          channels[channel]->filter_stages;
      for (size_t stage_index = 0; stage_index < stages.size(); ++stage_index) {
        sample = ProcessPassiveFilter(
            sample, stages[stage_index], sample_rate,
            &g_passive_filter_states[channel][stage_index]);
      }
      if (channels[channel]->amplifier_bias_fault) {
        const float bias_error =
            channels[channel]->amplifier_bias_error_volts;
        if (bias_error < 0.0f) {
          // Under-bias creates a zero-crossing dead zone rather than broadband
          // microphone-style saturation.
          const float dead_zone = std::clamp(
              -bias_error / 1.3f * 0.25f, 0.0f, 0.35f);
          const float magnitude = std::abs(sample);
          sample = magnitude <= dead_zone
                       ? 0.0f
                       : std::copysign(
                             (magnitude - dead_zone) /
                                 std::max(1.0f - dead_zone, 0.000001f),
                             sample);
        } else {
          // Over-bias produces a gentler cubic compression and heating model.
          const float over_bias =
              std::clamp(bias_error / 1.3f, 0.0f, 1.0f) * 0.20f;
          sample -= over_bias * sample * sample * sample;
        }
      }
      g_noise_state = g_noise_state * 1664525U + 1013904223U;
      const float uniform_noise =
          static_cast<float>((g_noise_state >> 8U) & 0x00ffffffU) /
              static_cast<float>(0x00ffffffU) *
              2.0f -
          1.0f;
      const float noise_floor_clamped =
          std::clamp(channels[channel]->noise_floor_db, -180.0f, 0.0f);
      const float noise_amplitude =
          std::pow(10.0f, noise_floor_clamped / 20.0f);
      sample += uniform_noise * noise_amplitude;
      const float generic_distortion_percent = std::max(
          0.0f, channels[channel]->thd_percent -
                    channels[channel]->dac_weight_error_percent -
                    channels[channel]->specialized_distortion_percent);
      const float distortion_mix =
          std::clamp(generic_distortion_percent, 0.0f, 100.0f) / 100.0f;
      if (distortion_mix > 0.0f) {
        constexpr float kSaturationDrive = 3.0f;
        const float saturated =
            std::tanh(sample * kSaturationDrive) / kSaturationDrive;
        sample += (saturated - sample) * distortion_mix;
      }
      if (!std::isfinite(sample)) sample = 0.0f;
      interleaved_stereo[frame * 2 + channel] =
          std::clamp(sample * applied[channel], -1.0f, 1.0f);
    }
  }
  RememberRealtimeOutput(interleaved_stereo, frame_count);
  UpdateSignalAnalysis(interleaved_stereo, frame_count, sample_rate);
  block_result.mna_valid = true;
  block_result.output_rms = BlockRms(interleaved_stereo, frame_count);
  return block_result;
}

void ProcessAudioCircuitBlock(float* interleaved_stereo, size_t frame_count,
                              unsigned int sample_rate) {
  static std::atomic<std::uint64_t> compatibility_sequence{0};
  (void)ProcessAudioCircuitBlockSequenced(
      interleaved_stereo, frame_count, sample_rate,
      compatibility_sequence.fetch_add(1, std::memory_order_relaxed));
}

bool WarmupAudioCircuitRuntime(std::string* diagnostics) {
  const std::shared_ptr<const RealtimeAudioSnapshot> realtime =
      g_realtime_audio_snapshot.load(std::memory_order_acquire);
  if (!realtime || !realtime->rtl_runtime_manager ||
      !realtime->has_dac_component) {
    if (diagnostics) *diagnostics = "Functional audio DAC is unavailable.";
    return false;
  }
  const bool ready =
      realtime->rtl_runtime_manager->WarmupAudioDac(diagnostics);
  ResetAudioCircuitPipeline();
  return ready;
}

void ResetAudioCircuitPipeline() {
  g_rtl_pending_head = 0;
  g_rtl_pending_count = 0;
  g_last_dac_code_left = 0.0f;
  g_last_dac_code_right = 0.0f;
  g_has_last_dac_code = false;
  g_last_realtime_left = 0.0f;
  g_last_realtime_right = 0.0f;
  g_passive_filter_states = {};
  g_active_filter_stages = {};
  g_mna_acoustic_filter_states = {};
  g_mna_acoustic_filter_count = {};
  g_pitch_shift_states = {};
  g_async_sample_clock.oldest_sequence = 0;
  g_async_sample_clock.write_sequence = 0;
  g_async_sample_clock.read_position = 0.0;
  g_async_sample_clock.ratio = 1.0f;
  g_async_sample_clock.held_left = 0.0f;
  g_async_sample_clock.held_right = 0.0f;
  g_async_sample_clock.initialized = false;
  g_noise_state = 0x8f3a91c5U;
  const std::shared_ptr<audio::CompiledAudioCircuit> compiled =
      g_compiled_audio_circuit.load(std::memory_order_acquire);
  if (compiled) (void)compiled->ResetRealtimeState();
  const std::shared_ptr<const RealtimeAudioSnapshot> realtime =
      g_realtime_audio_snapshot.load(std::memory_order_acquire);
  if (realtime && realtime->rtl_runtime_manager) {
    realtime->rtl_runtime_manager->ResetAudioPipeline();
  }
}

AudioCircuitStatus GetAudioCircuitStatus() {
  std::lock_guard<std::mutex> lock(g_status_mutex);
  return g_status;
}

AudioSignalAnalysis GetAudioSignalAnalysis() {
  std::lock_guard<std::mutex> lock(g_signal_analysis_mutex);
  return g_signal_analysis;
}

void ResetAudioSignalAnalysis() {
  g_analysis_queue.Clear();
  std::lock_guard<std::mutex> lock(g_signal_analysis_mutex);
  g_signal_analysis = {};
  g_analysis_ring_write = 0;
  g_analysis_ring_size = 0;
  g_analysis_update_counter = 0;
  g_analysis_has_snapshot = false;
}

void ShutdownAudioCircuitRuntime() {
  g_analysis_worker_stop.store(true, std::memory_order_release);
  g_analysis_worker_wake.notify_all();
  if (g_analysis_worker.joinable()) g_analysis_worker.join();
  g_analysis_worker_started.store(false, std::memory_order_release);
  RestoreEndpointVolume();
#ifdef _WIN32
  if (g_endpoint) {
    g_endpoint->Release();
    g_endpoint = nullptr;
  }
  if (g_com_initialized) {
    CoUninitialize();
    g_com_initialized = false;
  }
#endif
  g_passive_filter_states = {};
  g_active_filter_stages = {};
  g_pitch_shift_states = {};
  ResetAudioSignalAnalysis();
  std::lock_guard<std::mutex> lock(g_status_mutex);
  g_has_audio_dac_component = false;
  g_audio_rtl_runtime_manager = nullptr;
  g_audio_worker_error.clear();
  g_realtime_worker_unavailable.store(false, std::memory_order_release);
  g_last_realtime_left = 0.0f;
  g_last_realtime_right = 0.0f;
  g_async_sample_clock.oldest_sequence = 0;
  g_async_sample_clock.write_sequence = 0;
  g_async_sample_clock.read_position = 0.0;
  g_async_sample_clock.initialized = false;
  g_audio_output_was_enabled = false;
}

}  // namespace plc
