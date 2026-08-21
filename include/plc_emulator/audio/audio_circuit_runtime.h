#ifndef PLC_EMULATOR_AUDIO_AUDIO_CIRCUIT_RUNTIME_H_
#define PLC_EMULATOR_AUDIO_AUDIO_CIRCUIT_RUNTIME_H_

#include "plc_emulator/core/data_types.h"

#include <string>
#include <array>
#include <cstddef>
#include <cstdint>
#include <vector>

namespace plc {

class RtlRuntimeManager;

inline constexpr std::array<float, 10> kAudioEqFrequenciesHz = {
    31.25f, 62.5f, 125.0f, 250.0f, 500.0f,
    1000.0f, 2000.0f, 4000.0f, 8000.0f, 16000.0f};
inline constexpr size_t kAudioAnalyzerWaveformPoints = 256;
inline constexpr size_t kAudioAnalyzerSpectrumBins = 48;
inline constexpr size_t kAudioCircuitResponsePoints = 256;

enum class AudioFilterType {
  HIGH_PASS,
  LOW_PASS,
  PEAKING_EQ,
};

enum class AudioDiagnosticSeverity { INFO, WARNING, ERROR_LEVEL, FATAL };

struct AudioCircuitDiagnostic {
  std::string code;
  AudioDiagnosticSeverity severity = AudioDiagnosticSeverity::INFO;
  int component_instance_id = -1;
  int port_id = -1;
  std::string message;
};

struct AudioComponentOperatingPoint {
  int component_instance_id = -1;
  float current_amps = 0.0f;
  float power_watts = 0.0f;
  float junction_temperature_c = 25.0f;
};

struct AudioFilterStage {
  AudioFilterType type = AudioFilterType::HIGH_PASS;
  float cutoff_hz = 0.0f;
  float reference_gain = 1.0f;
  int component_instance_id = -1;
  float gain_db = 0.0f;
  float q = 0.70710678f;
};

struct AudioChannelStatus {
  bool route_complete = false;
  bool return_complete = false;
  bool amplifier_present = false;
  bool amplifier_powered = false;
  bool dc_blocking_capacitor_present = false;
  bool emitter_ballast_present = false;
  bool speaker_damage_risk = false;
  bool amplifier_bias_fault = false;
  bool amplifier_thermal_tracking_fault = false;
  bool amplifier_bandwidth_limited = false;
  bool amplifier_power_insufficient = false;
  float amplifier_bias_error_volts = 0.0f;
  float specialized_distortion_percent = 0.0f;
  float estimated_dc_offset_volts = 0.0f;
  float output_scalar = 0.0f;
  float output_db = -120.0f;
  float amplifier_supply_voltage = 0.0f;
  float amplifier_current_limit_amps = 0.0f;
  float maximum_speaker_peak_voltage = 0.0f;
  float maximum_speaker_power_watts = 0.0f;
  float speaker_rated_power_watts = 100.0f;
  float noise_floor_db = -120.0f;
  float thd_percent = 0.0f;
  float dac_weight_error_percent = 0.0f;
  std::uint16_t dac_connected_mask = 0;
  bool dac_bus_complete = false;
  std::array<float, 16> dac_bit_weights{};
  std::array<float, 10> eq_db{};
  std::vector<AudioFilterStage> filter_stages;
};

struct AudioCircuitStatus {
  bool route_complete = false;
  bool return_complete = false;
  float output_scalar = 0.0f;
  float output_db = -96.0f;
  float noise_floor_db = -96.0f;
  float thd_percent = 0.0f;
  float pitch_shift_ratio = 1.0f;
  std::array<float, 10> eq_frequencies_hz = kAudioEqFrequenciesHz;
  AudioChannelStatus left;
  AudioChannelStatus right;
  float applied_left_scalar = 0.0f;
  float applied_right_scalar = 0.0f;
  bool physical_solver_active = false;
  int mna_node_count = 0;
  int mna_matrix_order = 0;
  int mna_reduced_matrix_order = 0;
  int mna_eliminated_unknowns = 0;
  int mna_dynamic_elements = 0;
  int mna_dac_output_bits = 0;
  float mna_dc_left_volts = 0.0f;
  float mna_dc_right_volts = 0.0f;
  float mna_residual = 0.0f;
  float mna_block_time_ms = 0.0f;
  float mna_deadline_percent = 0.0f;
  float mna_speaker_power_left_watts = 0.0f;
  float mna_speaker_power_right_watts = 0.0f;
  float mna_coil_temperature_left_c = 25.0f;
  float mna_coil_temperature_right_c = 25.0f;
  int mna_newton_iterations = 0;
  int mna_nonlinear_substeps = 1;
  bool mna_converged = true;
  int mna_failure_component_instance_id = -1;
  int mna_failure_frame = -1;
  std::string mna_error;
  std::array<std::array<float, 10>, 2> mna_ac_response_db{};
  std::array<float, kAudioCircuitResponsePoints>
      mna_ac_response_frequencies_hz{};
  std::array<std::array<float, kAudioCircuitResponsePoints>, 2>
      mna_ac_response_high_resolution_db{};
  std::vector<float> mna_dc_node_voltages;
  std::vector<AudioComponentOperatingPoint> mna_component_operating_points;
  std::vector<AudioCircuitDiagnostic> structured_diagnostics;
  std::string diagnosis;
};

struct AudioOutputDevice {
  std::string id;
  std::string name;
  bool is_default = false;
};

struct AudioSignalAnalysis {
  bool active = false;
  unsigned int sample_rate = 0;
  size_t analyzed_frames = 0;
  std::uint64_t sequence = 0;
  std::array<float, 2> rms_dbfs = {-120.0f, -120.0f};
  std::array<float, 2> peak_dbfs = {-120.0f, -120.0f};
  std::array<float, 2> clipped_percent{};
  std::array<float, 2> fundamental_hz{};
  std::array<float, 2> thd_percent{};
  std::array<float, 2> thd_plus_noise_percent{};
  std::array<float, 2> snr_db{};
  std::array<std::array<float, kAudioAnalyzerWaveformPoints>, 2> waveform{};
  std::array<float, kAudioAnalyzerSpectrumBins> spectrum_frequencies_hz{};
  std::array<std::array<float, kAudioAnalyzerSpectrumBins>, 2> spectrum_dbfs{};
  std::array<std::array<float, 10>, 2> model_response_db{};
};

enum class AudioSilenceReason {
  NONE = 0,
  RTL_WAIT,
  MNA_FAILURE,
  RENDER_STARVATION,
  PHYSICAL_ZERO_OUTPUT,
};

struct AudioBlockProcessResult {
  bool rtl_valid = false;
  bool mna_valid = false;
  bool concealed = false;
  bool rtl_sequence_lost = false;
  AudioSilenceReason silence_reason = AudioSilenceReason::NONE;
  float rtl_time_ms = 0.0f;
  float mna_time_ms = 0.0f;
  float input_rms = 0.0f;
  float output_rms = 0.0f;
};

void UpdateAudioCircuitRuntime(std::vector<PlacedComponent>* components,
                               const std::vector<Wire>& wires,
                               bool output_enabled,
                               float minimum_volume_percent,
                               float maximum_volume_percent,
                               const std::string& output_device_id,
                               RtlRuntimeManager* rtl_runtime_manager);
const std::vector<AudioOutputDevice>& GetAudioOutputDevices(bool refresh);
void ProcessAudioCircuitBlock(float* interleaved_stereo, size_t frame_count,
                              unsigned int sample_rate);
AudioBlockProcessResult ProcessAudioCircuitBlockSequenced(
    float* interleaved_stereo, size_t frame_count, unsigned int sample_rate,
    std::uint64_t sequence);
bool WarmupAudioCircuitRuntime(std::string* diagnostics);
void ResetAudioCircuitPipeline();
AudioCircuitStatus GetAudioCircuitStatus();
AudioSignalAnalysis GetAudioSignalAnalysis();
void ResetAudioSignalAnalysis();
void ShutdownAudioCircuitRuntime();

}  // namespace plc

#endif
