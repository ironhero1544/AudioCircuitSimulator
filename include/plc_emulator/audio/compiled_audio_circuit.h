#ifndef PLC_EMULATOR_AUDIO_COMPILED_AUDIO_CIRCUIT_H_
#define PLC_EMULATOR_AUDIO_COMPILED_AUDIO_CIRCUIT_H_

#include "plc_emulator/audio/mna_solver.h"
#include "plc_emulator/core/data_types.h"

#include <array>
#include <atomic>
#include <cstddef>
#include <cstdint>
#include <map>
#include <span>
#include <string>
#include <vector>

namespace plc::audio {

struct CompiledComponentOperatingPoint {
  int component_instance_id = -1;
  double current_amps = 0.0;
  double power_watts = 0.0;
  double junction_temperature_c = 25.0;
};

struct CompiledAudioCircuitMetrics {
  bool valid = false;
  int electrical_nodes = 0;
  int matrix_order = 0;
  int reduced_matrix_order = 0;
  int eliminated_unknowns = 0;
  int dynamic_elements = 0;
  int dac_output_bits = 0;
  bool left_signal_route = false;
  bool right_signal_route = false;
  bool left_return_grounded = false;
  bool right_return_grounded = false;
  double dc_left_volts = 0.0;
  double dc_right_volts = 0.0;
  double last_residual = 0.0;
  std::vector<double> dc_node_voltages;
  std::vector<CompiledComponentOperatingPoint> component_operating_points;
  std::vector<int> default_model_component_ids;
  std::string error;
};

struct CompiledAudioRealtimeMetrics {
  bool converged = true;
  int failure_component_instance_id = -1;
  int failure_frame = -1;
  double left_speaker_power_watts = 0.0;
  double right_speaker_power_watts = 0.0;
  double left_coil_temperature_c = 25.0;
  double right_coil_temperature_c = 25.0;
  int newton_iterations = 0;
  int nonlinear_substeps = 1;
  double residual = 0.0;
};

class CompiledAudioCircuit {
 public:
  bool Compile(const std::vector<PlacedComponent>& components,
               const std::vector<Wire>& wires, double sample_rate);

  bool ProcessBlock(float* interleaved_stereo, size_t frame_count);

  std::array<float, 10> SolveChannelResponse(
      bool right_channel,
      const std::array<float, 10>& frequencies_hz) const;
  std::vector<float> SolveChannelResponse(
      bool right_channel, std::span<const float> frequencies_hz) const;

  const CompiledAudioCircuitMetrics& Metrics() const { return metrics_; }
  CompiledAudioRealtimeMetrics RealtimeMetrics() const;
  bool IsValid() const { return metrics_.valid; }
  double PortVoltage(int component_instance_id, int port_id) const;
  bool ResetRealtimeState();

 private:
  struct DacBitSource {
    int slot = -1;
    int channel = 0;
    int bit = 0;
    int output_node = 0;
    double high_current_amps = 0.0;
  };

  struct TimeSource {
    enum class Type { SINE, PULSE };
    int slot = -1;
    Type type = Type::SINE;
    double amplitude = 0.0;
    double frequency_hz = 0.0;
    double phase_radians = 0.0;
    double low_volts = 0.0;
    double high_volts = 0.0;
    double duty = 0.5;
  };
  struct FloatingNoiseSource {
    int slot = -1;
    double phase = 0.0;
    double white_current_rms = 9.0e-12;
    double hum_current_peak = 5.0e-11;
  };

  MnaSolver solver_;
  std::vector<double> source_values_;
  std::vector<DacBitSource> dac_bit_sources_;
  std::vector<TimeSource> time_sources_;
  std::vector<FloatingNoiseSource> floating_noise_sources_;
  std::uint32_t noise_state_ = 0x51f15eU;
  std::uint64_t sample_index_ = 0;
  double sample_rate_ = 48000.0;
  int left_positive_node_ = -1;
  int left_negative_node_ = -1;
  int right_positive_node_ = -1;
  int right_negative_node_ = -1;
  std::array<int, 2> speaker_coil_nodes_{-1, -1};
  std::array<int, 2> speaker_motional_nodes_{-1, -1};
  int left_ac_source_slot_ = -1;
  int right_ac_source_slot_ = -1;
  double left_ac_source_scale_ = 1.0;
  double right_ac_source_scale_ = 1.0;
  double left_dc_voltage_ = 0.0;
  double right_dc_voltage_ = 0.0;
  std::array<double, 2> speaker_dc_motional_volts_{};
  double left_full_scale_volts_ = 1.0;
  double right_full_scale_volts_ = 1.0;
  double speaker_voice_coil_resistance_ = 6.4;
  double speaker_nominal_impedance_ = 8.0;
  float last_stable_left_ = 0.0f;
  float last_stable_right_ = 0.0f;
  std::atomic<double> left_speaker_power_watts_{0.0};
  std::atomic<double> right_speaker_power_watts_{0.0};
  std::atomic<double> left_coil_temperature_c_{25.0};
  std::atomic<double> right_coil_temperature_c_{25.0};
  std::atomic<int> last_newton_iterations_{0};
  std::atomic<int> last_nonlinear_substeps_{1};
  std::atomic<double> last_residual_{0.0};
  std::atomic<bool> last_converged_{true};
  std::atomic<int> last_failure_component_instance_id_{-1};
  std::atomic<int> last_failure_frame_{-1};
  CompiledAudioCircuitMetrics metrics_;
  void InitializeDcSourceValues();
  std::map<std::pair<int, int>, int> port_nodes_;
};

}  // namespace plc::audio

#endif
