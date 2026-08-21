#ifndef PLC_EMULATOR_AUDIO_CONVERTER_MODELS_H_
#define PLC_EMULATOR_AUDIO_CONVERTER_MODELS_H_

#include <cstdint>

namespace plc::audio {

struct AdcModelConfig {
  unsigned int bits = 16;
  double minimum_volts = -1.0;
  double maximum_volts = 1.0;
  double sample_rate_hz = 48000.0;
  double acquisition_time_seconds = 1.0e-6;
  double input_resistance_ohms = 100000.0;
  double hold_capacitance_farads = 20.0e-12;
};

struct AdcSample {
  std::uint32_t code = 0;
  double held_volts = 0.0;
  bool sampled = false;
  bool clipped = false;
};

class AdcModel {
 public:
  explicit AdcModel(AdcModelConfig config = {});
  void Reset(double initial_voltage = 0.0);
  AdcSample Process(double input_voltage, double circuit_sample_rate_hz);

 private:
  AdcModelConfig config_;
  double held_voltage_ = 0.0;
  double sample_phase_ = 1.0;
};

}  // namespace plc::audio

#endif
