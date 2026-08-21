#include "plc_emulator/audio/converter_models.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace plc::audio {

AdcModel::AdcModel(AdcModelConfig config) : config_(config) {
  config_.bits = std::clamp(config_.bits, 1U, 24U);
  config_.sample_rate_hz = std::max(config_.sample_rate_hz, 1.0);
  if (config_.maximum_volts <= config_.minimum_volts) {
    config_.maximum_volts = config_.minimum_volts + 1.0;
  }
}

void AdcModel::Reset(double initial_voltage) {
  held_voltage_ = initial_voltage;
  sample_phase_ = 1.0;
}

AdcSample AdcModel::Process(double input_voltage,
                            double circuit_sample_rate_hz) {
  AdcSample result;
  const double rate = std::max(circuit_sample_rate_hz, 1.0);
  sample_phase_ += config_.sample_rate_hz / rate;
  if (sample_phase_ >= 1.0) {
    sample_phase_ -= std::floor(sample_phase_);
    const double time_constant =
        std::max(config_.input_resistance_ohms, 0.0) *
        std::max(config_.hold_capacitance_farads, 0.0);
    const double acquisition =
        std::max(config_.acquisition_time_seconds, 0.0);
    const double settled_fraction =
        time_constant > std::numeric_limits<double>::epsilon()
            ? 1.0 - std::exp(-acquisition / time_constant)
            : 1.0;
    held_voltage_ +=
        (input_voltage - held_voltage_) * settled_fraction;
    result.sampled = true;
  }
  result.clipped = held_voltage_ < config_.minimum_volts ||
                   held_voltage_ > config_.maximum_volts;
  result.held_volts = std::clamp(
      held_voltage_, config_.minimum_volts, config_.maximum_volts);
  const std::uint32_t maximum_code =
      (std::uint32_t{1} << config_.bits) - 1U;
  const double normalized =
      (result.held_volts - config_.minimum_volts) /
      (config_.maximum_volts - config_.minimum_volts);
  result.code = static_cast<std::uint32_t>(
      std::clamp(std::llround(normalized * maximum_code), 0LL,
                 static_cast<long long>(maximum_code)));
  return result;
}

}  // namespace plc::audio
