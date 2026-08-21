#include "plc_emulator/audio/compiled_audio_circuit.h"

#include "plc_emulator/components/state_keys.h"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <limits>
#include <numeric>
#include <queue>
#include <set>
#include <tuple>
#include <utility>

namespace plc::audio {
namespace {

using PortKey = std::pair<int, int>;
constexpr double kPi = 3.1415926535897932384626433832795;

double State(const PlacedComponent& component, const char* key,
             double fallback) {
  const auto found = component.internalStates.find(key);
  return found == component.internalStates.end()
             ? fallback
             : static_cast<double>(found->second);
}

bool HasState(const PlacedComponent& component, const char* key) {
  return component.internalStates.find(key) != component.internalStates.end();
}

class DisjointSet {
 public:
  int Add() {
    const int index = static_cast<int>(parent_.size());
    parent_.push_back(index);
    rank_.push_back(0);
    return index;
  }

  int Find(int value) {
    if (parent_[static_cast<size_t>(value)] != value) {
      parent_[static_cast<size_t>(value)] =
          Find(parent_[static_cast<size_t>(value)]);
    }
    return parent_[static_cast<size_t>(value)];
  }

  void Join(int left, int right) {
    left = Find(left);
    right = Find(right);
    if (left == right) return;
    if (rank_[static_cast<size_t>(left)] <
        rank_[static_cast<size_t>(right)]) {
      std::swap(left, right);
    }
    parent_[static_cast<size_t>(right)] = left;
    if (rank_[static_cast<size_t>(left)] ==
        rank_[static_cast<size_t>(right)]) {
      ++rank_[static_cast<size_t>(left)];
    }
  }

 private:
  std::vector<int> parent_;
  std::vector<int> rank_;
};

int FixedPortCount(ComponentType type) {
  switch (type) {
    case ComponentType::AUDIO_SOURCE:
      return 3;
    case ComponentType::AUDIO_ADC:
    case ComponentType::AUDIO_DAC:
    case ComponentType::AUDIO_RESISTOR:
    case ComponentType::AUDIO_CAPACITOR:
    case ComponentType::AUDIO_INDUCTOR:
    case ComponentType::AUDIO_DIODE:
    case ComponentType::AUDIO_DC_SOURCE:
    case ComponentType::AUDIO_AC_SOURCE:
    case ComponentType::AUDIO_PULSE_SOURCE:
      return 2;
    case ComponentType::AUDIO_POTENTIOMETER:
    case ComponentType::AUDIO_BJT_NPN:
    case ComponentType::AUDIO_BJT_PNP:
      return 3;
    case ComponentType::AUDIO_OP_AMP:
      return 5;
    case ComponentType::AUDIO_SPEAKER:
      return 4;
    case ComponentType::AUDIO_GROUND:
      return 1;
    default:
      return 0;
  }
}

double LogicHighVoltage(RtlLogicFamily family) {
  switch (family) {
    case RtlLogicFamily::INDUSTRIAL_24V:
      return 24.0;
    case RtlLogicFamily::TTL_5V:
    case RtlLogicFamily::CMOS_5V:
      return 5.0;
  }
  return 5.0;
}

double LogicOutputResistance(RtlLogicFamily family) {
  switch (family) {
    case RtlLogicFamily::INDUSTRIAL_24V:
      return 80.0;
    case RtlLogicFamily::TTL_5V:
      return 35.0;
    case RtlLogicFamily::CMOS_5V:
      return 25.0;
  }
  return 25.0;
}

int ParseDacBit(const std::string& name, int* channel) {
  const char* prefix = nullptr;
  if (name.rfind("dac_l[", 0) == 0) {
    prefix = "dac_l[";
    if (channel) *channel = 0;
  } else if (name.rfind("dac_r[", 0) == 0) {
    prefix = "dac_r[";
    if (channel) *channel = 1;
  } else {
    return -1;
  }
  const size_t prefix_size = std::char_traits<char>::length(prefix);
  if (name.back() != ']' || name.size() <= prefix_size + 1U) return -1;
  const std::string digits =
      name.substr(prefix_size, name.size() - prefix_size - 1U);
  char* end = nullptr;
  const long parsed = std::strtol(digits.c_str(), &end, 10);
  return end && *end == '\0' && parsed >= 0 && parsed < 16
             ? static_cast<int>(parsed)
             : -1;
}

}  // namespace

bool CompiledAudioCircuit::Compile(
    const std::vector<PlacedComponent>& components,
    const std::vector<Wire>& wires, double sample_rate) {
  metrics_ = {};
  source_values_.clear();
  dac_bit_sources_.clear();
  time_sources_.clear();
  floating_noise_sources_.clear();
  sample_index_ = 0;
  sample_rate_ = sample_rate;
  left_ac_source_slot_ = -1;
  right_ac_source_slot_ = -1;
  std::map<PortKey, int> port_indices;
  DisjointSet sets;
  auto ensure_port = [&](PortKey key) {
    const auto found = port_indices.find(key);
    if (found != port_indices.end()) return found->second;
    const int index = sets.Add();
    port_indices.emplace(key, index);
    return index;
  };
  for (const PlacedComponent& component : components) {
    if (component.type == ComponentType::RTL_MODULE) {
      for (const RtlPinBinding& binding : component.rtlPinBindings) {
        ensure_port({component.instanceId, binding.portId});
      }
    } else {
      const int count = FixedPortCount(component.type);
      for (int port = 0; port < count; ++port) {
        ensure_port({component.instanceId, port});
      }
    }
  }
  for (const Wire& wire : wires) {
    if (!wire.isElectric) continue;
    sets.Join(ensure_port({wire.fromComponentId, wire.fromPortId}),
              ensure_port({wire.toComponentId, wire.toPortId}));
  }
  int common_ground_index = -1;
  for (const PlacedComponent& component : components) {
    if (component.type != ComponentType::AUDIO_GROUND) continue;
    const int ground_index = ensure_port({component.instanceId, 0});
    if (common_ground_index < 0) {
      common_ground_index = ground_index;
    } else {
      sets.Join(common_ground_index, ground_index);
    }
  }
  if (common_ground_index < 0) {
    metrics_.error = "MNA requires an audio ground component.";
    return false;
  }
  common_ground_index = sets.Find(common_ground_index);
  std::map<int, int> root_nodes;
  root_nodes.emplace(common_ground_index, 0);
  int node_count = 1;
  for (const auto& entry : port_indices) {
    const int root = sets.Find(entry.second);
    if (root_nodes.count(root) == 0) root_nodes.emplace(root, node_count++);
  }
  const auto port_node = [&](int component, int port) {
    const auto found = port_indices.find({component, port});
    if (found == port_indices.end()) return 0;
    return root_nodes.at(sets.Find(found->second));
  };
  port_nodes_.clear();
  for (const auto& entry : port_indices) {
    port_nodes_[entry.first] = root_nodes.at(sets.Find(entry.second));
  }
  auto new_internal_node = [&]() { return node_count++; };

  std::vector<MnaElement> elements;
  int dynamic_elements = 0;
  int source_slot_count = 0;
  const PlacedComponent* speaker = nullptr;
  for (const PlacedComponent& component : components) {
    bool uses_default_model = false;
    switch (component.type) {
      case ComponentType::AUDIO_CAPACITOR:
        uses_default_model =
            !HasState(component, state_keys::kEsrOhms) ||
            !HasState(component, state_keys::kLeakageResistanceOhms) ||
            !HasState(component, state_keys::kVoltageRatingVolts);
        break;
      case ComponentType::AUDIO_INDUCTOR:
        uses_default_model =
            !HasState(component, state_keys::kDcrOhms) ||
            !HasState(component, state_keys::kSaturationCurrentAmps);
        break;
      case ComponentType::AUDIO_DC_SOURCE:
        uses_default_model =
            !HasState(component, state_keys::kInternalResistanceOhms);
        break;
      case ComponentType::AUDIO_SPEAKER:
        uses_default_model =
            !HasState(component, state_keys::kSpeakerReOhms) ||
            !HasState(component, state_keys::kSpeakerLeMh) ||
            !HasState(component, state_keys::kSpeakerBlTeslaMeters) ||
            !HasState(component, state_keys::kSpeakerMmsGrams) ||
            !HasState(component, state_keys::kSpeakerCmsMmPerNewton) ||
            !HasState(component,
                      state_keys::kSpeakerRmsNewtonSecondsPerMeter);
        break;
      default:
        break;
    }
    if (uses_default_model) {
      metrics_.default_model_component_ids.push_back(component.instanceId);
    }
    switch (component.type) {
      case ComponentType::AUDIO_RESISTOR: {
        const double nominal =
            std::max(State(component, state_keys::kResistanceOhms, 1000.0),
                     1.0e-6);
        const double tolerance = std::clamp(
            State(component, state_keys::kTolerancePercent, 1.0), 0.0, 20.0) /
            100.0;
        const std::uint32_t spread =
            static_cast<std::uint32_t>(component.instanceId) * 2654435761U;
        const double signed_error =
            static_cast<double>(spread & 0xffffU) / 32767.5 - 1.0;
        const double resistance =
            nominal * (1.0 + tolerance * signed_error);
        const int positive = port_node(component.instanceId, 0);
        const int negative = port_node(component.instanceId, 1);
        elements.push_back({MnaElementType::RESISTOR,
                            positive, negative, 0, 0, resistance, 0.0, -1,
                            component.instanceId});
        constexpr double kBoltzmann = 1.380649e-23;
        constexpr double kRoomTemperatureKelvin = 298.15;
        const int noise_slot = source_slot_count++;
        elements.push_back({MnaElementType::CURRENT_SOURCE, positive,
                            negative, 0, 0, 0.0, 0.0, noise_slot,
                            component.instanceId});
        floating_noise_sources_.push_back(
            {noise_slot,
             static_cast<double>(component.instanceId) * 0.173,
             std::sqrt(4.0 * kBoltzmann * kRoomTemperatureKelvin /
                       resistance * (sample_rate * 0.5)),
             0.0});
        break;
      }
      case ComponentType::AUDIO_CAPACITOR: {
        const int positive = port_node(component.instanceId, 0);
        const int negative = port_node(component.instanceId, 1);
        const int capacitor_node = new_internal_node();
        const double esr = std::max(
            State(component, state_keys::kEsrOhms, 0.05), 1.0e-6);
        const double leakage = std::max(
            State(component, state_keys::kLeakageResistanceOhms, 1.0e8),
            1.0);
        elements.push_back({MnaElementType::RESISTOR, positive,
                            capacitor_node, 0, 0, esr, 0.0, -1,
                            component.instanceId});
        elements.push_back(
            {MnaElementType::CAPACITOR,
             capacitor_node, negative, 0, 0,
             std::max(State(component, state_keys::kCapacitanceUf, 10.0),
                      1.0e-9) *
                 1.0e-6,
             0.0, -1, component.instanceId});
        elements.push_back({MnaElementType::RESISTOR, positive, negative, 0,
                            0, leakage, 0.0, -1,
                            component.instanceId});
        ++dynamic_elements;
        break;
      }
      case ComponentType::AUDIO_INDUCTOR: {
        const int positive = port_node(component.instanceId, 0);
        const int negative = port_node(component.instanceId, 1);
        const int inductor_node = new_internal_node();
        elements.push_back({MnaElementType::RESISTOR, positive,
                            inductor_node, 0, 0,
                            std::max(State(component, state_keys::kDcrOhms,
                                           0.1),
                                     1.0e-6),
                            0.0, -1, component.instanceId});
        elements.push_back(
            {MnaElementType::INDUCTOR,
             inductor_node, negative, 0, 0,
             std::max(State(component, state_keys::kInductanceMh, 10.0),
                      1.0e-9) *
                 1.0e-3,
             0.0, -1, component.instanceId});
        ++dynamic_elements;
        break;
      }
      case ComponentType::AUDIO_POTENTIOMETER: {
        const double resistance =
            std::max(State(component, state_keys::kResistanceOhms, 10000.0),
                     1.0e-3);
        const double wiper = std::clamp(
            State(component, state_keys::kWiperPosition, 0.5), 0.000001,
            0.999999);
        const int end_a = port_node(component.instanceId, 0);
        const int output = port_node(component.instanceId, 1);
        const int end_b = port_node(component.instanceId, 2);
        elements.push_back({MnaElementType::RESISTOR, end_a, output, 0, 0,
                            resistance * (1.0 - wiper), 0.0, -1,
                            component.instanceId});
        elements.push_back({MnaElementType::RESISTOR, output, end_b, 0, 0,
                            resistance * wiper, 0.0, -1,
                            component.instanceId});
        break;
      }
      case ComponentType::AUDIO_DC_SOURCE: {
        const int positive = port_node(component.instanceId, 0);
        const int negative = port_node(component.instanceId, 1);
        const double voltage =
            State(component, state_keys::kVoltageVolts, 5.0);
        const double current_limit = std::max(
            State(component, state_keys::kCurrentLimitAmps, 1.0), 0.001);
        const double output_resistance = std::max(
            State(component, state_keys::kInternalResistanceOhms, 0.05),
            std::max(std::abs(voltage) / (current_limit * 10.0), 0.0001));
        elements.push_back({MnaElementType::RESISTOR, positive, negative, 0,
                            0, output_resistance, 0.0, -1,
                            component.instanceId});
        elements.push_back({MnaElementType::CURRENT_SOURCE, negative,
                            positive, 0, 0, voltage / output_resistance, 0.0,
                            -1, component.instanceId});
        break;
      }
      case ComponentType::AUDIO_AC_SOURCE:
      case ComponentType::AUDIO_PULSE_SOURCE: {
        const int source_slot = source_slot_count++;
        elements.push_back(
            {MnaElementType::VOLTAGE_SOURCE,
             port_node(component.instanceId, 0),
             port_node(component.instanceId, 1), 0, 0, 0.0, 0.0,
             source_slot, component.instanceId});
        TimeSource source;
        source.slot = source_slot;
        source.frequency_hz = std::max(
            State(component, state_keys::kFrequencyHz, 1000.0), 0.0);
        source.phase_radians =
            State(component, state_keys::kPhaseDegrees, 0.0) * kPi / 180.0;
        if (component.type == ComponentType::AUDIO_AC_SOURCE) {
          source.type = TimeSource::Type::SINE;
          source.amplitude =
              State(component, state_keys::kVoltageVolts, 1.0);
        } else {
          source.type = TimeSource::Type::PULSE;
          source.low_volts =
              State(component, state_keys::kPulseLowVolts, 0.0);
          source.high_volts =
              State(component, state_keys::kPulseHighVolts, 5.0);
          source.duty = std::clamp(
              State(component, state_keys::kDutyCyclePercent, 50.0) / 100.0,
              0.0, 1.0);
        }
        time_sources_.push_back(source);
        break;
      }
      case ComponentType::AUDIO_OP_AMP: {
        const int drive = new_internal_node();
        const int filtered = new_internal_node();
        const int offset_input = new_internal_node();
        const int positive_supply = port_node(component.instanceId, 2);
        const int negative_supply = port_node(component.instanceId, 3);
        const int output = port_node(component.instanceId, 4);
        const double open_loop_gain = std::clamp(
            State(component, state_keys::kOpenLoopGain, 100000.0), 1.0,
            1.0e9);
        const double gain_bandwidth = std::max(
            State(component, state_keys::kGainBandwidthHz, 1000000.0), 1.0);
        const double pole = gain_bandwidth / open_loop_gain;
        constexpr double kPoleResistance = 1000.0;
        const double pole_capacitance =
            1.0 / (2.0 * kPi * kPoleResistance * pole);
        const double input_offset_volts =
            State(component, state_keys::kInputOffsetMillivolts, 1.0) *
            1.0e-3;
        elements.push_back({MnaElementType::VOLTAGE_SOURCE, offset_input,
                            port_node(component.instanceId, 1), 0, 0,
                            input_offset_volts, 0.0, -1,
                            component.instanceId});
        elements.push_back({MnaElementType::RESISTOR,
                            port_node(component.instanceId, 1),
                            port_node(component.instanceId, 0), 0, 0,
                            2.0e6, 0.0, -1, component.instanceId});
        elements.push_back(
            {MnaElementType::VCVS, drive, negative_supply,
             offset_input,
             port_node(component.instanceId, 0), open_loop_gain, 0.0, -1,
             component.instanceId});
        elements.push_back({MnaElementType::RESISTOR, drive, filtered, 0, 0,
                            kPoleResistance, 0.0, -1,
                            component.instanceId});
        elements.push_back({MnaElementType::CAPACITOR, filtered,
                            negative_supply, 0, 0, pole_capacitance, 0.0, -1,
                            component.instanceId});
        // Junction clamps make the large-signal output follow the actual
        // supply rails instead of an unlimited ideal VCVS.
        elements.push_back({MnaElementType::DIODE, filtered,
                            positive_supply, 0, 0, 1.0e-14, 1.0, -1,
                            component.instanceId});
        elements.push_back({MnaElementType::DIODE, negative_supply,
                            filtered, 0, 0, 1.0e-14, 1.0, -1,
                            component.instanceId});
        const double current_limit = std::max(
            State(component, state_keys::kOutputCurrentLimitAmps, 0.03),
            1.0e-4);
        constexpr double kNominalHalfSupplyVolts = 15.0;
        const double output_resistance = std::max(
            std::max(State(component, state_keys::kOutputResistanceOhms,
                           50.0),
                     0.001),
            kNominalHalfSupplyVolts / current_limit);
        elements.push_back({MnaElementType::RESISTOR, filtered, output, 0, 0,
                            output_resistance, 0.0, -1,
                            component.instanceId});
        const double slew_rate_volts_per_second =
            std::max(State(component, state_keys::kSlewRateVoltsPerUs, 0.5),
                     0.001) *
            1.0e6;
        elements.push_back({MnaElementType::CAPACITOR, output,
                            negative_supply, 0, 0,
                            current_limit / slew_rate_volts_per_second, 0.0,
                            -1, component.instanceId});
        dynamic_elements += 2;
        break;
      }
      case ComponentType::AUDIO_DIODE: {
        constexpr double kNominalBiasCurrentAmps = 1.0e-3;
        constexpr double kThermalVoltageVolts = 0.02585;
        const double forward_voltage = std::clamp(
            State(component, state_keys::kForwardVoltageVolts, 0.65),
            0.05, 1.2);
        const double dynamic_resistance = std::max(
            State(component, state_keys::kDynamicResistanceOhms, 2.0),
            0.001);
        if (State(component, state_keys::kThermalCoupling, 0.0) >= 0.5) {
          // A heatsink-coupled Class-AB bias diode is operated around a fixed
          // standing current. Model its local Vf + rd slope directly: this is
          // the useful audio-band small-signal model and avoids spending a
          // Newton junction on four bias devices every sample. Ordinary
          // rectifier/clipping diodes still use the Shockley model below.
          const int series_node = new_internal_node();
          elements.push_back({MnaElementType::VOLTAGE_SOURCE,
                              port_node(component.instanceId, 0),
                              series_node, 0, 0, forward_voltage, 0.0, -1,
                              component.instanceId});
          elements.push_back({MnaElementType::RESISTOR, series_node,
                              port_node(component.instanceId, 1), 0, 0,
                              dynamic_resistance, 0.0, -1,
                              component.instanceId});
          break;
        }
        const double saturation_current =
            kNominalBiasCurrentAmps /
            std::expm1(forward_voltage / kThermalVoltageVolts);
        elements.push_back(
            {MnaElementType::DIODE,
             port_node(component.instanceId, 0),
             port_node(component.instanceId, 1), 0, 0,
             saturation_current,
             dynamic_resistance,
             -1, component.instanceId});
        break;
      }
      case ComponentType::AUDIO_BJT_NPN:
      case ComponentType::AUDIO_BJT_PNP: {
        const int base = port_node(component.instanceId, 0);
        const int collector = port_node(component.instanceId, 1);
        const int emitter = port_node(component.instanceId, 2);
        const double beta = std::max(
            State(component, state_keys::kCurrentGainBeta, 100.0), 1.0);
        elements.push_back(
            {component.type == ComponentType::AUDIO_BJT_NPN
                 ? MnaElementType::BJT_NPN
                 : MnaElementType::BJT_PNP,
             collector, emitter, base, emitter, 1.0e-14, beta, -1,
             component.instanceId, 100.0});
        break;
      }
      case ComponentType::AUDIO_SPEAKER:
        speaker = &component;
        break;
      default:
        break;
    }
  }

  if (!speaker) {
    metrics_.error = "MNA requires a stereo speaker load.";
    return false;
  }
  left_positive_node_ = port_node(speaker->instanceId, 0);
  left_negative_node_ = port_node(speaker->instanceId, 1);
  right_positive_node_ = port_node(speaker->instanceId, 2);
  right_negative_node_ = port_node(speaker->instanceId, 3);
  const double nominal_impedance = std::max(
      State(*speaker, state_keys::kImpedanceOhms, 8.0), 0.1);
  speaker_nominal_impedance_ = nominal_impedance;
  const double voice_coil_resistance = std::max(
      State(*speaker, state_keys::kSpeakerReOhms,
            nominal_impedance * 0.8),
      0.01);
  speaker_voice_coil_resistance_ = voice_coil_resistance;
  const double voice_coil_inductance = std::max(
      State(*speaker, state_keys::kSpeakerLeMh,
            nominal_impedance <= 64.0 ? 0.5 : 0.1) *
          1.0e-3,
      1.0e-9);
  const double force_factor = std::max(
      State(*speaker, state_keys::kSpeakerBlTeslaMeters, 7.0), 0.01);
  const double moving_mass = std::max(
      State(*speaker, state_keys::kSpeakerMmsGrams, 15.0) * 1.0e-3,
      1.0e-6);
  const double compliance = std::max(
      State(*speaker, state_keys::kSpeakerCmsMmPerNewton, 0.5) * 1.0e-3,
      1.0e-9);
  const double mechanical_loss = std::max(
      State(*speaker, state_keys::kSpeakerRmsNewtonSecondsPerMeter, 1.5),
      1.0e-6);
  const double reflected_resistance =
      force_factor * force_factor / mechanical_loss;
  const double reflected_inductance =
      force_factor * force_factor * compliance;
  const double reflected_capacitance =
      moving_mass / (force_factor * force_factor);
  for (int channel = 0; channel < 2; ++channel) {
    const int positive = channel == 0 ? left_positive_node_
                                      : right_positive_node_;
    const int negative = channel == 0 ? left_negative_node_
                                      : right_negative_node_;
    const int coil_node = new_internal_node();
    const int mechanical_node = new_internal_node();
    speaker_coil_nodes_[static_cast<size_t>(channel)] = coil_node;
    speaker_motional_nodes_[static_cast<size_t>(channel)] = mechanical_node;
    elements.push_back({MnaElementType::RESISTOR, positive, coil_node, 0, 0,
                        voice_coil_resistance, 0.0, -1,
                        speaker->instanceId});
    elements.push_back({MnaElementType::INDUCTOR, coil_node,
                        mechanical_node, 0, 0,
                        voice_coil_inductance, 0.0, -1,
                        speaker->instanceId});
    elements.push_back({MnaElementType::RESISTOR, mechanical_node, negative,
                        0, 0, reflected_resistance, 0.0, -1,
                        speaker->instanceId});
    elements.push_back({MnaElementType::INDUCTOR, mechanical_node, negative,
                        0, 0, reflected_inductance, 0.0, -1,
                        speaker->instanceId});
    elements.push_back({MnaElementType::CAPACITOR, mechanical_node, negative,
                        0, 0, reflected_capacitance, 0.0, -1,
                        speaker->instanceId});
    dynamic_elements += 3;
  }
  const double rated_power = std::max(
      State(*speaker, state_keys::kMaximumPowerWatts, 100.0), 0.1);
  const double rated_peak =
      std::sqrt(2.0 * rated_power * nominal_impedance);
  left_full_scale_volts_ = rated_peak;
  right_full_scale_volts_ = rated_peak;

  // Give genuinely floating electrical nodes a realistic high-value leakage
  // path plus tiny thermal/mains-coupled current. They remain noisy instead
  // of being treated as a Boolean mute condition.
  std::vector<std::vector<int>> dc_connectivity(
      static_cast<size_t>(node_count));
  for (const MnaElement& element : elements) {
    if (element.type == MnaElementType::CURRENT_SOURCE ||
        element.type == MnaElementType::VCCS) {
      continue;
    }
    if (element.positive_node != element.negative_node) {
      dc_connectivity[static_cast<size_t>(element.positive_node)]
          .push_back(element.negative_node);
      dc_connectivity[static_cast<size_t>(element.negative_node)]
          .push_back(element.positive_node);
    }
  }
  std::vector<bool> grounded(static_cast<size_t>(node_count), false);
  std::queue<int> ground_search;
  grounded[0] = true;
  ground_search.push(0);
  while (!ground_search.empty()) {
    const int node = ground_search.front();
    ground_search.pop();
    for (int next : dc_connectivity[static_cast<size_t>(node)]) {
      if (!grounded[static_cast<size_t>(next)]) {
        grounded[static_cast<size_t>(next)] = true;
        ground_search.push(next);
      }
    }
  }
  constexpr double kFloatingLeakageOhms = 1.0e8;
  for (int node = 1; node < node_count; ++node) {
    if (grounded[static_cast<size_t>(node)]) continue;
    elements.push_back({MnaElementType::RESISTOR, node, 0, 0, 0,
                        kFloatingLeakageOhms, 0.0, -1, -1});
    const int noise_slot = source_slot_count++;
    elements.push_back({MnaElementType::CURRENT_SOURCE, 0, node, 0, 0,
                        0.0, 0.0, noise_slot, -1});
    floating_noise_sources_.push_back(
        {noise_slot, static_cast<double>(node) * 0.731, 9.0e-12,
         5.0e-11});
  }

  int slot = source_slot_count;
  std::set<int> dac_component_ids;
  for (const PlacedComponent& component : components) {
    if (component.type != ComponentType::RTL_MODULE ||
        component.rtlModuleId != "audio_shell_dac") {
      continue;
    }
    dac_component_ids.insert(component.instanceId);
    const double high_voltage = LogicHighVoltage(component.rtlLogicFamily);
    const double output_resistance =
        LogicOutputResistance(component.rtlLogicFamily);
    for (const RtlPinBinding& binding : component.rtlPinBindings) {
      if (binding.isInput) continue;
      int channel = 0;
      const int bit = ParseDacBit(binding.pinName, &channel);
      if (bit < 0) continue;
      const int output_node = port_node(component.instanceId, binding.portId);
      elements.push_back({MnaElementType::RESISTOR, output_node, 0, 0, 0,
                          output_resistance, 0.0, -1,
                          component.instanceId});
      elements.push_back({MnaElementType::CURRENT_SOURCE, 0, output_node, 0,
                          0, 0.0, 0.0, slot,
                          component.instanceId});
      dac_bit_sources_.push_back(
          {slot, channel, bit, output_node,
           high_voltage / output_resistance});
      if (bit == 15) {
        if (channel == 0) {
          left_ac_source_slot_ = slot;
          left_ac_source_scale_ = high_voltage / output_resistance;
        } else {
          right_ac_source_slot_ = slot;
          right_ac_source_scale_ = high_voltage / output_resistance;
        }
      }
      ++slot;
    }
  }
  source_values_.assign(static_cast<size_t>(slot), 0.0);
  InitializeDcSourceValues();
  std::vector<std::vector<int>> signal_graph(static_cast<size_t>(node_count));
  const auto connect_signal = [&](int from, int to, bool bidirectional) {
    if (from < 0 || to < 0 || from >= node_count || to >= node_count ||
        from == to) {
      return;
    }
    signal_graph[static_cast<size_t>(from)].push_back(to);
    if (bidirectional) {
      signal_graph[static_cast<size_t>(to)].push_back(from);
    }
  };
  for (const MnaElement& element : elements) {
    if (element.component_instance_id == speaker->instanceId ||
        dac_component_ids.count(element.component_instance_id) != 0) {
      continue;
    }
    switch (element.type) {
      case MnaElementType::RESISTOR:
      case MnaElementType::CAPACITOR:
      case MnaElementType::INDUCTOR:
      case MnaElementType::DIODE:
      case MnaElementType::VOLTAGE_SOURCE:
        connect_signal(element.positive_node, element.negative_node, true);
        break;
      case MnaElementType::VCVS:
      case MnaElementType::VCCS:
        connect_signal(element.control_positive_node, element.positive_node,
                       false);
        connect_signal(element.control_negative_node, element.positive_node,
                       false);
        connect_signal(element.control_positive_node, element.negative_node,
                       false);
        connect_signal(element.control_negative_node, element.negative_node,
                       false);
        break;
      case MnaElementType::BJT_NPN:
      case MnaElementType::BJT_PNP:
        connect_signal(element.control_positive_node, element.positive_node,
                       false);
        connect_signal(element.control_positive_node, element.negative_node,
                       false);
        connect_signal(element.positive_node, element.negative_node, true);
        break;
      case MnaElementType::CURRENT_SOURCE:
        break;
    }
  }
  const auto reaches = [&](int channel, int target) {
    std::vector<bool> visited(static_cast<size_t>(node_count), false);
    std::queue<int> pending;
    for (const DacBitSource& source : dac_bit_sources_) {
      if (source.channel == channel &&
          !visited[static_cast<size_t>(source.output_node)]) {
        visited[static_cast<size_t>(source.output_node)] = true;
        pending.push(source.output_node);
      }
    }
    while (!pending.empty()) {
      const int node = pending.front();
      pending.pop();
      if (node == target) return true;
      for (int next : signal_graph[static_cast<size_t>(node)]) {
        if (!visited[static_cast<size_t>(next)]) {
          visited[static_cast<size_t>(next)] = true;
          pending.push(next);
        }
      }
    }
    return false;
  };
  MnaSolveMetrics solve_metrics;
  if (!solver_.Compile(node_count, 0, sample_rate, elements,
                       &solve_metrics) ||
      !solver_.SolveDc(source_values_, &solve_metrics)) {
    metrics_.error = solve_metrics.error;
    return false;
  }
  left_dc_voltage_ = solver_.NodeVoltage(left_positive_node_) -
                     solver_.NodeVoltage(left_negative_node_);
  right_dc_voltage_ = solver_.NodeVoltage(right_positive_node_) -
                      solver_.NodeVoltage(right_negative_node_);
  speaker_dc_motional_volts_[0] =
      solver_.NodeVoltage(speaker_motional_nodes_[0]) -
      solver_.NodeVoltage(left_negative_node_);
  speaker_dc_motional_volts_[1] =
      solver_.NodeVoltage(speaker_motional_nodes_[1]) -
      solver_.NodeVoltage(right_negative_node_);
  metrics_.valid = true;
  metrics_.electrical_nodes = node_count;
  metrics_.matrix_order = solver_.MatrixOrder();
  metrics_.reduced_matrix_order = solver_.ReducedMatrixOrder();
  metrics_.eliminated_unknowns = solver_.EliminatedUnknownCount();
  metrics_.dynamic_elements = dynamic_elements;
  metrics_.dac_output_bits = static_cast<int>(dac_bit_sources_.size());
  metrics_.left_signal_route = reaches(0, left_positive_node_);
  metrics_.right_signal_route = reaches(1, right_positive_node_);
  metrics_.left_return_grounded = left_negative_node_ == 0;
  metrics_.right_return_grounded = right_negative_node_ == 0;
  metrics_.dc_left_volts = left_dc_voltage_;
  metrics_.dc_right_volts = right_dc_voltage_;
  metrics_.dc_node_voltages = solver_.NodeVoltages();
  std::map<int, CompiledComponentOperatingPoint> operating_points;
  for (const MnaElement& element : elements) {
    if (element.component_instance_id < 0) continue;
    const double voltage =
        solver_.NodeVoltage(element.positive_node) -
        solver_.NodeVoltage(element.negative_node);
    double current = 0.0;
    switch (element.type) {
      case MnaElementType::RESISTOR:
        current = voltage /
                  std::max(std::abs(element.value), 1.0e-9);
        break;
      case MnaElementType::DIODE:
        current = std::clamp(
            std::max(std::abs(element.value), 1.0e-18) *
                std::expm1(std::clamp(voltage, -5.0, 1.2) /
                           0.02585),
            -10.0, 10.0);
        break;
      case MnaElementType::CURRENT_SOURCE:
        current = element.source_slot >= 0 ? 0.0 : element.value;
        break;
      case MnaElementType::VCCS:
        current =
            element.value *
            (solver_.NodeVoltage(element.control_positive_node) -
             solver_.NodeVoltage(element.control_negative_node));
        break;
      case MnaElementType::BJT_NPN:
      case MnaElementType::BJT_PNP: {
        const double polarity =
            element.type == MnaElementType::BJT_NPN ? 1.0 : -1.0;
        const double vbe = polarity *
                           (solver_.NodeVoltage(
                                element.control_positive_node) -
                            solver_.NodeVoltage(element.negative_node));
        const double vce = polarity * voltage;
        const double early_voltage =
            std::max(std::abs(element.auxiliary_value), 1.0);
        current = polarity * std::clamp(
                                 std::max(std::abs(element.value), 1.0e-18) *
                                     std::expm1(std::clamp(vbe, -5.0, 0.8) /
                                                0.02585) *
                                     std::max(0.05,
                                              1.0 + vce / early_voltage),
                                 -100.0, 100.0);
        break;
      }
      case MnaElementType::CAPACITOR:
      case MnaElementType::INDUCTOR:
      case MnaElementType::VOLTAGE_SOURCE:
      case MnaElementType::VCVS:
        break;
    }
    CompiledComponentOperatingPoint& point =
        operating_points[element.component_instance_id];
    point.component_instance_id = element.component_instance_id;
    point.current_amps = std::max(point.current_amps, std::abs(current));
    point.power_watts += std::abs(voltage * current);
  }
  for (auto& [component_id, point] : operating_points) {
    const auto component = std::find_if(
        components.begin(), components.end(),
        [component_id](const PlacedComponent& candidate) {
          return candidate.instanceId == component_id;
        });
    const double thermal_resistance =
        component != components.end()
            ? std::max(State(*component,
                             state_keys::kThermalResistanceCPerW, 35.0),
                       0.0)
            : 35.0;
    point.junction_temperature_c =
        25.0 + point.power_watts * thermal_resistance;
    metrics_.component_operating_points.push_back(point);
  }
  return true;
}

bool CompiledAudioCircuit::ProcessBlock(float* interleaved_stereo,
                                        size_t frame_count) {
  if (!metrics_.valid || !interleaved_stereo) return false;
  MnaSolveMetrics solve_metrics;
  double left_power_sum = 0.0;
  double right_power_sum = 0.0;
  for (size_t frame = 0; frame < frame_count; ++frame) {
    const double time_seconds =
        static_cast<double>(sample_index_) / sample_rate_;
    for (const TimeSource& source : time_sources_) {
      if (source.type == TimeSource::Type::SINE) {
        source_values_[static_cast<size_t>(source.slot)] =
            source.amplitude *
            std::sin(2.0 * kPi * source.frequency_hz * time_seconds +
                     source.phase_radians);
      } else {
        const double phase = source.frequency_hz > 0.0
                                 ? std::fmod(time_seconds *
                                                 source.frequency_hz +
                                             source.phase_radians /
                                                 (2.0 * kPi),
                                             1.0)
                                 : 0.0;
        source_values_[static_cast<size_t>(source.slot)] =
            phase < source.duty ? source.high_volts : source.low_volts;
      }
    }
    for (const FloatingNoiseSource& source : floating_noise_sources_) {
      noise_state_ = noise_state_ * 1664525U + 1013904223U;
      const double white =
          static_cast<double>((noise_state_ >> 8U) & 0x00ffffffU) /
              static_cast<double>(0x00ffffffU) *
              2.0 -
          1.0;
      // Johnson-noise sources have no mains component. Avoid dozens of
      // transcendental calls per sample for the common zero-hum case.
      const double hum = source.hum_current_peak != 0.0
                             ? std::sin(2.0 * kPi * 60.0 * time_seconds +
                                        source.phase)
                             : 0.0;
      source_values_[static_cast<size_t>(source.slot)] =
          white * source.white_current_rms * std::sqrt(3.0) +
          hum * source.hum_current_peak;
    }
    std::array<std::uint16_t, 2> codes{};
    for (int channel = 0; channel < 2; ++channel) {
      const float normalized = std::clamp(
          interleaved_stereo[frame * 2 + static_cast<size_t>(channel)],
          -1.0f, 1.0f);
      codes[static_cast<size_t>(channel)] = static_cast<std::uint16_t>(
          std::clamp(std::lround(static_cast<double>(normalized) * 32768.0 +
                                 32768.0),
                     0L, 65535L));
    }
    for (const DacBitSource& source : dac_bit_sources_) {
      const bool high =
          (codes[static_cast<size_t>(source.channel)] &
           (std::uint16_t{1} << static_cast<unsigned int>(source.bit))) != 0;
      source_values_[static_cast<size_t>(source.slot)] =
          high ? source.high_current_amps : 0.0;
    }
    if (!solver_.ProcessSample(source_values_, &solve_metrics)) {
      constexpr size_t kFailureFadeFrames = 64;
      for (size_t remaining = frame; remaining < frame_count; ++remaining) {
        const size_t fade_index = remaining - frame;
        const float gain = fade_index < kFailureFadeFrames
                               ? 1.0f - static_cast<float>(fade_index + 1) /
                                            static_cast<float>(kFailureFadeFrames)
                               : 0.0f;
        interleaved_stereo[remaining * 2] = last_stable_left_ * gain;
        interleaved_stereo[remaining * 2 + 1] = last_stable_right_ * gain;
      }
      last_stable_left_ = 0.0f;
      last_stable_right_ = 0.0f;
      last_newton_iterations_.store(solve_metrics.iterations,
                                    std::memory_order_relaxed);
      last_nonlinear_substeps_.store(solve_metrics.substeps,
                                     std::memory_order_relaxed);
      last_residual_.store(solve_metrics.maximum_residual,
                           std::memory_order_relaxed);
      last_converged_.store(false, std::memory_order_relaxed);
      last_failure_component_instance_id_.store(
          solve_metrics.failure_component_instance_id,
          std::memory_order_relaxed);
      last_failure_frame_.store(static_cast<int>(frame),
                                std::memory_order_relaxed);
      return false;
    }
    const double left_terminal_voltage =
        solver_.NodeVoltage(left_positive_node_) -
        solver_.NodeVoltage(left_negative_node_);
    const double right_terminal_voltage =
        solver_.NodeVoltage(right_positive_node_) -
        solver_.NodeVoltage(right_negative_node_);
    const double left_ac_voltage =
        std::isfinite(left_terminal_voltage)
            ? left_terminal_voltage - left_dc_voltage_
            : 0.0;
    const double right_ac_voltage =
        std::isfinite(right_terminal_voltage)
            ? right_terminal_voltage - right_dc_voltage_
            : 0.0;
    // The old metric used the voltage between Re and the numerical motional
    // companion node. Trapezoidal LC state voltage can be much larger than the
    // speaker terminal voltage and reported multi-kilowatt heating on a 41 V
    // rail. Rated speaker power is the real terminal AC power; DC heating is
    // tracked separately through the terminal DC offset below.
    const double left_ac_power =
        speaker_nominal_impedance_ > 0.0
            ? (left_ac_voltage * left_ac_voltage) / speaker_nominal_impedance_
            : 0.0;
    const double left_dc_power =
        speaker_voice_coil_resistance_ > 0.0
            ? (left_dc_voltage_ * left_dc_voltage_) /
                  speaker_voice_coil_resistance_
            : 0.0;
    const double right_ac_power =
        speaker_nominal_impedance_ > 0.0
            ? (right_ac_voltage * right_ac_voltage) / speaker_nominal_impedance_
            : 0.0;
    const double right_dc_power =
        speaker_voice_coil_resistance_ > 0.0
            ? (right_dc_voltage_ * right_dc_voltage_) /
                  speaker_voice_coil_resistance_
            : 0.0;
    constexpr double kMaxSamplePowerWatts = 1000.0;
    left_power_sum += std::clamp(
        (std::isfinite(left_ac_power) ? left_ac_power : 0.0) +
            (std::isfinite(left_dc_power) ? left_dc_power : 0.0),
        0.0, kMaxSamplePowerWatts);
    right_power_sum += std::clamp(
        (std::isfinite(right_ac_power) ? right_ac_power : 0.0) +
            (std::isfinite(right_dc_power) ? right_dc_power : 0.0),
        0.0, kMaxSamplePowerWatts);
    const double left = left_ac_voltage;
    const double right = right_ac_voltage;
    const float left_norm = static_cast<float>(
        left_full_scale_volts_ > 0.0
            ? std::clamp(left / left_full_scale_volts_, -1.0, 1.0)
            : 0.0);
    const float right_norm = static_cast<float>(
        right_full_scale_volts_ > 0.0
            ? std::clamp(right / right_full_scale_volts_, -1.0, 1.0)
            : 0.0);
    interleaved_stereo[frame * 2] =
        std::isfinite(left_norm) ? left_norm : 0.0f;
    interleaved_stereo[frame * 2 + 1] =
        std::isfinite(right_norm) ? right_norm : 0.0f;
    last_stable_left_ = interleaved_stereo[frame * 2];
    last_stable_right_ = interleaved_stereo[frame * 2 + 1];
    ++sample_index_;
  }
  const double left_power =
      frame_count > 0 ? std::clamp(left_power_sum / static_cast<double>(frame_count),
                                   0.0, 1000.0)
                      : 0.0;
  const double right_power =
      frame_count > 0 ? std::clamp(right_power_sum / static_cast<double>(frame_count),
                                   0.0, 1000.0)
                      : 0.0;
  left_speaker_power_watts_.store(
      std::isfinite(left_power) ? left_power : 0.0, std::memory_order_relaxed);
  right_speaker_power_watts_.store(
      std::isfinite(right_power) ? right_power : 0.0, std::memory_order_relaxed);
  const double block_seconds =
      static_cast<double>(frame_count) / sample_rate_;
  const double thermal_alpha =
      1.0 - std::exp(-block_seconds / 60.0);
  const auto update_temperature = [&](std::atomic<double>* temperature,
                                      double power) {
    const double raw_prev = temperature->load(std::memory_order_relaxed);
    const double previous =
        std::isfinite(raw_prev) ? std::clamp(raw_prev, 20.0, 350.0) : 25.0;
    const double target = std::clamp(25.0 + (std::isfinite(power) ? power : 0.0) * 2.5,
                                     20.0, 350.0);
    const double next_temp = std::clamp(
        previous + (target - previous) * thermal_alpha, 20.0, 350.0);
    temperature->store(next_temp, std::memory_order_relaxed);
  };
  update_temperature(&left_coil_temperature_c_, left_power);
  update_temperature(&right_coil_temperature_c_, right_power);
  last_newton_iterations_.store(solve_metrics.iterations,
                                std::memory_order_relaxed);
  last_nonlinear_substeps_.store(solve_metrics.substeps,
                                 std::memory_order_relaxed);
  last_residual_.store(solve_metrics.maximum_residual,
                       std::memory_order_relaxed);
  last_converged_.store(true, std::memory_order_relaxed);
  last_failure_component_instance_id_.store(-1,
                                             std::memory_order_relaxed);
  last_failure_frame_.store(-1, std::memory_order_relaxed);
  return true;
}

CompiledAudioRealtimeMetrics CompiledAudioCircuit::RealtimeMetrics() const {
  return {
      last_converged_.load(std::memory_order_relaxed),
      last_failure_component_instance_id_.load(std::memory_order_relaxed),
      last_failure_frame_.load(std::memory_order_relaxed),
      left_speaker_power_watts_.load(std::memory_order_relaxed),
      right_speaker_power_watts_.load(std::memory_order_relaxed),
      left_coil_temperature_c_.load(std::memory_order_relaxed),
      right_coil_temperature_c_.load(std::memory_order_relaxed),
      last_newton_iterations_.load(std::memory_order_relaxed),
      last_nonlinear_substeps_.load(std::memory_order_relaxed),
      last_residual_.load(std::memory_order_relaxed)};
}

void CompiledAudioCircuit::InitializeDcSourceValues() {
  source_values_.assign(source_values_.size(), 0.0);
  for (const TimeSource& source : time_sources_) {
    if (source.slot >= 0 &&
        static_cast<size_t>(source.slot) < source_values_.size()) {
      // SINE: quiescent DC is 0 V (no offset at t=0).
      // PULSE: quiescent value is low_volts (pre-trigger state).
      source_values_[static_cast<size_t>(source.slot)] =
          (source.type == TimeSource::Type::PULSE) ? source.low_volts : 0.0;
    }
  }
  for (const FloatingNoiseSource& source : floating_noise_sources_) {
    if (source.slot >= 0 &&
        static_cast<size_t>(source.slot) < source_values_.size()) {
      source_values_[static_cast<size_t>(source.slot)] = 0.0;
    }
  }
  // Signed PCM silence maps to the midscale unsigned DAC code. Starting the
  // DC operating point with every bit low creates a full-scale transient on
  // the first real frame and can throw the Class-AB junctions out of the
  // three-iteration realtime Newton budget.
  constexpr std::uint16_t kSignedPcmSilenceCode = 0x8000U;
  for (const DacBitSource& source : dac_bit_sources_) {
    if (source.slot >= 0 &&
        static_cast<size_t>(source.slot) < source_values_.size()) {
      const bool high =
          (kSignedPcmSilenceCode &
           (std::uint16_t{1} << static_cast<unsigned int>(source.bit))) != 0;
      source_values_[static_cast<size_t>(source.slot)] =
          high ? source.high_current_amps : 0.0;
    }
  }
}

bool CompiledAudioCircuit::ResetRealtimeState() {
  if (!metrics_.valid) return false;

  InitializeDcSourceValues();

  MnaSolveMetrics reset_metrics;
  if (!solver_.SolveDc(source_values_, &reset_metrics)) return false;

  // Re-synchronize DC terminal voltages from the freshly solved operating point.
  // Without this, left_dc_voltage_ would keep the value from Compile() and
  // create a sustained DC offset step on every RUN press.
  left_dc_voltage_ = solver_.NodeVoltage(left_positive_node_) -
                     solver_.NodeVoltage(left_negative_node_);
  right_dc_voltage_ = solver_.NodeVoltage(right_positive_node_) -
                      solver_.NodeVoltage(right_negative_node_);
  speaker_dc_motional_volts_[0] =
      solver_.NodeVoltage(speaker_motional_nodes_[0]) -
      solver_.NodeVoltage(left_negative_node_);
  speaker_dc_motional_volts_[1] =
      solver_.NodeVoltage(speaker_motional_nodes_[1]) -
      solver_.NodeVoltage(right_negative_node_);
  metrics_.dc_left_volts = left_dc_voltage_;
  metrics_.dc_right_volts = right_dc_voltage_;
  metrics_.dc_node_voltages = solver_.NodeVoltages();

  sample_index_ = 0;
  noise_state_ = 0x51f15eU;
  last_stable_left_ = 0.0f;
  last_stable_right_ = 0.0f;
  left_speaker_power_watts_.store(0.0, std::memory_order_relaxed);
  right_speaker_power_watts_.store(0.0, std::memory_order_relaxed);
  left_coil_temperature_c_.store(25.0, std::memory_order_relaxed);
  right_coil_temperature_c_.store(25.0, std::memory_order_relaxed);
  last_newton_iterations_.store(reset_metrics.iterations,
                                std::memory_order_relaxed);
  last_nonlinear_substeps_.store(1, std::memory_order_relaxed);
  last_residual_.store(reset_metrics.maximum_residual,
                       std::memory_order_relaxed);
  last_converged_.store(true, std::memory_order_relaxed);
  last_failure_component_instance_id_.store(-1,
                                             std::memory_order_relaxed);
  last_failure_frame_.store(-1, std::memory_order_relaxed);
  return true;
}

double CompiledAudioCircuit::PortVoltage(int component_instance_id,
                                         int port_id) const {
  const auto found = port_nodes_.find({component_instance_id, port_id});
  return found != port_nodes_.end() ? solver_.NodeVoltage(found->second) : 0.0;
}

std::array<float, 10> CompiledAudioCircuit::SolveChannelResponse(
    bool right_channel,
    const std::array<float, 10>& frequencies_hz) const {
  std::array<float, 10> result{};
  result.fill(-120.0f);
  const int source_slot = right_channel ? right_ac_source_slot_
                                        : left_ac_source_slot_;
  if (!metrics_.valid || source_slot < 0) return result;
  std::vector<double> frequencies;
  frequencies.reserve(frequencies_hz.size());
  for (float frequency : frequencies_hz) frequencies.push_back(frequency);
  const int positive =
      right_channel ? right_positive_node_ : left_positive_node_;
  const int negative = right_channel ? right_negative_node_
                                     : left_negative_node_;
  const std::vector<MnaAcPoint> response =
      solver_.SolveAc(frequencies, source_slot, positive, negative);
  const double source_scale =
      right_channel ? right_ac_source_scale_ : left_ac_source_scale_;
  for (size_t index = 0; index < response.size() && index < result.size();
       ++index) {
    const double magnitude =
        std::abs(response[index].output) * source_scale;
    result[index] = magnitude > 1.0e-12
                        ? static_cast<float>(20.0 * std::log10(magnitude))
                        : -120.0f;
  }
  return result;
}

std::vector<float> CompiledAudioCircuit::SolveChannelResponse(
    bool right_channel, std::span<const float> frequencies_hz) const {
  std::vector<float> result(frequencies_hz.size(), -120.0f);
  const int source_slot = right_channel ? right_ac_source_slot_
                                        : left_ac_source_slot_;
  if (!metrics_.valid || source_slot < 0) return result;
  std::vector<double> frequencies;
  frequencies.reserve(frequencies_hz.size());
  for (float frequency : frequencies_hz) frequencies.push_back(frequency);
  const int positive =
      right_channel ? right_positive_node_ : left_positive_node_;
  const int negative = right_channel ? right_negative_node_
                                     : left_negative_node_;
  const std::vector<MnaAcPoint> response =
      solver_.SolveAc(frequencies, source_slot, positive, negative);
  const double source_scale =
      right_channel ? right_ac_source_scale_ : left_ac_source_scale_;
  for (size_t index = 0; index < response.size() && index < result.size();
       ++index) {
    const double magnitude =
        std::abs(response[index].output) * source_scale;
    result[index] = magnitude > 1.0e-12
                        ? static_cast<float>(20.0 * std::log10(magnitude))
                        : -120.0f;
  }
  return result;
}

}  // namespace plc::audio
