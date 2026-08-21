#include "plc_emulator/audio/compiled_audio_circuit.h"

#include <nlohmann/json.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <numbers>
#include <sstream>
#include <string>
#include <vector>

#if defined(_WIN32)
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <avrt.h>
#endif

namespace {

using nlohmann::json;
using plc::ComponentType;
using plc::PlacedComponent;
using plc::RtlLogicFamily;
using plc::RtlPinBinding;
using plc::Wire;
using plc::audio::CompiledAudioCircuit;

void Require(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

json ReadJson(const std::filesystem::path& path) {
  std::ifstream input(path, std::ios::binary);
  Require(input.is_open(), "Could not open " + path.string());
  json value;
  input >> value;
  return value;
}

ComponentType ParseComponentType(const std::string& name) {
  static const std::map<std::string, ComponentType> types = {
      {"RTL_MODULE", ComponentType::RTL_MODULE},
      {"AUDIO_SOURCE", ComponentType::AUDIO_SOURCE},
      {"AUDIO_RESISTOR", ComponentType::AUDIO_RESISTOR},
      {"AUDIO_POTENTIOMETER", ComponentType::AUDIO_POTENTIOMETER},
      {"AUDIO_CAPACITOR", ComponentType::AUDIO_CAPACITOR},
      {"AUDIO_INDUCTOR", ComponentType::AUDIO_INDUCTOR},
      {"AUDIO_OP_AMP", ComponentType::AUDIO_OP_AMP},
      {"AUDIO_SPEAKER", ComponentType::AUDIO_SPEAKER},
      {"AUDIO_GROUND", ComponentType::AUDIO_GROUND},
      {"AUDIO_DC_SOURCE", ComponentType::AUDIO_DC_SOURCE},
      {"AUDIO_AC_SOURCE", ComponentType::AUDIO_AC_SOURCE},
      {"AUDIO_PULSE_SOURCE", ComponentType::AUDIO_PULSE_SOURCE},
      {"AUDIO_DIODE", ComponentType::AUDIO_DIODE},
      {"AUDIO_BJT_NPN", ComponentType::AUDIO_BJT_NPN},
      {"AUDIO_BJT_PNP", ComponentType::AUDIO_BJT_PNP},
  };
  const auto found = types.find(name);
  Require(found != types.end(), "Unsupported Profile07 component: " + name);
  return found->second;
}

std::vector<PlacedComponent> LoadComponents(
    const json& layout, const json& rtl_library) {
  std::map<std::string, std::vector<RtlPinBinding>> module_pins;
  for (const json& entry : rtl_library.at("entries")) {
    std::vector<RtlPinBinding> pins;
    for (const json& port : entry.value("ports", json::array())) {
      pins.push_back({port.value("pin_name", std::string()),
                      port.value("port_id", -1),
                      port.value("is_input", true)});
    }
    module_pins.emplace(entry.value("module_id", std::string()),
                        std::move(pins));
  }

  std::vector<PlacedComponent> components;
  for (const json& item : layout.at("components")) {
    PlacedComponent component;
    component.instanceId = item.value("instance_id", 0);
    component.type = ParseComponentType(item.value("type", std::string()));
    component.customLabel = item.value("custom_label", std::string());
    if (const auto parameters = item.find("audio_parameters");
        parameters != item.end() && parameters->is_object()) {
      for (auto parameter = parameters->begin(); parameter != parameters->end();
           ++parameter) {
        if (parameter.value().is_number()) {
          component.internalStates[parameter.key()] =
              parameter.value().get<float>();
        }
      }
    }
    if (const auto rtl = item.find("rtl");
        rtl != item.end() && rtl->is_object()) {
      component.rtlModuleId = rtl->value("module_id", std::string());
      component.rtlSourceMode = rtl->value("source_mode", std::string());
      component.rtlClockPinName =
          rtl->value("clock_pin_name", std::string());
      component.rtlResetPinName =
          rtl->value("reset_pin_name", std::string());
      component.rtlClockFrequencyHz =
          rtl->value("clock_frequency_hz", 1.0f);
      component.rtlResetActiveLow =
          rtl->value("reset_active_low", false);
      const std::string family =
          rtl->value("logic_family", std::string("CMOS_5V"));
      component.rtlLogicFamily =
          family == "TTL_5V" ? RtlLogicFamily::TTL_5V
          : family == "INDUSTRIAL_24V"
              ? RtlLogicFamily::INDUSTRIAL_24V
              : RtlLogicFamily::CMOS_5V;
      const auto pins = module_pins.find(component.rtlModuleId);
      Require(pins != module_pins.end(),
              "Profile07 RTL module has no saved pin list.");
      component.rtlPinBindings = pins->second;
    }
    components.push_back(std::move(component));
  }
  return components;
}

std::vector<Wire> LoadWires(const json& layout) {
  std::vector<Wire> wires;
  for (const json& item : layout.at("wires")) {
    Wire wire;
    wire.id = item.value("id", 0);
    wire.fromComponentId = item.value("from_component_id", 0);
    wire.fromPortId = item.value("from_port_id", 0);
    wire.toComponentId = item.value("to_component_id", 0);
    wire.toPortId = item.value("to_port_id", 0);
    wire.isElectric = item.value("is_electric", true);
    wire.thickness = item.value("thickness", 2.0f);
    wires.push_back(std::move(wire));
  }
  return wires;
}

double Rms(const std::vector<float>& samples) {
  double sum = 0.0;
  for (float sample : samples) sum += static_cast<double>(sample) * sample;
  return std::sqrt(sum / static_cast<double>(samples.size()));
}

struct ExampleMeasurement {
  double left_rms = 0.0;
  double right_rms = 0.0;
  double left_gain = 0.0;
  double left_thdn = 0.0;
  double left_dc = 0.0;
  double right_dc = 0.0;
  double left_power_watts = 0.0;
  double right_power_watts = 0.0;
  std::vector<float> left_response;
  std::vector<float> right_response;
};

ExampleMeasurement MeasureExample(const std::string& directory,
                                  double amplitude) {
  const std::filesystem::path root =
      std::filesystem::path(AUDIO_ENGINE_SOURCE_DIR) / "examples" /
      directory;
  const json layout = ReadJson(root / "layout.json");
  const json rtl_library = ReadJson(root / "rtl_library.json");
  const auto components = LoadComponents(layout, rtl_library);
  const auto wires = LoadWires(layout);
  CompiledAudioCircuit circuit;
  Require(circuit.Compile(components, wires, 48000.0),
          directory + " compile failed: " + circuit.Metrics().error);
  Require(circuit.Metrics().left_signal_route &&
              circuit.Metrics().right_signal_route,
          directory + " lost a stereo signal route");

  const std::vector<float> frequencies{100.0f, 1000.0f, 10000.0f};
  ExampleMeasurement result;
  result.left_response = circuit.SolveChannelResponse(false, frequencies);
  result.right_response = circuit.SolveChannelResponse(true, frequencies);
  result.left_dc = circuit.Metrics().dc_left_volts;
  result.right_dc = circuit.Metrics().dc_right_volts;

  constexpr size_t kFrames = 480;
  constexpr int kBlocks = 160;
  constexpr int kAnalyzeBlocks = 60;
  constexpr double kFrequency = 997.0;
  std::vector<float> block(kFrames * 2U);
  std::vector<double> left;
  std::vector<double> right;
  left.reserve(kFrames * kAnalyzeBlocks);
  right.reserve(kFrames * kAnalyzeBlocks);
  std::uint64_t sample_index = 0;
  for (int block_index = 0; block_index < kBlocks; ++block_index) {
    for (size_t frame = 0; frame < kFrames; ++frame) {
      const double phase = 2.0 * std::numbers::pi * kFrequency *
          static_cast<double>(sample_index++) / 48000.0;
      const float value = static_cast<float>(amplitude * std::sin(phase));
      block[frame * 2U] = value;
      block[frame * 2U + 1U] = value;
    }
    Require(circuit.ProcessBlock(block.data(), kFrames),
            directory + " failed to process its reference tone");
    if (block_index >= kBlocks - kAnalyzeBlocks) {
      for (size_t frame = 0; frame < kFrames; ++frame) {
        left.push_back(block[frame * 2U]);
        right.push_back(block[frame * 2U + 1U]);
      }
    }
  }
  const auto realtime = circuit.RealtimeMetrics();
  result.left_power_watts = realtime.left_speaker_power_watts;
  result.right_power_watts = realtime.right_speaker_power_watts;
  auto channel_metrics = [&](const std::vector<double>& samples,
                             double* rms, double* gain, double* thdn) {
    double energy = 0.0;
    double mean = 0.0;
    double sin_sum = 0.0;
    double cos_sum = 0.0;
    const std::uint64_t first_sample =
        static_cast<std::uint64_t>(kBlocks - kAnalyzeBlocks) * kFrames;
    for (size_t index = 0; index < samples.size(); ++index) {
      const double value = samples[index];
      const double phase = 2.0 * std::numbers::pi * kFrequency *
          static_cast<double>(first_sample + index) / 48000.0;
      energy += value * value;
      mean += value;
      sin_sum += value * std::sin(phase);
      cos_sum += value * std::cos(phase);
    }
    const double count = static_cast<double>(samples.size());
    mean /= count;
    const double sin_coefficient = 2.0 * sin_sum / count;
    const double cos_coefficient = 2.0 * cos_sum / count;
    const double fundamental_rms =
        std::hypot(sin_coefficient, cos_coefficient) / std::sqrt(2.0);
    double residual_energy = 0.0;
    for (size_t index = 0; index < samples.size(); ++index) {
      const double phase = 2.0 * std::numbers::pi * kFrequency *
          static_cast<double>(first_sample + index) / 48000.0;
      const double predicted = mean + sin_coefficient * std::sin(phase) +
          cos_coefficient * std::cos(phase);
      const double residual = samples[index] - predicted;
      residual_energy += residual * residual;
    }
    *rms = std::sqrt(energy / count);
    *gain = fundamental_rms / (amplitude / std::sqrt(2.0));
    *thdn = std::sqrt(residual_energy / count) /
        std::max(fundamental_rms, 1.0e-12);
  };
  double unused_gain = 0.0;
  double unused_thdn = 0.0;
  channel_metrics(left, &result.left_rms, &result.left_gain,
                  &result.left_thdn);
  channel_metrics(right, &result.right_rms, &unused_gain, &unused_thdn);
  return result;
}

void TestBundledExampleIntent() {
  const ExampleMeasurement normal =
      MeasureExample("complete_stereo_volume", 0.10);
  const ExampleMeasurement bit_fault =
      MeasureExample("fault_01_bit_weight_dac", 0.10);
  const ExampleMeasurement eq_fault =
      MeasureExample("fault_02_stereo_filter_chaos", 0.10);
  const ExampleMeasurement power_fault =
      MeasureExample("fault_03_brownout_clipper", 0.80);
  const ExampleMeasurement dc_fault =
      MeasureExample("fault_04_thermal_dc_hazard", 0.10);
  const ExampleMeasurement bandwidth_fault =
      MeasureExample("fault_05_crossover_bandwidth", 0.10);

  std::cout << "example metrics: normal gain/thdn/stereo="
            << normal.left_gain << '/' << normal.left_thdn << '/'
            << normal.left_rms / std::max(normal.right_rms, 1.0e-12)
            << "/power=" << normal.left_power_watts << '/'
            << normal.right_power_watts
            << "/response=" << normal.left_response[0] << '/'
            << normal.left_response[1] << '/' << normal.left_response[2]
            << ", bit=" << bit_fault.left_gain << '/'
            << bit_fault.left_thdn
            << ", eq 10k L/R=" << eq_fault.left_response[2] << '/'
            << eq_fault.right_response[2]
            << ", power thdn=" << power_fault.left_thdn
            << ", dc=" << dc_fault.left_dc << '/' << dc_fault.right_dc
            << ", bandwidth thdn/10k L/R=" << bandwidth_fault.left_thdn
            << '/' << bandwidth_fault.left_response[2] << '/'
            << bandwidth_fault.right_response[2] << '\n';

  Require(normal.left_gain > 0.05 && normal.left_gain < 3.0,
          "Normal example has an unreasonable reference gain");
  Require(normal.left_thdn < 0.10,
          "Normal example is audibly nonlinear");
  Require(normal.left_power_watts <= 100.0 &&
              normal.right_power_watts <= 100.0,
          "Normal example exceeds the 100 W speaker rating");
  const auto normal_response_bounds = std::minmax_element(
      normal.left_response.begin(), normal.left_response.end());
  Require(*normal_response_bounds.second - *normal_response_bounds.first < 6.0,
          "Normal example has a visibly muffled 100 Hz-10 kHz response");
  Require(std::abs(20.0 * std::log10(
              normal.left_rms / std::max(normal.right_rms, 1.0e-12))) < 1.0,
          "Normal example is not stereo symmetric");
  Require(bit_fault.left_thdn > normal.left_thdn + 0.015,
          "Fault01 does not create bit-weight distortion");
  Require(std::abs(20.0 * std::log10(
              bit_fault.left_gain / std::max(normal.left_gain, 1.0e-12))) <
              6.0,
          "Fault01 changed mostly volume instead of code linearity");
  Require(eq_fault.right_response[2] - eq_fault.left_response[2] > 5.0,
          "Fault02 does not create the named stereo EQ split");
  Require(power_fault.left_thdn > normal.left_thdn + 0.01,
          "Fault03 does not clip under an overdriven input");
  Require(std::max(std::abs(dc_fault.left_dc),
                   std::abs(dc_fault.right_dc)) > 0.02,
          "Fault04 does not put a hazardous DC offset on the load");
  Require(bandwidth_fault.left_thdn > normal.left_thdn + 0.01 ||
              bandwidth_fault.left_response[2] <
                  normal.left_response[2] - 3.0 ||
              bandwidth_fault.right_response[2] <
                  normal.right_response[2] - 3.0,
          "Fault05 does not create crossover/bandwidth distortion");
}

void TestActualProfile07AtRealtimeRate() {
  const std::filesystem::path profile =
      std::filesystem::path(AUDIO_ENGINE_SOURCE_DIR) / "examples" /
      "profile_07_ath_m50x";
  const json layout = ReadJson(profile / "layout.json");
  const json rtl_library = ReadJson(profile / "rtl_library.json");
  const auto components = LoadComponents(layout, rtl_library);
  const auto wires = LoadWires(layout);

  CompiledAudioCircuit circuit;
  Require(circuit.Compile(components, wires, 48000.0),
          "Profile07 compile failed: " + circuit.Metrics().error);
  Require(circuit.Metrics().dac_output_bits == 32,
          "Profile07 did not compile all stereo DAC bits.");
  Require(circuit.Metrics().left_signal_route &&
              circuit.Metrics().right_signal_route,
          "Profile07 lost a DAC-to-speaker signal route.");
  const std::vector<float> response_frequency{1000.0f};
  const std::vector<float> left_response =
      circuit.SolveChannelResponse(false, response_frequency);
  const double initial_dac_voltage = circuit.PortVoltage(74, 0);

  constexpr size_t kFrames = 480;
  std::vector<float> block(kFrames * 2U);
  std::vector<double> timings;
  constexpr int kMeasuredBlocks = 500;
  timings.reserve(kMeasuredBlocks);
  std::uint64_t sample_index = 0;
  std::uint32_t stimulus_state = 0x5a17c9e3U;
  double maximum_rms = 0.0;
  double steady_state_rms_sum = 0.0;
  int steady_state_rms_blocks = 0;
  std::array<double, 9> stage_peaks{};
  std::array<double, 9> steady_stage_peaks{};
  for (int block_index = 0; block_index < kMeasuredBlocks; ++block_index) {
    for (size_t frame = 0; frame < kFrames; ++frame) {
      const double time = static_cast<double>(sample_index++) / 48000.0;
      stimulus_state = stimulus_state * 1664525U + 1013904223U;
      const double noise =
          static_cast<double>((stimulus_state >> 8U) & 0x00ffffffU) /
              static_cast<double>(0x00ffffffU) *
              2.0 -
          1.0;
      const float value = static_cast<float>(
          0.08 * (std::sin(2.0 * std::numbers::pi * 997.0 * time) +
                  0.50 * std::sin(2.0 * std::numbers::pi * 3719.0 * time)) +
          0.06 * noise);
      block[frame * 2U] = value;
      block[frame * 2U + 1U] = value;
    }
    const auto started = std::chrono::steady_clock::now();
    const bool processed = circuit.ProcessBlock(block.data(), kFrames);
    const auto realtime = circuit.RealtimeMetrics();
    Require(processed,
            "Profile07 MNA processing did not converge at component " +
                std::to_string(realtime.failure_component_instance_id) +
                " frame " + std::to_string(realtime.failure_frame) +
                ", residual " + std::to_string(realtime.residual) +
                ", iterations " +
                std::to_string(realtime.newton_iterations));
    for (float sample : block) {
      Require(std::isfinite(sample),
              "Profile07 produced a non-finite PCM sample.");
    }
    timings.push_back(std::chrono::duration<double, std::milli>(
                          std::chrono::steady_clock::now() - started)
                          .count());
    const double block_rms = Rms(block);
    maximum_rms = std::max(maximum_rms, block_rms);
    if (block_index >= kMeasuredBlocks - 100) {
      steady_state_rms_sum += block_rms;
      ++steady_state_rms_blocks;
    }
    const std::array<std::pair<int, int>, 9> stage_ports{{
        {74, 0}, {58, 4}, {3, 1}, {15, 4}, {52, 0},
        {52, 2}, {53, 0}, {53, 2}, {7, 0}}};
    for (size_t stage = 0; stage < stage_ports.size(); ++stage) {
      const double voltage = std::abs(circuit.PortVoltage(
          stage_ports[stage].first, stage_ports[stage].second));
      stage_peaks[stage] = std::max(stage_peaks[stage], voltage);
      if (block_index >= kMeasuredBlocks - 100) {
        steady_stage_peaks[stage] =
            std::max(steady_stage_peaks[stage], voltage);
      }
    }
  }
  std::sort(timings.begin(), timings.end());
  const size_t p99_index = timings.size() * 99U / 100U;
  const size_t p95_index = timings.size() * 95U / 100U;
  Require(maximum_rms > 1.0e-6,
          "Profile07 produced physical zero output for a 1 kHz input: RMS " +
              std::to_string(maximum_rms) + " (x1e12 " +
              std::to_string(maximum_rms * 1.0e12) + ", AC " +
              std::to_string(left_response.empty() ? -999.0f
                                                   : left_response.front()) +
              " dB, stages " + std::to_string(stage_peaks[0]) + "/" +
              std::to_string(stage_peaks[1]) + "/" +
              std::to_string(stage_peaks[2]) + "/" +
              std::to_string(stage_peaks[3]) + "/" +
              std::to_string(stage_peaks[4]) + "/" +
              std::to_string(stage_peaks[5]) + "/" +
              std::to_string(stage_peaks[6]) + "/" +
              std::to_string(stage_peaks[7]) + "/" +
              std::to_string(stage_peaks[8]) + ", initial DAC " +
              std::to_string(initial_dac_voltage) + ", orders " +
              std::to_string(circuit.Metrics().matrix_order) + "/" +
              std::to_string(circuit.Metrics().reduced_matrix_order) + ")");
  const double steady_state_rms =
      steady_state_rms_blocks > 0
          ? steady_state_rms_sum /
                static_cast<double>(steady_state_rms_blocks)
          : 0.0;
  std::ostringstream steady_diagnostics;
  steady_diagnostics << std::scientific << std::setprecision(6)
                     << "Profile07 decayed to physical zero after startup: "
                     << "steady RMS " << steady_state_rms
                     << ", maximum RMS " << maximum_rms << ", stages ";
  for (size_t stage = 0; stage < steady_stage_peaks.size(); ++stage) {
    if (stage != 0) steady_diagnostics << '/';
    steady_diagnostics << steady_stage_peaks[stage];
  }
  Require(steady_state_rms > 1.0e-6, steady_diagnostics.str());
  Require(timings[p95_index] < 20.0,
          "Profile07 480-frame MNA p95 exceeded 20 ms: " +
              std::to_string(timings[p95_index]) + " ms, Newton " +
              std::to_string(circuit.RealtimeMetrics().newton_iterations) +
              ", substeps " +
              std::to_string(circuit.RealtimeMetrics().nonlinear_substeps));
  Require(timings[p99_index] < 30.0,
          "Profile07 480-frame circuit pipeline p99 exceeded 30 ms: " +
              std::to_string(timings[p99_index]) + " ms (full " +
              std::to_string(circuit.Metrics().matrix_order) + ", p50 " +
              std::to_string(timings[timings.size() / 2U]) + ", p95 " +
              std::to_string(timings[timings.size() * 95U / 100U]) +
              ", full " +
              std::to_string(circuit.Metrics().matrix_order) + ", reduced " +
              std::to_string(circuit.Metrics().reduced_matrix_order) +
              ", eliminated " +
              std::to_string(circuit.Metrics().eliminated_unknowns) +
              ", Newton " +
              std::to_string(circuit.RealtimeMetrics().newton_iterations) +
              ", substeps " +
              std::to_string(circuit.RealtimeMetrics().nonlinear_substeps) +
              ")");

  // A grossly overdriven headphone profile may legitimately hit the
  // nonlinear safety fade, but it must never leak NaN/Inf into WASAPI.
  CompiledAudioCircuit overload_circuit;
  Require(overload_circuit.Compile(components, wires, 48000.0),
          "Could not compile the Profile07 overload fixture.");
  std::vector<float> overload(kFrames * 2U);
  for (size_t frame = 0; frame < kFrames; ++frame) {
    const float value = static_cast<float>(
        0.25 * std::sin(2.0 * std::numbers::pi * 1000.0 *
                        static_cast<double>(frame) / 48000.0));
    overload[frame * 2U] = value;
    overload[frame * 2U + 1U] = value;
  }
  (void)overload_circuit.ProcessBlock(overload.data(), kFrames);
  Require(std::all_of(overload.begin(), overload.end(),
                      [](float sample) { return std::isfinite(sample); }),
          "Profile07 overload leaked a non-finite sample instead of the "
          "safe MNA failure fade.");
}

}  // namespace

int main() {
#if defined(_WIN32)
  DWORD task_index = 0;
  HANDLE mmcss = AvSetMmThreadCharacteristicsW(L"Pro Audio", &task_index);
  if (mmcss) AvSetMmThreadPriority(mmcss, AVRT_PRIORITY_CRITICAL);
#endif
  TestActualProfile07AtRealtimeRate();
  TestBundledExampleIntent();
#if defined(_WIN32)
  if (mmcss) AvRevertMmThreadCharacteristics(mmcss);
#endif
  std::cout << "profile07_audio_integration_tests: all tests passed\n";
  return 0;
}
