#include "plc_emulator/audio/mna_solver.h"
#include "plc_emulator/audio/spsc_audio_ring.h"
#include "plc_emulator/audio/compiled_audio_circuit.h"
#include "plc_emulator/audio/converter_models.h"
#include "plc_emulator/components/state_keys.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <numbers>
#include <span>
#include <string>
#include <vector>

namespace {

using plc::audio::MnaAcPoint;
using plc::audio::MnaElement;
using plc::audio::MnaElementType;
using plc::audio::MnaSolveMetrics;
using plc::audio::MnaSolver;

void Require(bool condition, const std::string& message) {
  if (!condition) {
    std::cerr << "FAIL: " << message << '\n';
    std::exit(1);
  }
}

void TestVoltageDivider() {
  MnaSolver solver;
  const std::vector<MnaElement> elements = {
      {MnaElementType::VOLTAGE_SOURCE, 1, 0, 0, 0, 0.0, 0.0, 0, 1},
      {MnaElementType::RESISTOR, 1, 2, 0, 0, 1000.0},
      {MnaElementType::RESISTOR, 2, 0, 0, 0, 1000.0},
  };
  MnaSolveMetrics metrics;
  Require(solver.Compile(3, 0, 48000.0, elements, &metrics), metrics.error);
  const double source[] = {5.0};
  Require(solver.SolveDc(source, &metrics), metrics.error);
  Require(std::abs(solver.NodeVoltage(2) - 2.5) < 0.0025,
          "1k/1k divider must produce 2.5 V within 0.1%.");
}

void TestRcAcResponse() {
  MnaSolver solver;
  constexpr double resistance = 1000.0;
  constexpr double capacitance = 1.0e-6;
  const std::vector<MnaElement> elements = {
      {MnaElementType::VOLTAGE_SOURCE, 1, 0, 0, 0, 0.0, 0.0, 0, 1},
      {MnaElementType::RESISTOR, 1, 2, 0, 0, resistance},
      {MnaElementType::CAPACITOR, 2, 0, 0, 0, capacitance},
  };
  MnaSolveMetrics metrics;
  Require(solver.Compile(3, 0, 48000.0, elements, &metrics), metrics.error);
  const double cutoff =
      1.0 / (2.0 * std::numbers::pi * resistance * capacitance);
  const std::vector<MnaAcPoint> response =
      solver.SolveAc({cutoff}, 0, 2, 0, &metrics);
  Require(response.size() == 1, metrics.error);
  const double magnitude_db = 20.0 * std::log10(std::abs(response[0].output));
  const double phase_deg =
      std::arg(response[0].output) * 180.0 / std::numbers::pi;
  Require(std::abs(magnitude_db + 3.0102999566) < 0.2,
          "RC cutoff magnitude must be -3.01 dB within 0.2 dB.");
  Require(std::abs(phase_deg + 45.0) < 2.0,
          "RC cutoff phase must be -45 degrees within 2 degrees.");
}

void TestRcTransientSettles() {
  MnaSolver solver;
  const std::vector<MnaElement> elements = {
      {MnaElementType::VOLTAGE_SOURCE, 1, 0, 0, 0, 0.0, 0.0, 0, 1},
      {MnaElementType::RESISTOR, 1, 2, 0, 0, 1000.0},
      {MnaElementType::CAPACITOR, 2, 0, 0, 0, 1.0e-6},
  };
  MnaSolveMetrics metrics;
  Require(solver.Compile(3, 0, 48000.0, elements, &metrics), metrics.error);
  const double zero[] = {0.0};
  const double one[] = {1.0};
  Require(solver.SolveDc(zero, &metrics), metrics.error);
  for (int frame = 0; frame < 480; ++frame) {
    Require(solver.ProcessSample(one, &metrics), metrics.error);
  }
  Require(std::abs(solver.NodeVoltage(2) - 1.0) < 0.001,
          "RC transient must settle to the DC source voltage.");
}

void TestVcvsClosedLoop() {
  MnaSolver solver;
  const std::vector<MnaElement> elements = {
      {MnaElementType::VOLTAGE_SOURCE, 1, 0, 0, 0, 0.0, 0.0, 0, 1},
      {MnaElementType::VCVS, 2, 0, 1, 3, 100000.0, 0.0, -1, 2},
      {MnaElementType::RESISTOR, 2, 3, 0, 0, 9000.0},
      {MnaElementType::RESISTOR, 3, 0, 0, 0, 1000.0},
      {MnaElementType::RESISTOR, 2, 0, 0, 0, 10000.0},
  };
  MnaSolveMetrics metrics;
  Require(solver.Compile(4, 0, 48000.0, elements, &metrics), metrics.error);
  const double source[] = {0.1};
  Require(solver.SolveDc(source, &metrics), metrics.error);
  Require(std::abs(solver.NodeVoltage(2) - 0.9999) < 0.002,
          "Feedback wiring, not a gain property, must set closed-loop gain.");
}

void TestCompiledWiringUsesPhysicalLoad() {
  using plc::ComponentType;
  using plc::PlacedComponent;
  using plc::RtlLogicFamily;
  using plc::RtlPinBinding;
  using plc::Wire;
  std::vector<PlacedComponent> components;
  PlacedComponent ground;
  ground.instanceId = 1;
  ground.type = ComponentType::AUDIO_GROUND;
  components.push_back(ground);
  PlacedComponent speaker;
  speaker.instanceId = 2;
  speaker.type = ComponentType::AUDIO_SPEAKER;
  speaker.internalStates[plc::state_keys::kImpedanceOhms] = 8.0f;
  speaker.internalStates[plc::state_keys::kMaximumPowerWatts] = 100.0f;
  components.push_back(speaker);
  PlacedComponent dac;
  dac.instanceId = 3;
  dac.type = ComponentType::RTL_MODULE;
  dac.rtlModuleId = "audio_shell_dac";
  dac.rtlLogicFamily = RtlLogicFamily::CMOS_5V;
  dac.rtlPinBindings.push_back(RtlPinBinding{"dac_l[15]", 100, false});
  components.push_back(dac);
  PlacedComponent resistor;
  resistor.instanceId = 4;
  resistor.type = ComponentType::AUDIO_RESISTOR;
  resistor.internalStates[plc::state_keys::kResistanceOhms] = 20000.0f;
  resistor.internalStates[plc::state_keys::kTolerancePercent] = 0.0f;
  components.push_back(resistor);

  std::vector<Wire> wires;
  const auto add_wire = [&](int from_component, int from_port,
                            int to_component, int to_port) {
    Wire wire{};
    wire.isElectric = true;
    wire.fromComponentId = from_component;
    wire.fromPortId = from_port;
    wire.toComponentId = to_component;
    wire.toPortId = to_port;
    wires.push_back(wire);
  };
  add_wire(3, 100, 4, 0);
  add_wire(4, 1, 2, 0);
  add_wire(2, 1, 1, 0);
  add_wire(2, 2, 1, 0);
  add_wire(2, 3, 1, 0);

  plc::audio::CompiledAudioCircuit circuit;
  Require(circuit.Compile(components, wires, 48000.0),
          circuit.Metrics().error);
  float frames[8] = {1.0f, 0.0f, 1.0f, 0.0f,
                     1.0f, 0.0f, 1.0f, 0.0f};
  circuit.ProcessBlock(frames, 4);
  Require(std::abs(frames[6]) > 1.0e-7f,
          "A wired DAC bit must drive the physical speaker load.");
  Require(std::abs(frames[7]) < 1.0e-7f,
          "A grounded right speaker channel must remain silent.");
  Require(circuit.ResetRealtimeState(),
          "A compiled circuit must restore its DC operating point on restart.");
  const plc::audio::CompiledAudioRealtimeMetrics reset =
      circuit.RealtimeMetrics();
  Require(reset.left_speaker_power_watts == 0.0 &&
              reset.right_speaker_power_watts == 0.0 &&
              reset.left_coil_temperature_c == 25.0 &&
              reset.right_coil_temperature_c == 25.0,
          "Restart must clear speaker power and return both coils to ambient.");
}

void TestSpscRingWrapsWithoutAllocation() {
  plc::audio::SpscAudioRing ring(8);
  const float first[] = {1, 2, 3, 4, 5, 6};
  Require(ring.Write(first, 6) == 6, "SPSC initial write failed.");
  float consumed[4]{};
  Require(ring.Read(consumed, 4) == 4 && consumed[3] == 4.0f,
          "SPSC initial read failed.");
  const float second[] = {7, 8, 9, 10, 11, 12};
  Require(ring.Write(second, 6) == 6, "SPSC wrapped write failed.");
  float wrapped[8]{};
  Require(ring.Read(wrapped, 8) == 8 && wrapped[0] == 5.0f &&
              wrapped[7] == 12.0f,
          "SPSC wrapped ordering failed.");
}

void TestAdcQuantizationAndClipping() {
  plc::audio::AdcModel adc({16, -2.0, 2.0, 48000.0, 1.0e-6,
                            1000.0, 20.0e-12});
  const plc::audio::AdcSample midpoint = adc.Process(0.0, 48000.0);
  Require(midpoint.sampled && !midpoint.clipped &&
              std::abs(static_cast<int>(midpoint.code) - 32768) <= 1,
          "ADC midpoint quantization is incorrect.");
  const plc::audio::AdcSample clipped = adc.Process(3.0, 48000.0);
  Require(clipped.clipped && clipped.code == 65535U,
          "ADC input-range clipping is incorrect.");
}

std::array<double, 16> SolveR2RBitWeights(double changed_branch_scale) {
  constexpr int kSourceFirst = 1;
  constexpr int kLadderFirst = 17;
  std::vector<MnaElement> elements;
  for (int bit = 0; bit < 16; ++bit) {
    elements.push_back({MnaElementType::VOLTAGE_SOURCE,
                        kSourceFirst + bit, 0, 0, 0, 0.0, 0.0, bit,
                        100 + bit});
    const double branch = 20000.0 * (bit == 7 ? changed_branch_scale : 1.0);
    elements.push_back({MnaElementType::RESISTOR, kSourceFirst + bit,
                        kLadderFirst + bit, 0, 0, branch});
    if (bit != 15) {
      elements.push_back({MnaElementType::RESISTOR, kLadderFirst + bit,
                          kLadderFirst + bit + 1, 0, 0, 10000.0});
    }
  }
  elements.push_back(
      {MnaElementType::RESISTOR, kLadderFirst + 15, 0, 0, 0, 20000.0});

  MnaSolver solver;
  MnaSolveMetrics metrics;
  Require(solver.Compile(33, 0, 48000.0, elements, &metrics), metrics.error);
  std::array<double, 16> weights{};
  std::array<double, 16> sources{};
  for (int bit = 0; bit < 16; ++bit) {
    sources.fill(0.0);
    sources[bit] = 1.0;
    Require(solver.SolveDc(sources, &metrics), metrics.error);
    weights[bit] = solver.NodeVoltage(kLadderFirst);
  }
  return weights;
}

void TestSixteenBitR2RMonotonicityAndDnl() {
  const auto ideal_weights = SolveR2RBitWeights(1.0);
  const auto mismatched_weights = SolveR2RBitWeights(1.01);
  const auto measure = [](const std::array<double, 16>& weights) {
    std::array<double, 65536> levels{};
    for (unsigned code = 0; code < levels.size(); ++code) {
      double level = 0.0;
      for (unsigned numeric_bit = 0; numeric_bit < 16; ++numeric_bit) {
        if ((code & (1U << numeric_bit)) != 0U) {
          level += weights[15U - numeric_bit];
        }
      }
      levels[code] = level;
    }
    const double lsb = levels.back() / 65535.0;
    double minimum_step = 1.0e30;
    double maximum_dnl = 0.0;
    for (size_t code = 1; code < levels.size(); ++code) {
      const double step = levels[code] - levels[code - 1];
      minimum_step = std::min(minimum_step, step);
      maximum_dnl = std::max(maximum_dnl, std::abs(step / lsb - 1.0));
    }
    return std::array<double, 2>{minimum_step, maximum_dnl};
  };
  const auto ideal = measure(ideal_weights);
  const auto mismatched = measure(mismatched_weights);
  Require(ideal[0] > 0.0, "The ideal 16-bit R-2R ladder must be monotonic.");
  Require(ideal[1] < 1.0e-3,
          "The ideal 16-bit R-2R ladder has excessive DNL: " +
              std::to_string(ideal[1]) + ", minimum step " +
              std::to_string(ideal[0]));
  Require(mismatched[1] > ideal[1] + 0.1,
          "A 1% branch error must appear as measurable DAC DNL.");
}

void TestRealtimeBlockBudget() {
  std::vector<plc::audio::MnaElement> elements;
  constexpr int kNodes = 96;
  for (int node = 1; node < kNodes; ++node) {
    elements.push_back({plc::audio::MnaElementType::RESISTOR, node - 1,
                        node, 0, 0, 1000.0});
    elements.push_back({plc::audio::MnaElementType::CAPACITOR, node, 0,
                        0, 0, 10.0e-9});
  }
  elements.push_back({plc::audio::MnaElementType::CURRENT_SOURCE, 0, 1,
                      0, 0, 0.0, 0.0, 0});
  plc::audio::MnaSolver solver;
  plc::audio::MnaSolveMetrics metrics;
  Require(solver.Compile(kNodes, 0, 48000.0, elements, &metrics),
          metrics.error);
  std::array<double, 1> sources{0.001};
  std::array<double, 100> timings{};
  for (double& timing : timings) {
    const auto started = std::chrono::steady_clock::now();
    for (size_t frame = 0; frame < 960; ++frame) {
      Require(solver.ProcessSample(sources, &metrics), metrics.error);
    }
    timing = std::chrono::duration<double, std::milli>(
                 std::chrono::steady_clock::now() - started)
                 .count();
  }
  std::sort(timings.begin(), timings.end());
  Require(timings[98] < 10.0,
          "960-frame linear MNA block exceeded the 10 ms budget.");
}

void TestDiodeNonlinearClamp() {
  std::vector<MnaElement> elements = {
      {MnaElementType::CURRENT_SOURCE, 0, 1, 0, 0, 0.001},
      {MnaElementType::DIODE, 1, 0, 0, 0, 1.0e-12, 2.0},
  };
  MnaSolver solver;
  MnaSolveMetrics metrics;
  Require(solver.Compile(2, 0, 48000.0, elements, &metrics), metrics.error);
  Require(solver.SolveDc({}, &metrics), metrics.error);
  Require(solver.NodeVoltage(1) > 0.2 && solver.NodeVoltage(1) < 0.9,
          "Nonlinear DC diode bias was " +
              std::to_string(solver.NodeVoltage(1)) + " V.");
  for (int sample = 0; sample < 1000; ++sample) {
    Require(solver.ProcessSample({}, &metrics), metrics.error);
  }
  const double diode_voltage = solver.NodeVoltage(1);
  Require(diode_voltage > 0.2 && diode_voltage < 0.9,
          "Shockley diode clamp voltage was " +
              std::to_string(diode_voltage) + " V.");
  Require(metrics.iterations <= 3,
          "Nonlinear solve exceeded the three-iteration realtime limit.");
}

void TestNonlinearRealtimeBlockBudget() {
  std::vector<MnaElement> elements;
  constexpr int kNodes = 96;
  for (int node = 1; node < kNodes; ++node) {
    elements.push_back(
        {MnaElementType::RESISTOR, node - 1, node, 0, 0, 1000.0});
    elements.push_back(
        {MnaElementType::CAPACITOR, node, 0, 0, 0, 10.0e-9});
  }
  for (int node : {8, 16, 24, 32, 40, 56, 72, 88}) {
    elements.push_back(
        {MnaElementType::DIODE, node, 0, 0, 0, 1.0e-12, 2.0});
  }
  elements.push_back(
      {MnaElementType::CURRENT_SOURCE, 0, 1, 0, 0, 0.0, 0.0, 0});
  MnaSolver solver;
  MnaSolveMetrics metrics;
  Require(solver.Compile(kNodes, 0, 48000.0, elements, &metrics),
          metrics.error);
  std::array<double, 1> sources{0.001};
  std::array<double, 100> timings{};
  std::uint64_t sample_index = 0;
  int maximum_iterations = 0;
  for (double& elapsed_ms : timings) {
    const auto started = std::chrono::steady_clock::now();
    for (size_t frame = 0; frame < 960; ++frame) {
      sources[0] = 0.001 + 0.00005 * std::sin(
          2.0 * std::numbers::pi * 997.0 *
          static_cast<double>(sample_index++) / 48000.0);
      Require(solver.ProcessSample(sources, &metrics), metrics.error);
      maximum_iterations = std::max(maximum_iterations, metrics.iterations);
    }
    elapsed_ms = std::chrono::duration<double, std::milli>(
                     std::chrono::steady_clock::now() - started)
                     .count();
  }
  std::sort(timings.begin(), timings.end());
  Require(timings[98] < 10.0,
          "960-frame nonlinear MNA block took " +
              std::to_string(timings[98]) + " ms at p99.");
  Require(maximum_iterations <= 3,
          "Changing nonlinear input exceeded the strict three-iteration "
          "Newton limit.");
}

void TestUnrelatedDacNodeMovementDoesNotRefactorNonlinearMatrix() {
  // Node 1 contains a steady nonlinear junction. Node 2 represents a DAC
  // node whose voltage changes every audio sample but is electrically
  // unrelated to that junction. Such linear movement must not trigger extra
  // Newton matrix factorizations.
  const std::vector<MnaElement> elements = {
      {MnaElementType::CURRENT_SOURCE, 0, 1, 0, 0, 0.0, 0.0, 0},
      {MnaElementType::DIODE, 1, 0, 0, 0, 1.0e-12, 2.0, -1, 1},
      {MnaElementType::VOLTAGE_SOURCE, 2, 0, 0, 0, 0.0, 0.0, 1, 2},
  };
  MnaSolver solver;
  MnaSolveMetrics metrics;
  Require(solver.Compile(3, 0, 48000.0, elements, &metrics), metrics.error);
  std::array<double, 2> sources{0.001, 0.0};
  Require(solver.SolveDc(sources, &metrics), metrics.error);
  // Prime the cached junction linearization.
  Require(solver.ProcessSample(sources, &metrics), metrics.error);
  for (int sample = 0; sample < 64; ++sample) {
    sources[1] = (sample & 1) != 0 ? 1.0 : -1.0;
    Require(solver.ProcessSample(sources, &metrics), metrics.error);
    Require(metrics.iterations == 1,
            "Unrelated DAC node movement refactorized the nonlinear MNA matrix.");
  }
}

void TestBjtEbersMollBiasAndEarlyEffect() {
  const std::vector<MnaElement> elements = {
      {MnaElementType::VOLTAGE_SOURCE, 1, 0, 0, 0, 0.0, 0.0, 0, 1},
      {MnaElementType::VOLTAGE_SOURCE, 2, 0, 0, 0, 0.0, 0.0, 1, 2},
      {MnaElementType::RESISTOR, 1, 3, 0, 0, 1000.0},
      {MnaElementType::BJT_NPN, 3, 0, 2, 0, 1.0e-14, 100.0, -1, 3,
       100.0},
  };
  MnaSolver solver;
  MnaSolveMetrics metrics;
  Require(solver.Compile(4, 0, 48000.0, elements, &metrics), metrics.error);
  const std::array<double, 2> low_bias{5.0, 0.58};
  Require(solver.SolveDc(low_bias, &metrics), metrics.error);
  const double low_current = (5.0 - solver.NodeVoltage(3)) / 1000.0;
  const std::array<double, 2> high_bias{5.0, 0.68};
  Require(solver.SolveDc(high_bias, &metrics), metrics.error);
  const double high_current = (5.0 - solver.NodeVoltage(3)) / 1000.0;
  Require(high_current > low_current * 5.0 && high_current > 1.0e-4,
          "BJT collector current must follow the exponential B-E bias.");

  const std::array<double, 2> higher_supply{10.0, 0.62};
  Require(solver.SolveDc(higher_supply, &metrics), metrics.error);
  const double early_high = (10.0 - solver.NodeVoltage(3)) / 1000.0;
  const std::array<double, 2> lower_supply{5.0, 0.62};
  Require(solver.SolveDc(lower_supply, &metrics), metrics.error);
  const double early_low = (5.0 - solver.NodeVoltage(3)) / 1000.0;
  Require(early_high > early_low,
          "Finite Early voltage must increase collector current with Vce.");
}

void TestSpeakerImpedanceResonance() {
  constexpr double re = 6.4;
  constexpr double le = 0.5e-3;
  constexpr double bl = 7.0;
  constexpr double mms = 15.0e-3;
  constexpr double cms = 0.5e-3;
  constexpr double rms = 1.5;
  const std::vector<MnaElement> elements = {
      {MnaElementType::CURRENT_SOURCE, 0, 1, 0, 0, 0.0, 0.0, 0},
      {MnaElementType::RESISTOR, 1, 2, 0, 0, re},
      {MnaElementType::INDUCTOR, 2, 3, 0, 0, le},
      {MnaElementType::RESISTOR, 3, 0, 0, 0, bl * bl / rms},
      {MnaElementType::INDUCTOR, 3, 0, 0, 0, bl * bl * cms},
      {MnaElementType::CAPACITOR, 3, 0, 0, 0, mms / (bl * bl)},
  };
  MnaSolver solver;
  MnaSolveMetrics metrics;
  Require(solver.Compile(4, 0, 48000.0, elements, &metrics), metrics.error);
  const double resonance_hz =
      1.0 / (2.0 * std::numbers::pi * std::sqrt(mms * cms));
  const auto response =
      solver.SolveAc({20.0, resonance_hz, 1000.0}, 0, 1, 0, &metrics);
  Require(response.size() == 3, metrics.error);
  const double low = std::abs(response[0].output);
  const double resonance = std::abs(response[1].output);
  const double high = std::abs(response[2].output);
  Require(resonance > low * 2.0 && resonance > high * 1.5,
          "The electromechanical speaker model must expose its impedance resonance.");
}

}  // namespace

int main() {
  TestVoltageDivider();
  TestRcAcResponse();
  TestRcTransientSettles();
  TestVcvsClosedLoop();
  TestCompiledWiringUsesPhysicalLoad();
  TestSpscRingWrapsWithoutAllocation();
  TestAdcQuantizationAndClipping();
  TestSixteenBitR2RMonotonicityAndDnl();
  TestRealtimeBlockBudget();
  TestDiodeNonlinearClamp();
  TestNonlinearRealtimeBlockBudget();
  TestUnrelatedDacNodeMovementDoesNotRefactorNonlinearMatrix();
  TestBjtEbersMollBiasAndEarlyEffect();
  TestSpeakerImpedanceResonance();
  std::cout << "audio_engine_tests: all tests passed\n";
  return 0;
}
