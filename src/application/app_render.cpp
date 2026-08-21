// app_render.cpp
//
// Application rendering and UI.

#include "plc_emulator/core/application.h"
#include "plc_emulator/audio/audio_circuit_runtime.h"
#include "plc_emulator/audio/audio_stream_bridge.h"

#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "imgui_internal.h"
#include "plc_emulator/lang/lang_manager.h"

#include <glad/glad.h>

#define GLFW_INCLUDE_NONE
#include <GLFW/glfw3.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <windows.h>
#include <mmsystem.h>
#include <shellapi.h>
#define GLFW_EXPOSE_NATIVE_WIN32
#define GLFW_NATIVE_INCLUDE_NONE
#include <GLFW/glfw3native.h>
#include <commdlg.h>
#endif

#include <chrono>
#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <iostream>
#include <limits>
#include <string>
#include <thread>

namespace plc {
namespace {

constexpr ImU32 kAnalyzerLeftColor = IM_COL32(45, 190, 255, 255);
constexpr ImU32 kAnalyzerRightColor = IM_COL32(255, 105, 155, 255);
constexpr ImU32 kAnalyzerGridColor = IM_COL32(48, 60, 66, 255);
constexpr ImU32 kAnalyzerLabelColor = IM_COL32(145, 158, 166, 255);

struct LiveAnalyzerMetrics {
  std::array<float, 2> dc{};
  std::array<float, 2> peak_to_peak{};
  std::array<float, 2> crest_db{};
  std::array<float, 2> headroom_db{};
  std::array<float, 2> dominant_hz{};
  std::array<float, 2> spectral_centroid_hz{};
  float stereo_correlation = 0.0f;
  float stereo_width_db = -120.0f;
  float balance_db = 0.0f;
};

bool AnalyzerPointInRect(ImVec2 point, ImVec2 min, ImVec2 max) {
  return point.x >= min.x && point.x <= max.x && point.y >= min.y &&
         point.y <= max.y;
}

float FiniteAnalyzerValue(float value, float fallback = 0.0f) {
  return std::isfinite(value) ? value : fallback;
}

float NiceAnalyzerAmplitude(float required) {
  if (!std::isfinite(required) || required <= 0.0f) return 1.0f;
  return std::clamp(
      std::pow(2.0f, std::ceil(std::log2(required))), 0.0009765625f, 64.0f);
}

LiveAnalyzerMetrics ComputeLiveAnalyzerMetrics(
    const AudioSignalAnalysis& live) {
  LiveAnalyzerMetrics metrics;
  if (!live.active) return metrics;

  std::array<float, 2> minimum = {
      (std::numeric_limits<float>::max)(),
      (std::numeric_limits<float>::max)()};
  std::array<float, 2> maximum = {
      (std::numeric_limits<float>::lowest)(),
      (std::numeric_limits<float>::lowest)()};
  for (size_t channel = 0; channel < 2; ++channel) {
    double sum = 0.0;
    size_t valid_samples = 0;
    for (float sample : live.waveform[channel]) {
      if (!std::isfinite(sample)) continue;
      sum += sample;
      ++valid_samples;
      minimum[channel] = std::min(minimum[channel], sample);
      maximum[channel] = std::max(maximum[channel], sample);
    }
    if (valid_samples > 0) {
      metrics.dc[channel] =
          static_cast<float>(sum / static_cast<double>(valid_samples));
      metrics.peak_to_peak[channel] = maximum[channel] - minimum[channel];
    }
    metrics.crest_db[channel] =
        std::max(0.0f, live.peak_dbfs[channel] - live.rms_dbfs[channel]);
    metrics.headroom_db[channel] = std::max(0.0f, -live.peak_dbfs[channel]);

    float strongest_db = -121.0f;
    double weighted_frequency = 0.0;
    double spectral_weight = 0.0;
    for (size_t bin = 0; bin < kAudioAnalyzerSpectrumBins; ++bin) {
      const float db = live.spectrum_dbfs[channel][bin];
      if (!std::isfinite(db)) continue;
      if (db > strongest_db) {
        strongest_db = db;
        metrics.dominant_hz[channel] = live.spectrum_frequencies_hz[bin];
      }
      const double power = std::pow(10.0, static_cast<double>(db) / 10.0);
      weighted_frequency +=
          static_cast<double>(live.spectrum_frequencies_hz[bin]) * power;
      spectral_weight += power;
    }
    if (spectral_weight > 0.0) {
      metrics.spectral_centroid_hz[channel] =
          static_cast<float>(weighted_frequency / spectral_weight);
    }
  }

  double covariance = 0.0;
  double left_energy = 0.0;
  double right_energy = 0.0;
  double mid_energy = 0.0;
  double side_energy = 0.0;
  for (size_t point = 0; point < kAudioAnalyzerWaveformPoints; ++point) {
    const double left = live.waveform[0][point] - metrics.dc[0];
    const double right = live.waveform[1][point] - metrics.dc[1];
    if (!std::isfinite(left) || !std::isfinite(right)) continue;
    covariance += left * right;
    left_energy += left * left;
    right_energy += right * right;
    const double mid = (left + right) * 0.5;
    const double side = (left - right) * 0.5;
    mid_energy += mid * mid;
    side_energy += side * side;
  }
  const double correlation_denominator =
      std::sqrt(left_energy * right_energy);
  if (correlation_denominator > 0.000000001) {
    metrics.stereo_correlation = static_cast<float>(
        std::clamp(covariance / correlation_denominator, -1.0, 1.0));
  }
  if (mid_energy > 0.000000001 && side_energy > 0.000000001) {
    metrics.stereo_width_db = static_cast<float>(
        10.0 * std::log10(side_energy / mid_energy));
  }
  metrics.balance_db = live.rms_dbfs[0] - live.rms_dbfs[1];
  return metrics;
}

void FormatAnalyzerFrequency(char* buffer, size_t size, float frequency_hz) {
  if (frequency_hz >= 1000.0f) {
    std::snprintf(buffer, size, "%.2f kHz", frequency_hz / 1000.0f);
  } else {
    std::snprintf(buffer, size, "%.1f Hz", frequency_hz);
  }
}

void DrawAnalyzerPlotBackground(ImDrawList* draw, ImVec2 min, ImVec2 max) {
  draw->AddRectFilled(min, max, IM_COL32(18, 25, 30, 255), 5.0f);
  draw->AddRect(min, max, IM_COL32(75, 88, 96, 255), 5.0f, 0, 1.0f);
}

void RenderLivePcmGraphs(const AudioSignalAnalysis& live,
                         const LiveAnalyzerMetrics& metrics) {
  ImGui::SetCursorPosX(30.0f);
  if (live.active) {
    ImGui::Text(
        "Crest L/R %.2f / %.2f dB   Headroom %.2f / %.2f dB   DC %+0.5f / %+0.5f",
        metrics.crest_db[0], metrics.crest_db[1], metrics.headroom_db[0],
        metrics.headroom_db[1], metrics.dc[0], metrics.dc[1]);
    ImGui::SetCursorPosX(30.0f);
    ImGui::Text(
        "Peak-to-peak L/R %.3f / %.3f FS   Correlation %+0.3f   Balance L-R %+0.2f dB",
        metrics.peak_to_peak[0], metrics.peak_to_peak[1],
        metrics.stereo_correlation, metrics.balance_db);
  } else {
    ImGui::TextDisabled("Waiting for a live PCM analysis window...");
  }

  const float available_width =
      std::max(360.0f, ImGui::GetContentRegionAvail().x - 60.0f);
  ImVec2 waveform_min = ImGui::GetCursorScreenPos();
  waveform_min.x += 45.0f;
  waveform_min.y += 30.0f;
  ImVec2 waveform_max{waveform_min.x + available_width - 30.0f,
                      waveform_min.y + 220.0f};
  ImDrawList* draw = ImGui::GetWindowDrawList();
  DrawAnalyzerPlotBackground(draw, waveform_min, waveform_max);

  float waveform_peak = 0.0f;
  if (live.active) {
    for (const auto& channel : live.waveform) {
      for (float sample : channel) {
        if (std::isfinite(sample)) {
          waveform_peak = std::max(waveform_peak, std::abs(sample));
        }
      }
    }
  }
  const float waveform_scale = NiceAnalyzerAmplitude(waveform_peak * 1.05f);

  for (int line = 0; line <= 4; ++line) {
    const float normalized = static_cast<float>(line) / 4.0f;
    const float y = waveform_min.y +
                    (waveform_max.y - waveform_min.y) * normalized;
    draw->AddLine({waveform_min.x, y}, {waveform_max.x, y},
                  line == 2 ? IM_COL32(95, 108, 116, 255)
                            : kAnalyzerGridColor);
    char label[16] = {};
    std::snprintf(label, sizeof(label), "%+.2f",
                  waveform_scale * (1.0f - normalized * 2.0f));
    draw->AddText({waveform_min.x - 38.0f, y - 7.0f}, kAnalyzerLabelColor,
                  label);
  }
  for (int line = 1; line < 8; ++line) {
    const float x = waveform_min.x +
                    (waveform_max.x - waveform_min.x) * line / 8.0f;
    draw->AddLine({x, waveform_min.y}, {x, waveform_max.y},
                  kAnalyzerGridColor);
  }

  const ImU32 colors[2] = {kAnalyzerLeftColor, kAnalyzerRightColor};
  const auto interpolate_model = [&](size_t channel, float frequency) {
    if (frequency <= kAudioEqFrequenciesHz.front()) {
      return live.model_response_db[channel].front();
    }
    if (frequency >= kAudioEqFrequenciesHz.back()) {
      return live.model_response_db[channel].back();
    }
    for (size_t band = 1; band < kAudioEqFrequenciesHz.size(); ++band) {
      if (frequency > kAudioEqFrequenciesHz[band]) continue;
      const float low_frequency = kAudioEqFrequenciesHz[band - 1];
      const float high_frequency = kAudioEqFrequenciesHz[band];
      const float blend = std::log(frequency / low_frequency) /
                          std::log(high_frequency / low_frequency);
      return live.model_response_db[channel][band - 1] +
             (live.model_response_db[channel][band] -
              live.model_response_db[channel][band - 1]) *
                 blend;
    }
    return live.model_response_db[channel].back();
  };
  if (live.active) {
    for (size_t channel = 0; channel < 2; ++channel) {
      ImVec2 previous{};
      for (size_t point = 0; point < kAudioAnalyzerWaveformPoints; ++point) {
        const float x = waveform_min.x +
                        (waveform_max.x - waveform_min.x) *
                            static_cast<float>(point) /
                            static_cast<float>(kAudioAnalyzerWaveformPoints - 1);
        const float sample = FiniteAnalyzerValue(
            live.waveform[channel][point]);
        const float y = (waveform_min.y + waveform_max.y) * 0.5f -
                        std::clamp(sample / waveform_scale, -1.0f, 1.0f) *
                            (waveform_max.y - waveform_min.y) * 0.48f;
        const ImVec2 current{x, y};
        if (point > 0) draw->AddLine(previous, current, colors[channel], 1.6f);
        previous = current;
      }
    }
  }
  draw->AddText({waveform_min.x + 9.0f, waveform_min.y + 7.0f},
                kAnalyzerLabelColor, "OUTPUT WAVEFORM");
  char waveform_range[48] = {};
  std::snprintf(waveform_range, sizeof(waveform_range), "RANGE +/-%.2f FS",
                waveform_scale);
  draw->AddText({waveform_min.x + 150.0f, waveform_min.y + 7.0f},
                kAnalyzerLabelColor, waveform_range);
  draw->AddText({waveform_max.x - 132.0f, waveform_min.y + 7.0f},
                kAnalyzerLeftColor, "L");
  draw->AddText({waveform_max.x - 104.0f, waveform_min.y + 7.0f},
                kAnalyzerRightColor, "R");

  const float window_ms =
      live.active && live.sample_rate > 0
          ? static_cast<float>(live.analyzed_frames) * 1000.0f /
                static_cast<float>(live.sample_rate)
          : 0.0f;
  for (int label_index = 0; label_index <= 4; ++label_index) {
    const float normalized = static_cast<float>(label_index) / 4.0f;
    const float x = waveform_min.x +
                    (waveform_max.x - waveform_min.x) * normalized;
    char label[24] = {};
    std::snprintf(label, sizeof(label), "%.1f ms", window_ms * normalized);
    draw->AddText({x - 16.0f, waveform_max.y + 5.0f}, kAnalyzerLabelColor,
                  label);
  }

  const ImVec2 mouse = ImGui::GetIO().MousePos;
  if (live.active && AnalyzerPointInRect(mouse, waveform_min, waveform_max)) {
    const float normalized = std::clamp(
        (mouse.x - waveform_min.x) / (waveform_max.x - waveform_min.x),
        0.0f, 1.0f);
    const size_t point = static_cast<size_t>(std::lround(
        normalized * static_cast<float>(kAudioAnalyzerWaveformPoints - 1)));
    const float cursor_x = waveform_min.x +
                           (waveform_max.x - waveform_min.x) * normalized;
    draw->AddLine({cursor_x, waveform_min.y}, {cursor_x, waveform_max.y},
                  IM_COL32(235, 235, 235, 180), 1.0f);
    ImGui::BeginTooltip();
    ImGui::Text("Time %.3f ms", window_ms * normalized);
    ImGui::TextColored(ImVec4(0.18f, 0.75f, 1.0f, 1.0f), "Left  %+0.6f FS",
                       FiniteAnalyzerValue(live.waveform[0][point]));
    ImGui::TextColored(ImVec4(1.0f, 0.41f, 0.61f, 1.0f), "Right %+0.6f FS",
                       FiniteAnalyzerValue(live.waveform[1][point]));
    ImGui::EndTooltip();
  }

  ImVec2 spectrum_min{waveform_min.x, waveform_max.y + 58.0f};
  ImVec2 spectrum_max{waveform_max.x, spectrum_min.y + 300.0f};
  DrawAnalyzerPlotBackground(draw, spectrum_min, spectrum_max);
  float spectrum_top_db = 0.0f;
  float spectrum_floor_db = -120.0f;
  if (live.active) {
    for (const auto& channel : live.spectrum_dbfs) {
      for (float db : channel) {
        if (!std::isfinite(db)) continue;
        spectrum_top_db = std::max(spectrum_top_db,
            std::ceil(db / 10.0f) * 10.0f);
        spectrum_floor_db = std::min(spectrum_floor_db,
            std::floor(db / 20.0f) * 20.0f);
      }
    }
  }
  spectrum_top_db = std::clamp(spectrum_top_db, 0.0f, 60.0f);
  spectrum_floor_db = std::clamp(spectrum_floor_db, -240.0f, -60.0f);
  for (int line = 0; line <= 6; ++line) {
    const float normalized = static_cast<float>(line) / 6.0f;
    const float y = spectrum_min.y +
                    (spectrum_max.y - spectrum_min.y) * normalized;
    draw->AddLine({spectrum_min.x, y}, {spectrum_max.x, y},
                  kAnalyzerGridColor);
    char label[20] = {};
    const float label_db = spectrum_top_db +
        (spectrum_floor_db - spectrum_top_db) * normalized;
    std::snprintf(label, sizeof(label), "%.0f dB", label_db);
    draw->AddText({spectrum_min.x - 43.0f, y - 7.0f}, kAnalyzerLabelColor,
                  label);
  }
  const char* frequency_labels[] = {"20", "50", "100", "500", "1k",
                                    "5k", "10k", "20k"};
  const float frequency_values[] = {20.0f, 50.0f, 100.0f, 500.0f,
                                    1000.0f, 5000.0f, 10000.0f, 20000.0f};
  for (size_t index = 0; index < 8; ++index) {
    const float normalized =
        std::log10(frequency_values[index] / 20.0f) /
        std::log10(20000.0f / 20.0f);
    const float x = spectrum_min.x +
                    (spectrum_max.x - spectrum_min.x) * normalized;
    draw->AddLine({x, spectrum_min.y}, {x, spectrum_max.y},
                  kAnalyzerGridColor);
    draw->AddText({x - 8.0f, spectrum_max.y + 5.0f}, kAnalyzerLabelColor,
                  frequency_labels[index]);
  }
  if (live.active) {
    for (size_t channel = 0; channel < 2; ++channel) {
      ImVec2 previous{};
      for (size_t bin = 0; bin < kAudioAnalyzerSpectrumBins; ++bin) {
        const float x = spectrum_min.x +
                        (spectrum_max.x - spectrum_min.x) *
                            static_cast<float>(bin) /
                            static_cast<float>(kAudioAnalyzerSpectrumBins - 1);
        const float db = std::clamp(
            FiniteAnalyzerValue(live.spectrum_dbfs[channel][bin],
                                spectrum_floor_db),
            spectrum_floor_db, spectrum_top_db);
        const float y = spectrum_min.y +
            (spectrum_top_db - db) /
                (spectrum_top_db - spectrum_floor_db) *
                (spectrum_max.y - spectrum_min.y);
        const ImVec2 current{x, y};
        if (bin > 0) draw->AddLine(previous, current, colors[channel], 2.0f);
        previous = current;
      }
      const float actual_peak = *std::max_element(
          live.spectrum_dbfs[channel].begin(),
          live.spectrum_dbfs[channel].end());
      const float model_peak = *std::max_element(
          live.model_response_db[channel].begin(),
          live.model_response_db[channel].end());
      previous = {};
      for (size_t bin = 0; bin < kAudioAnalyzerSpectrumBins; ++bin) {
        const float x = spectrum_min.x +
                        (spectrum_max.x - spectrum_min.x) *
                            static_cast<float>(bin) /
                            static_cast<float>(kAudioAnalyzerSpectrumBins - 1);
        const float predicted_db = std::clamp(
            actual_peak +
                interpolate_model(channel,
                                  live.spectrum_frequencies_hz[bin]) -
                model_peak,
            spectrum_floor_db, spectrum_top_db);
        const float y = spectrum_min.y +
                        (spectrum_top_db - predicted_db) /
                            (spectrum_top_db - spectrum_floor_db) *
                            (spectrum_max.y - spectrum_min.y);
        const ImVec2 current{x, y};
        if (bin > 0 && (bin & 1U) == 0U) {
          draw->AddLine(previous, current,
                        channel == 0 ? IM_COL32(70, 180, 220, 155)
                                     : IM_COL32(225, 90, 125, 155),
                        1.5f);
        }
        previous = current;
      }
    }
  }
  draw->AddText({spectrum_min.x + 9.0f, spectrum_min.y + 7.0f},
                kAnalyzerLabelColor,
                "LIVE FFT SPECTRUM (solid) + MNA EXPECTED SHAPE (dashed)");

  if (live.active && AnalyzerPointInRect(mouse, spectrum_min, spectrum_max)) {
    const float normalized = std::clamp(
        (mouse.x - spectrum_min.x) / (spectrum_max.x - spectrum_min.x),
        0.0f, 1.0f);
    const size_t bin = static_cast<size_t>(std::lround(
        normalized * static_cast<float>(kAudioAnalyzerSpectrumBins - 1)));
    const float cursor_x = spectrum_min.x +
                           (spectrum_max.x - spectrum_min.x) * normalized;
    draw->AddLine({cursor_x, spectrum_min.y}, {cursor_x, spectrum_max.y},
                  IM_COL32(235, 235, 235, 180), 1.0f);
    char frequency[32] = {};
    FormatAnalyzerFrequency(frequency, sizeof(frequency),
                            live.spectrum_frequencies_hz[bin]);
    ImGui::BeginTooltip();
    ImGui::Text("Frequency %s", frequency);
    ImGui::TextColored(ImVec4(0.18f, 0.75f, 1.0f, 1.0f), "Left  %+.2f dBFS",
                       FiniteAnalyzerValue(live.spectrum_dbfs[0][bin],
                                           spectrum_floor_db));
    ImGui::TextColored(ImVec4(1.0f, 0.41f, 0.61f, 1.0f), "Right %+.2f dBFS",
                       FiniteAnalyzerValue(live.spectrum_dbfs[1][bin],
                                           spectrum_floor_db));
    ImGui::EndTooltip();
  }
  ImGui::Dummy({0.0f, 640.0f});
}

void RenderStereoScope(const AudioSignalAnalysis& live,
                       const LiveAnalyzerMetrics& metrics) {
  ImGui::SetCursorPosX(30.0f);
  ImGui::Text("Correlation %+0.3f   Stereo width (Side/Mid) %+.2f dB   Balance L-R %+.2f dB",
              metrics.stereo_correlation, metrics.stereo_width_db,
              metrics.balance_db);
  ImGui::SetCursorPosX(30.0f);
  char dominant_left[32] = {};
  char dominant_right[32] = {};
  char centroid_left[32] = {};
  char centroid_right[32] = {};
  FormatAnalyzerFrequency(dominant_left, sizeof(dominant_left),
                          metrics.dominant_hz[0]);
  FormatAnalyzerFrequency(dominant_right, sizeof(dominant_right),
                          metrics.dominant_hz[1]);
  FormatAnalyzerFrequency(centroid_left, sizeof(centroid_left),
                          metrics.spectral_centroid_hz[0]);
  FormatAnalyzerFrequency(centroid_right, sizeof(centroid_right),
                          metrics.spectral_centroid_hz[1]);
  ImGui::Text("Dominant L/R %s / %s   Spectral centroid %s / %s",
              dominant_left, dominant_right, centroid_left, centroid_right);

  const float scope_size =
      std::clamp(ImGui::GetContentRegionAvail().x - 120.0f, 300.0f, 520.0f);
  ImVec2 scope_min = ImGui::GetCursorScreenPos();
  scope_min.x += 60.0f;
  scope_min.y += 30.0f;
  ImVec2 scope_max{scope_min.x + scope_size, scope_min.y + scope_size};
  ImDrawList* draw = ImGui::GetWindowDrawList();
  DrawAnalyzerPlotBackground(draw, scope_min, scope_max);
  const ImVec2 center{(scope_min.x + scope_max.x) * 0.5f,
                      (scope_min.y + scope_max.y) * 0.5f};
  draw->AddLine({center.x, scope_min.y}, {center.x, scope_max.y},
                IM_COL32(80, 95, 104, 255));
  draw->AddLine({scope_min.x, center.y}, {scope_max.x, center.y},
                IM_COL32(80, 95, 104, 255));
  draw->AddLine(scope_min, scope_max, IM_COL32(52, 67, 75, 255));
  draw->AddLine({scope_min.x, scope_max.y}, {scope_max.x, scope_min.y},
                IM_COL32(52, 67, 75, 255));
  draw->AddText({scope_min.x + 8.0f, scope_min.y + 7.0f},
                kAnalyzerLabelColor, "STEREO VECTORSCOPE (L / R)");
  float scope_peak = 0.0f;
  if (live.active) {
    for (const auto& channel : live.waveform) {
      for (float sample : channel) {
        if (std::isfinite(sample)) {
          scope_peak = std::max(scope_peak, std::abs(sample));
        }
      }
    }
  }
  const float scope_scale = NiceAnalyzerAmplitude(scope_peak * 1.05f);
  char scope_range[40] = {};
  std::snprintf(scope_range, sizeof(scope_range), "+/-%.2f FS", scope_scale);
  draw->AddText({scope_min.x + 210.0f, scope_min.y + 7.0f},
                kAnalyzerLabelColor, scope_range);
  if (live.active) {
    ImVec2 previous{};
    for (size_t point = 0; point < kAudioAnalyzerWaveformPoints; ++point) {
      const float left = std::clamp(
          FiniteAnalyzerValue(live.waveform[0][point]) / scope_scale,
          -1.0f, 1.0f);
      const float right = std::clamp(
          FiniteAnalyzerValue(live.waveform[1][point]) / scope_scale,
          -1.0f, 1.0f);
      const ImVec2 current{
          center.x + left * scope_size * 0.46f,
          center.y - right * scope_size * 0.46f};
      if (point > 0) {
        draw->AddLine(previous, current, IM_COL32(80, 235, 170, 155), 1.2f);
      }
      draw->AddCircleFilled(current, 1.5f, IM_COL32(120, 255, 195, 180));
      previous = current;
    }
  }

  ImVec2 meter_min{scope_max.x + 38.0f, scope_min.y};
  ImVec2 meter_max{meter_min.x + 28.0f, scope_max.y};
  draw->AddRectFilled(meter_min, meter_max, IM_COL32(30, 39, 44, 255), 4.0f);
  const float zero_y = (meter_min.y + meter_max.y) * 0.5f;
  draw->AddLine({meter_min.x, zero_y}, {meter_max.x, zero_y},
                IM_COL32(180, 190, 195, 255));
  const float marker_y = meter_max.y -
      (metrics.stereo_correlation + 1.0f) * 0.5f *
          (meter_max.y - meter_min.y);
  draw->AddRectFilled({meter_min.x - 4.0f, marker_y - 3.0f},
                      {meter_max.x + 4.0f, marker_y + 3.0f},
                      metrics.stereo_correlation < 0.0f
                          ? IM_COL32(245, 90, 75, 255)
                          : IM_COL32(75, 220, 135, 255),
                      2.0f);
  draw->AddText({meter_min.x - 8.0f, meter_min.y - 20.0f},
                kAnalyzerLabelColor, "+1");
  draw->AddText({meter_min.x - 6.0f, zero_y - 7.0f}, kAnalyzerLabelColor, "0");
  draw->AddText({meter_min.x - 8.0f, meter_max.y + 5.0f},
                kAnalyzerLabelColor, "-1");
  ImGui::Dummy({0.0f, scope_size + 75.0f});
}

void RenderCircuitResponseGraphs(const AudioCircuitStatus& status) {
  const auto& response_frequencies =
      status.mna_ac_response_frequencies_hz;
  std::array<std::array<float, kAudioCircuitResponsePoints>, 2> response_db{};
  for (size_t channel = 0; channel < 2; ++channel) {
    const std::array<float, 10>& acoustic =
        channel == 0 ? status.left.eq_db : status.right.eq_db;
    for (size_t point = 0; point < response_frequencies.size(); ++point) {
      const float frequency = response_frequencies[point];
      size_t upper = 1;
      while (upper < status.eq_frequencies_hz.size() &&
             status.eq_frequencies_hz[upper] < frequency) {
        ++upper;
      }
      upper = std::min(upper, status.eq_frequencies_hz.size() - 1);
      const size_t lower = upper > 0 ? upper - 1 : 0;
      const float low_frequency = status.eq_frequencies_hz[lower];
      const float high_frequency = status.eq_frequencies_hz[upper];
      const float blend = high_frequency > low_frequency
                              ? std::clamp(
                                    std::log(frequency / low_frequency) /
                                        std::log(high_frequency /
                                                 low_frequency),
                                    0.0f, 1.0f)
                              : 0.0f;
      const float acoustic_db =
          acoustic[lower] + (acoustic[upper] - acoustic[lower]) * blend;
      response_db[channel][point] =
          (status.physical_solver_active
               ? status.mna_ac_response_high_resolution_db[channel][point]
               : 0.0f) +
          acoustic_db;
    }
  }
  const float available_width =
      std::max(360.0f, ImGui::GetContentRegionAvail().x - 60.0f);
  ImVec2 plot_min = ImGui::GetCursorScreenPos();
  plot_min.x += 45.0f;
  plot_min.y += 35.0f;
  ImVec2 plot_max{plot_min.x + available_width - 30.0f,
                  plot_min.y + 320.0f};
  ImDrawList* draw = ImGui::GetWindowDrawList();
  DrawAnalyzerPlotBackground(draw, plot_min, plot_max);
  float response_top_db = 12.0f;
  float response_bottom_db = -60.0f;
  for (const auto* channel : {&response_db[0], &response_db[1]}) {
    for (float db : *channel) {
      if (!std::isfinite(db)) continue;
      response_top_db = std::max(response_top_db,
          std::ceil((db + 3.0f) / 6.0f) * 6.0f);
      response_bottom_db = std::min(response_bottom_db,
          std::floor((db - 3.0f) / 6.0f) * 6.0f);
    }
  }
  response_top_db = std::clamp(response_top_db, 12.0f, 120.0f);
  response_bottom_db = std::clamp(response_bottom_db, -240.0f, -12.0f);
  for (int line = 0; line <= 6; ++line) {
    const float normalized = static_cast<float>(line) / 6.0f;
    const float db = response_top_db +
                     (response_bottom_db - response_top_db) * normalized;
    const float y = plot_min.y + (plot_max.y - plot_min.y) * normalized;
    draw->AddLine({plot_min.x, y}, {plot_max.x, y},
                  std::abs(db) < 0.1f ? IM_COL32(120, 135, 142, 255)
                                      : kAnalyzerGridColor);
    char label[20] = {};
    std::snprintf(label, sizeof(label), "%+.0f dB", db);
    draw->AddText({plot_min.x - 43.0f, y - 7.0f}, kAnalyzerLabelColor,
                  label);
  }
  if (response_top_db > 0.0f && response_bottom_db < 0.0f) {
    const float zero_y = plot_min.y + response_top_db /
        (response_top_db - response_bottom_db) *
        (plot_max.y - plot_min.y);
    draw->AddLine({plot_min.x, zero_y}, {plot_max.x, zero_y},
                  IM_COL32(120, 135, 142, 255), 1.2f);
  }
  const ImU32 colors[2] = {kAnalyzerLeftColor, kAnalyzerRightColor};
  for (size_t band = 0; band < response_frequencies.size(); ++band) {
    const float x = plot_min.x +
                    (plot_max.x - plot_min.x) * static_cast<float>(band) /
                        static_cast<float>(response_frequencies.size() - 1);
    if (band % 32 != 0 && band + 1 != response_frequencies.size()) continue;
    draw->AddLine({x, plot_min.y}, {x, plot_max.y}, kAnalyzerGridColor);
    char frequency[20] = {};
    if (response_frequencies[band] >= 1000.0f) {
      std::snprintf(frequency, sizeof(frequency), "%.0fk",
                    response_frequencies[band] / 1000.0f);
    } else {
      std::snprintf(frequency, sizeof(frequency), "%.0f",
                    response_frequencies[band]);
    }
    draw->AddText({x - 8.0f, plot_max.y + 5.0f}, kAnalyzerLabelColor,
                  frequency);
  }
  for (int channel = 0; channel < 2; ++channel) {
    ImVec2 previous{};
    for (size_t band = 0; band < response_frequencies.size(); ++band) {
      const float x = plot_min.x +
                      (plot_max.x - plot_min.x) * static_cast<float>(band) /
                          static_cast<float>(response_frequencies.size() - 1);
      const float db = std::clamp(
          FiniteAnalyzerValue(response_db[channel][band],
                              response_bottom_db),
          response_bottom_db, response_top_db);
      const float y = plot_min.y +
          (response_top_db - db) /
              (response_top_db - response_bottom_db) *
              (plot_max.y - plot_min.y);
      const ImVec2 current{x, y};
      if (band > 0) draw->AddLine(previous, current, colors[channel], 2.5f);
      draw->AddCircleFilled(current, 3.2f, colors[channel], 12);
      previous = current;
    }
  }
  draw->AddText({plot_min.x + 9.0f, plot_min.y + 7.0f},
                kAnalyzerLabelColor, "MODELED MAGNITUDE RESPONSE");
  draw->AddText({plot_max.x - 110.0f, plot_min.y + 7.0f},
                kAnalyzerLeftColor, "LEFT");
  draw->AddText({plot_max.x - 58.0f, plot_min.y + 7.0f},
                kAnalyzerRightColor, "RIGHT");

  const ImVec2 mouse = ImGui::GetIO().MousePos;
  if (AnalyzerPointInRect(mouse, plot_min, plot_max)) {
    const float normalized = std::clamp(
        (mouse.x - plot_min.x) / (plot_max.x - plot_min.x), 0.0f, 1.0f);
    const size_t band = static_cast<size_t>(std::lround(
        normalized * static_cast<float>(response_frequencies.size() - 1)));
    const float cursor_x = plot_min.x +
        (plot_max.x - plot_min.x) * static_cast<float>(band) /
            static_cast<float>(response_frequencies.size() - 1);
    draw->AddLine({cursor_x, plot_min.y}, {cursor_x, plot_max.y},
                  IM_COL32(235, 235, 235, 180));
    char frequency[32] = {};
    FormatAnalyzerFrequency(frequency, sizeof(frequency),
                            response_frequencies[band]);
    ImGui::BeginTooltip();
    ImGui::Text("Frequency %s", frequency);
    ImGui::TextColored(ImVec4(0.18f, 0.75f, 1.0f, 1.0f), "Left  %+.2f dB",
                       FiniteAnalyzerValue(response_db[0][band],
                                           response_bottom_db));
    ImGui::TextColored(ImVec4(1.0f, 0.41f, 0.61f, 1.0f), "Right %+.2f dB",
                       FiniteAnalyzerValue(response_db[1][band],
                                           response_bottom_db));
    ImGui::Text("Delta L-R %+.2f dB",
                FiniteAnalyzerValue(response_db[0][band]) -
                    FiniteAnalyzerValue(response_db[1][band]));
    ImGui::EndTooltip();
  }

  ImVec2 delta_min{plot_min.x, plot_max.y + 58.0f};
  ImVec2 delta_max{plot_max.x, delta_min.y + 145.0f};
  DrawAnalyzerPlotBackground(draw, delta_min, delta_max);
  const float delta_mid = (delta_min.y + delta_max.y) * 0.5f;
  float delta_range_db = 12.0f;
  for (size_t band = 0; band < response_frequencies.size(); ++band) {
    const float delta =
        FiniteAnalyzerValue(response_db[0][band]) -
        FiniteAnalyzerValue(response_db[1][band]);
    delta_range_db = std::max(delta_range_db,
        std::ceil((std::abs(delta) + 1.0f) / 6.0f) * 6.0f);
  }
  delta_range_db = std::clamp(delta_range_db, 12.0f, 120.0f);
  draw->AddLine({delta_min.x, delta_mid}, {delta_max.x, delta_mid},
                IM_COL32(110, 125, 133, 255));
  char delta_title[80] = {};
  std::snprintf(delta_title, sizeof(delta_title),
                "STEREO RESPONSE DELTA (L - R, +/-%.0f dB)",
                delta_range_db);
  draw->AddText({delta_min.x + 9.0f, delta_min.y + 7.0f},
                kAnalyzerLabelColor, delta_title);
  ImVec2 previous{};
  for (size_t band = 0; band < response_frequencies.size(); ++band) {
    const float x = delta_min.x +
                    (delta_max.x - delta_min.x) * static_cast<float>(band) /
                        static_cast<float>(response_frequencies.size() - 1);
    const float delta = std::clamp(
        FiniteAnalyzerValue(response_db[0][band]) -
            FiniteAnalyzerValue(response_db[1][band]),
        -delta_range_db, delta_range_db);
    const float y = delta_mid - delta / (2.0f * delta_range_db) *
                                    (delta_max.y - delta_min.y);
    const ImVec2 current{x, y};
    if (band > 0) {
      draw->AddLine(previous, current, IM_COL32(250, 205, 75, 255), 2.0f);
    }
    draw->AddCircleFilled(current, 2.7f, IM_COL32(250, 205, 75, 255));
    previous = current;
  }
  ImGui::Dummy({0.0f, 570.0f});
}

void RenderAudioAnalysisMode() {
  const AudioCircuitStatus status = GetAudioCircuitStatus();
  const AudioSignalAnalysis live = GetAudioSignalAnalysis();
  ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.96f, 0.96f, 0.96f, 1.0f));
  if (ImGui::BeginChild("AudioAnalysis", ImVec2(0, 0), true)) {
    ImGui::SetCursorPos(ImVec2(30.0f, 28.0f));
    ImGui::TextUnformatted("SIGNAL ANALYZER");
    ImGui::SetCursorPosX(30.0f);
    ImGui::Separator();
    ImGui::SetCursorPosX(30.0f);
    ImGui::Text("Circuit gain L  %7.2f %%  (%+7.2f dB)",
                 status.applied_left_scalar * 100.0f, status.left.output_db);
    ImGui::SetCursorPosX(30.0f);
    ImGui::Text("Circuit gain R  %7.2f %%  (%+7.2f dB)",
                 status.applied_right_scalar * 100.0f,
                 status.right.output_db);
    ImGui::SetCursorPosX(30.0f);
    if (status.left.amplifier_powered || status.right.amplifier_powered) {
      ImGui::Text(
          "Amp supply L/R  %6.2fV %.2fA / %6.2fV %.2fA",
          status.left.amplifier_supply_voltage,
          status.left.amplifier_current_limit_amps,
          status.right.amplifier_supply_voltage,
          status.right.amplifier_current_limit_amps);
      ImGui::SetCursorPosX(30.0f);
      ImGui::Text(
          "Speaker limit   %6.2f / %6.2f W of %.1f W rated (peak %5.2f / %5.2f V)",
          status.left.maximum_speaker_power_watts,
          status.right.maximum_speaker_power_watts,
          status.left.speaker_rated_power_watts,
          status.left.maximum_speaker_peak_voltage,
          status.right.maximum_speaker_peak_voltage);
      ImGui::SetCursorPosX(30.0f);
      const bool output_safe = !status.left.speaker_damage_risk &&
                               !status.right.speaker_damage_risk;
      ImGui::TextColored(
          output_safe ? ImVec4(0.15f, 0.65f, 0.25f, 1.0f)
                      : ImVec4(0.85f, 0.22f, 0.18f, 1.0f),
          "Output safety   DC block L/R %s/%s | emitter R %s/%s | DC %.2f/%.2f V",
          status.left.dc_blocking_capacitor_present ? "OK" : "MISSING",
          status.right.dc_blocking_capacitor_present ? "OK" : "MISSING",
          status.left.emitter_ballast_present ? "OK" : "MISSING",
          status.right.emitter_ballast_present ? "OK" : "MISSING",
          status.left.estimated_dc_offset_volts,
          status.right.estimated_dc_offset_volts);
    } else {
      ImGui::TextDisabled("Amplifier power stage: not powered");
    }
    ImGui::SetCursorPosX(30.0f);
    ImGui::Text("Model noise L/R %7.2f / %7.2f dBFS",
                 status.left.noise_floor_db, status.right.noise_floor_db);
    ImGui::SetCursorPosX(30.0f);
    ImGui::Text("Model THD L/R   %7.3f / %7.3f %%",
                 status.left.thd_percent, status.right.thd_percent);
    ImGui::SetCursorPosX(30.0f);
    const auto count_dac_bits = [](std::uint16_t mask) {
      int count = 0;
      while (mask != 0) {
        count += mask & 1U;
        mask = static_cast<std::uint16_t>(mask >> 1U);
      }
      return count;
    };
    ImGui::Text("DAC bus L/R     %2d/16 / %2d/16  [%04X / %04X]",
                count_dac_bits(status.left.dac_connected_mask),
                count_dac_bits(status.right.dac_connected_mask),
                static_cast<unsigned int>(status.left.dac_connected_mask),
                static_cast<unsigned int>(status.right.dac_connected_mask));
    ImGui::SetCursorPosX(30.0f);
    ImGui::Text("DAC weight error L/R %7.3f / %7.3f %%",
                status.left.dac_weight_error_percent,
                status.right.dac_weight_error_percent);
    ImGui::SetCursorPosX(30.0f);
    ImGui::Text("DAC clock pitch  x%.3f%s", status.pitch_shift_ratio,
                std::abs(status.pitch_shift_ratio - 1.0f) > 0.001f
                    ? "  (clock mismatch effect)"
                    : "");
    ImGui::SetCursorPosX(30.0f);
    if (status.physical_solver_active) {
      ImGui::Text(
          "MNA nodes/order->reduced/dynamic  %d / %d->%d / %d | DC L/R %+.4f/%+.4f V",
          status.mna_node_count, status.mna_matrix_order,
          status.mna_reduced_matrix_order, status.mna_dynamic_elements,
          status.mna_dc_left_volts,
          status.mna_dc_right_volts);
      ImGui::SetCursorPosX(30.0f);
      ImGui::Text("MNA block %6.3f ms  deadline %5.1f %%",
                  status.mna_block_time_ms, status.mna_deadline_percent);
      ImGui::SetCursorPosX(30.0f);
      ImGui::TextColored(
          status.mna_converged ? ImVec4(0.18f, 0.62f, 0.28f, 1.0f)
                               : ImVec4(0.88f, 0.20f, 0.16f, 1.0f),
          "MNA NR %s %d x%d  residual %.3e | speaker %.2f/%.2f W  coil %.1f/%.1f C",
          status.mna_converged ? "OK" : "FAILED",
          status.mna_newton_iterations, status.mna_nonlinear_substeps,
          status.mna_residual,
          status.mna_speaker_power_left_watts,
          status.mna_speaker_power_right_watts,
          status.mna_coil_temperature_left_c,
          status.mna_coil_temperature_right_c);
    } else {
      ImGui::TextColored(ImVec4(0.85f, 0.22f, 0.18f, 1.0f),
                         "MNA unavailable: %s", status.mna_error.c_str());
    }
    ImGui::SetCursorPosX(30.0f);
    if (live.active) {
      ImGui::Text("Live RMS L/R    %7.2f / %7.2f dBFS",
                  live.rms_dbfs[0], live.rms_dbfs[1]);
      ImGui::SetCursorPosX(30.0f);
      ImGui::Text("Live peak L/R   %7.2f / %7.2f dBFS",
                  live.peak_dbfs[0], live.peak_dbfs[1]);
      ImGui::SetCursorPosX(30.0f);
      ImGui::Text("Clipped L/R     %7.3f / %7.3f %%   [%u Hz / %zu frames]",
                  live.clipped_percent[0], live.clipped_percent[1],
                  live.sample_rate, live.analyzed_frames);
      ImGui::SetCursorPosX(30.0f);
      ImGui::Text("Fundamental L/R %7.1f / %7.1f Hz",
                  live.fundamental_hz[0], live.fundamental_hz[1]);
      ImGui::SetCursorPosX(30.0f);
      ImGui::Text("THD L/R         %7.3f / %7.3f %%", live.thd_percent[0],
                  live.thd_percent[1]);
      ImGui::SetCursorPosX(30.0f);
      ImGui::Text("THD+N / SNR L   %7.3f %% / %7.2f dB | R %7.3f %% / %7.2f dB",
                  live.thd_plus_noise_percent[0], live.snr_db[0],
                  live.thd_plus_noise_percent[1], live.snr_db[1]);
    } else {
      ImGui::TextDisabled("Live PCM: waiting for processed audio...");
    }
    ImGui::SetCursorPosX(30.0f);
    const bool has_serious_diagnostic = std::any_of(
        status.structured_diagnostics.begin(),
        status.structured_diagnostics.end(),
        [](const AudioCircuitDiagnostic& diagnostic) {
          return diagnostic.severity == AudioDiagnosticSeverity::ERROR_LEVEL ||
                 diagnostic.severity == AudioDiagnosticSeverity::FATAL;
        });
    const bool diagnosis_healthy =
        status.physical_solver_active
            ? !has_serious_diagnostic
            : status.diagnosis.find("circuits are active") !=
                  std::string::npos;
    ImGui::TextColored(status.route_complete && status.return_complete &&
                               !status.left.speaker_damage_risk &&
                               !status.right.speaker_damage_risk &&
                               diagnosis_healthy
                           ? ImVec4(0.15f, 0.65f, 0.25f, 1.0f)
                           : ImVec4(0.85f, 0.22f, 0.18f, 1.0f),
                       "%s", status.diagnosis.c_str());

    const LiveAnalyzerMetrics live_metrics =
        ComputeLiveAnalyzerMetrics(live);
    if (ImGui::BeginTabBar("SignalAnalyzerTabs")) {
      if (ImGui::BeginTabItem("LIVE PCM")) {
        RenderLivePcmGraphs(live, live_metrics);
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("STEREO SCOPE")) {
        RenderStereoScope(live, live_metrics);
        ImGui::EndTabItem();
      }
      if (ImGui::BeginTabItem("CIRCUIT RESPONSE")) {
        RenderCircuitResponseGraphs(status);
        ImGui::EndTabItem();
      }
      ImGui::EndTabBar();
    }
  }
  ImGui::EndChild();
  ImGui::PopStyleColor();
}

void SetHighPrecisionTimer(bool enable, bool* active) {
#ifdef _WIN32
  if (!active) {
    return;
  }
  if (enable && !(*active)) {
    timeBeginPeriod(1);
    *active = true;
  } else if (!enable && *active) {
    timeEndPeriod(1);
    *active = false;
  }
#else
  (void)enable;
  (void)active;
#endif
}

double GetMonitorRefreshRateForWindow(GLFWwindow* window) {
  if (!window) {
    return 0.0;
  }
  int monitor_count = 0;
  GLFWmonitor** monitors = glfwGetMonitors(&monitor_count);
  if (!monitors || monitor_count <= 0) {
    return 0.0;
  }

  int win_x = 0;
  int win_y = 0;
  int win_w = 0;
  int win_h = 0;
  glfwGetWindowPos(window, &win_x, &win_y);
  glfwGetWindowSize(window, &win_w, &win_h);

  GLFWmonitor* best_monitor = nullptr;
  int best_overlap = 0;

  for (int i = 0; i < monitor_count; ++i) {
    GLFWmonitor* monitor = monitors[i];
    if (!monitor) {
      continue;
    }
    int mon_x = 0;
    int mon_y = 0;
    glfwGetMonitorPos(monitor, &mon_x, &mon_y);
    const GLFWvidmode* mode = glfwGetVideoMode(monitor);
    if (!mode) {
      continue;
    }
    int mon_w = mode->width;
    int mon_h = mode->height;

    int overlap_w =
        std::max(0, std::min(win_x + win_w, mon_x + mon_w) -
                        std::max(win_x, mon_x));
    int overlap_h =
        std::max(0, std::min(win_y + win_h, mon_y + mon_h) -
                        std::max(win_y, mon_y));
    int overlap = overlap_w * overlap_h;
    if (overlap > best_overlap) {
      best_overlap = overlap;
      best_monitor = monitor;
    }
  }

  if (!best_monitor) {
    best_monitor = glfwGetPrimaryMonitor();
  }
  if (!best_monitor) {
    return 0.0;
  }
  const GLFWvidmode* best_mode = glfwGetVideoMode(best_monitor);
  if (!best_mode || best_mode->refreshRate <= 0) {
    return 0.0;
  }
  return static_cast<double>(best_mode->refreshRate);
}

bool EqualsIgnoreCase(const char* lhs, const char* rhs) {
  if (!lhs || !rhs) {
    return false;
  }
  while (*lhs && *rhs) {
    unsigned char l = static_cast<unsigned char>(*lhs);
    unsigned char r = static_cast<unsigned char>(*rhs);
    if (std::tolower(l) != std::tolower(r)) {
      return false;
    }
    ++lhs;
    ++rhs;
  }
  return *lhs == '\0' && *rhs == '\0';
}

bool ContainsIgnoreCase(const std::string& text, const char* needle) {
  if (!needle || *needle == '\0') return true;
  return std::search(
             text.begin(), text.end(), needle, needle + std::strlen(needle),
             [](char lhs, char rhs) {
               return std::tolower(static_cast<unsigned char>(lhs)) ==
                      std::tolower(static_cast<unsigned char>(rhs));
             }) != text.end();
}

void OpenExternalUrl(const char* url) {
  if (!url) {
    return;
  }
#ifdef _WIN32
  ShellExecuteA(nullptr, "open", url, nullptr, nullptr, SW_SHOWNORMAL);
#else
  (void)url;
#endif
}

}  // namespace

bool Application::PromptSaveProjectPackageDialog(
    const std::string& default_file_name,
    const std::string& project_name) {
#ifdef _WIN32
  OPENFILENAMEA ofn;
  CHAR szFile[260] = {0};
  std::strncpy(szFile, default_file_name.c_str(), sizeof(szFile) - 1);
  ZeroMemory(&ofn, sizeof(ofn));
  ofn.lStructSize = sizeof(ofn);
  ofn.hwndOwner = glfwGetWin32Window(window_);
  ofn.lpstrFile = szFile;
  ofn.nMaxFile = sizeof(szFile);
  ofn.lpstrFilter =
      "Audio Circuit Project (*.acproj)\0*.acproj\0All Files (*.*)\0*.*\0";
  ofn.nFilterIndex = 1;
  ofn.lpstrDefExt = "acproj";
  ofn.Flags = OFN_OVERWRITEPROMPT | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
  if (GetSaveFileNameA(&ofn) != TRUE) {
    return false;
  }

  std::string save_path = ofn.lpstrFile;
  if (save_path.size() < 7 ||
      save_path.substr(save_path.size() - 7) != ".acproj") {
    save_path += ".acproj";
  }
  return SaveProjectPackage(save_path, project_name);
#else
  return SaveProjectPackage(default_file_name, project_name);
#endif
}

void Application::BeginFrame() {
  glClearColor(0.94f, 0.94f, 0.94f, 1.00f);
  glClear(GL_COLOR_BUFFER_BIT);

  RefreshImGuiFontsIfNeeded();
  ImGui::GetIO().FontGlobalScale = 1.0f;

  ImGui_ImplOpenGL3_NewFrame();
  ImGui_ImplGlfw_NewFrame();
  ImGui::NewFrame();
  BeginOverlayInputCaptureFrame();
  ProcessFrameKeyboardInput();
}

void Application::Render() {
  RenderUI();

  ImGui::Render();
  ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

  glfwSwapBuffers(window_);
  if (!ui_settings_.vsync_enabled && ui_settings_.frame_limit_enabled) {
    using clock = std::chrono::steady_clock;
    double refresh = GetMonitorRefreshRateForWindow(window_);
    if (refresh > 1.0) {
      monitor_refresh_rate_ = refresh;
    }
    double target_hz =
        (monitor_refresh_rate_ > 1.0) ? monitor_refresh_rate_ : 60.0;
    auto target_dt = std::chrono::duration_cast<clock::duration>(
        std::chrono::duration<double>(1.0 / target_hz));
    auto now = clock::now();
    if (!render_time_initialized_) {
      last_render_time_ = now;
      next_frame_time_ = now + target_dt;
      render_time_initialized_ = true;
    } else {
      if (now < next_frame_time_) {
        auto remaining = next_frame_time_ - now;
        const auto spin_margin = std::chrono::duration_cast<clock::duration>(
            std::chrono::duration<double>(0.001));
        if (remaining > spin_margin) {
          std::this_thread::sleep_for(remaining - spin_margin);
        }
        while (clock::now() < next_frame_time_) {
          std::this_thread::yield();
        }
        now = clock::now();
      } else {
        const auto reset_threshold =
            std::chrono::duration_cast<clock::duration>(
                std::chrono::duration<double>(0.25));
        if (now - next_frame_time_ > reset_threshold) {
          next_frame_time_ = now + target_dt;
        }
      }
      last_render_time_ = now;
      next_frame_time_ += target_dt;
    }
  } else {
    render_time_initialized_ = false;
  }
}

void Application::BeginOverlayInputCaptureFrame() {
  overlay_input_capture_.BeginFrame();
}

void Application::RegisterOverlayInputCaptureRect(ImVec2 min, ImVec2 max,
                                                  bool capturing) {
  overlay_input_capture_.RegisterRect(min, max, capturing);
}

bool Application::IsPointInOverlayCaptureRect(ImVec2 screen_pos) const {
  return overlay_input_capture_.Contains(screen_pos);
}

void Application::RenderUI() {
  if (warmup_stage_ != WarmupStage::kComplete) {
    RenderSplashScreen();
    return;
  }

  float menu_height = 0.0f;
  RenderMainMenuBar(&menu_height);
  ImVec2 display_size = ImGui::GetIO().DisplaySize;
  if (menu_height > display_size.y) {
    menu_height = 0.0f;
  }

  ImGui::SetNextWindowPos(ImVec2(0, menu_height));
  ImGui::SetNextWindowSize(
      ImVec2(display_size.x, display_size.y - menu_height));

  ImGui::PushStyleVar(ImGuiStyleVar_WindowRounding, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowBorderSize, 0.0f);
  ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0.0f, 0.0f));

  ImGuiWindowFlags window_flags =
      ImGuiWindowFlags_NoTitleBar | ImGuiWindowFlags_NoCollapse |
      ImGuiWindowFlags_NoResize | ImGuiWindowFlags_NoMove |
      ImGuiWindowFlags_NoBringToFrontOnFocus | ImGuiWindowFlags_NoNavFocus |
      ImGuiWindowFlags_NoScrollbar | ImGuiWindowFlags_NoScrollWithMouse;

  if (ImGui::Begin("MainAppWindow", nullptr, window_flags)) {
    RenderHeader();

    switch (current_mode_) {
      case Mode::WIRING:
        RenderWiringModeUI();
        break;
      case Mode::PROGRAMMING:
        RenderAudioAnalysisMode();
        break;
    }
  }
  ImGui::End();
  ImGui::PopStyleVar(3);
  RenderRtlLibraryPanel();
  RenderRtlToolchainPanel();
  if (current_mode_ == Mode::WIRING) {
    ProcessDeferredCanvasWheelInput();
  }
  RenderPhysicsWarningDialog();
  RenderShortcutHelpDialog();
}

void Application::RenderSplashScreen() {
  ImGuiIO& io = ImGui::GetIO();
  ImVec2 display = io.DisplaySize;

  ImGui::SetNextWindowPos(ImVec2(0.0f, 0.0f));
  ImGui::SetNextWindowSize(display);
  ImGuiWindowFlags flags =
      ImGuiWindowFlags_NoDecoration | ImGuiWindowFlags_NoMove |
      ImGuiWindowFlags_NoSavedSettings | ImGuiWindowFlags_NoInputs;

  if (ImGui::Begin("SplashScreen", nullptr, flags)) {
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    ImVec2 p0 = ImGui::GetWindowPos();
    ImVec2 p1 = ImVec2(p0.x + display.x, p0.y + display.y);
    draw_list->AddRectFilled(p0, p1, IM_COL32(245, 245, 245, 255));

    float t = static_cast<float>(ImGui::GetTime());
    int dots = static_cast<int>(t * 2.0f) % 4;
    char message[64] = "Starting";
    char cache_line[64] = "Preparing caches";
    std::snprintf(message + std::strlen(message),
                  sizeof(message) - std::strlen(message),
                  "%.*s", dots, "...");
    std::snprintf(cache_line + std::strlen(cache_line),
                  sizeof(cache_line) - std::strlen(cache_line),
                  "%.*s", dots, "...");

    const char* title = TR("ui.header.title", "Audio Circuit Simulator");
    ImVec2 title_size = ImGui::CalcTextSize(title);
    ImVec2 line1_size = ImGui::CalcTextSize(message);
    ImVec2 line2_size = ImGui::CalcTextSize(cache_line);

    float center_x = display.x * 0.5f;
    float center_y = display.y * 0.5f;
    ImGui::SetCursorPos(
        ImVec2(center_x - title_size.x * 0.5f, center_y - 60.0f));
    ImGui::TextUnformatted(title);

    ImGui::SetCursorPos(
        ImVec2(center_x - line1_size.x * 0.5f, center_y - 10.0f));
    ImGui::TextUnformatted(message);

    ImGui::SetCursorPos(
        ImVec2(center_x - line2_size.x * 0.5f, center_y + 15.0f));
    ImGui::TextUnformatted(cache_line);
  }
  ImGui::End();
}

void Application::RenderWiringModeUI() {
  RenderToolbar();
  RenderMainArea();
}

void Application::RenderMainMenuBar(float* out_height) {
  if (out_height) {
    *out_height = 0.0f;
  }

  if (!ImGui::BeginMainMenuBar()) {
    return;
  }
  {
    ImVec2 bar_min = ImGui::GetWindowPos();
    ImVec2 bar_size = ImGui::GetWindowSize();
    ImVec2 bar_max(bar_min.x + bar_size.x, bar_min.y + bar_size.y);
    RegisterOverlayInputCaptureRect(
        bar_min, bar_max,
        ImGui::IsWindowHovered(ImGuiHoveredFlags_RootAndChildWindows) ||
            ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows));
  }

  if (out_height) {
    *out_height = ImGui::GetFrameHeight();
  }

  if (ImGui::BeginMenu(TR("menu.project", "Project"))) {
    if (ImGui::MenuItem(TR("menu.project.save", "Save Project..."))) {
      PromptSaveProjectPackageDialog();
    }
    if (ImGui::MenuItem(TR("menu.project.load", "Load Project..."))) {
#ifdef _WIN32
      OPENFILENAMEA ofn;
      CHAR szFile[260] = {0};
      ZeroMemory(&ofn, sizeof(ofn));
      ofn.lStructSize = sizeof(ofn);
      ofn.hwndOwner = glfwGetWin32Window(window_);
      ofn.lpstrFile = szFile;
      ofn.nMaxFile = sizeof(szFile);
      ofn.lpstrFilter =
          "Audio Circuit Project (*.acproj)\0*.acproj\0All Files (*.*)\0*.*\0";
      ofn.nFilterIndex = 1;
      ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;
      if (GetOpenFileNameA(&ofn) == TRUE) {
        LoadProjectPackage(ofn.lpstrFile);
      }
#else
      LoadProjectPackage("project.acproj");
#endif
    }
    ImGui::EndMenu();
  }

  if (ImGui::BeginMenu(TR("menu.rtl", "RTL"))) {
    if (ImGui::MenuItem(TR("menu.rtl_library", "RTL Library"))) {
      rtl_editor_focus_module_id_.clear();
      show_rtl_library_panel_ = true;
    }
    if (ImGui::MenuItem(TR("ui.rtl.toolchain", "RTL Toolchain"))) {
      show_rtl_toolchain_panel_ = true;
    }
    ImGui::EndMenu();
  }

  if (ImGui::BeginMenu(TR("menu.settings", "Settings"))) {
    if (ImGui::BeginMenu(TR("menu.language", "Language"))) {
      const char* user_lang = GetUserLanguageCode();
      const char* active_lang = GetActiveLanguageCode();
      const char* selected = (user_lang && user_lang[0] != '\0')
                                 ? user_lang
                                 : active_lang;

      bool is_en = std::strcmp(selected, "en") == 0;
      bool is_ko = std::strcmp(selected, "ko") == 0;
      bool is_ja = std::strcmp(selected, "ja") == 0;

      if (ImGui::MenuItem(TR("lang.english", "English"), nullptr, is_en)) {
        SetUserLanguageCode("en");
      }
      if (ImGui::MenuItem(TR("lang.korean", "Korean"), nullptr, is_ko)) {
        SetUserLanguageCode("ko");
      }
      if (ImGui::MenuItem(TR("lang.japanese", "Japanese"), nullptr, is_ja)) {
        SetUserLanguageCode("ja");
      }
      ImGui::EndMenu();
    }
    if (ImGui::BeginMenu("Audio Output")) {
      const auto& output_devices = GetAudioOutputDevices(false);
      const char* selected_input_name = "Install/select Virtual Audio Cable";
      const char* selected_output_name = "System Default Output";
      const AudioOutputDevice* vac_device = nullptr;
      bool selected_input_found = false;
      for (const AudioOutputDevice& device : output_devices) {
        if (!device.id.empty() &&
            ContainsIgnoreCase(device.name, "virtual audio cable")) {
          if (!vac_device) vac_device = &device;
        }
        if (!device.id.empty() &&
            device.id == ui_settings_.audio_input_device_id) {
          selected_input_name = device.name.c_str();
          selected_input_found = true;
        }
        if (device.id == ui_settings_.audio_output_device_id) {
          selected_output_name = device.name.c_str();
        }
      }
      if (!selected_input_found && vac_device) {
        ui_settings_.audio_input_device_id = vac_device->id;
        selected_input_name = vac_device->name.c_str();
        SaveUiSettings(ui_settings_);
      }
      if (ImGui::BeginCombo("Engine Sink (VAC)", selected_input_name)) {
        for (const AudioOutputDevice& device : output_devices) {
          if (device.id.empty() ||
              !ContainsIgnoreCase(device.name, "virtual audio cable")) {
            continue;
          }
          const bool selected = device.id == ui_settings_.audio_input_device_id;
          if (ImGui::Selectable(device.name.c_str(), selected)) {
            ui_settings_.audio_input_device_id = device.id;
            SaveUiSettings(ui_settings_);
          }
          if (selected) ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
      }
      if (ImGui::BeginCombo("Speaker Output", selected_output_name)) {
        for (const AudioOutputDevice& device : output_devices) {
          const bool selected =
              device.id == ui_settings_.audio_output_device_id;
          std::string label = device.name;
          if (!device.id.empty() && device.is_default) {
            label += "  [Windows default]";
          }
          if (ImGui::Selectable(label.c_str(), selected)) {
            ui_settings_.audio_output_device_id = device.id;
            SaveUiSettings(ui_settings_);
          }
          if (selected) ImGui::SetItemDefaultFocus();
        }
        ImGui::EndCombo();
      }
      if (ImGui::Button("Refresh Device List")) {
        GetAudioOutputDevices(true);
      }
      const AudioStreamBridgeStatus bridge_status =
          GetAudioStreamBridgeStatus();
      if (bridge_status.running) {
        ImGui::TextColored(ImVec4(0.15f, 0.65f, 0.25f, 1.0f),
                           "PCM circuit engine is running.");
        ImGui::Text("Windows routing: %s",
                    bridge_status.windows_routing_active
                        ? "VAC endpoint loopback active"
                        : "not active");
        ImGui::Text("Captured: %llu / circuit processed: %llu frames",
                    static_cast<unsigned long long>(
                        bridge_status.captured_frames),
                    static_cast<unsigned long long>(
                        bridge_status.processed_frames));
        ImGui::Text(
            "Buffer: %d ms / capture q: %u / render q: %u",
            bridge_status.buffer_ms, bridge_status.capture_queued_frames,
            bridge_status.queued_frames);
        ImGui::Text(
            "Underrun: %llu / dropped: %llu",
            static_cast<unsigned long long>(bridge_status.underrun_frames),
            static_cast<unsigned long long>(bridge_status.dropped_frames));
        const char* silence_reason = "NONE";
        switch (bridge_status.silence_reason) {
          case AudioBridgeSilenceReason::RTL_WAIT:
            silence_reason = "RTL_WAIT";
            break;
          case AudioBridgeSilenceReason::MNA_FAILURE:
            silence_reason = "MNA_FAILURE";
            break;
          case AudioBridgeSilenceReason::RENDER_STARVATION:
            silence_reason = "RENDER_STARVATION";
            break;
          case AudioBridgeSilenceReason::PHYSICAL_ZERO_OUTPUT:
            silence_reason = "PHYSICAL_ZERO_OUTPUT";
            break;
          case AudioBridgeSilenceReason::NONE:
          default:
            break;
        }
        ImGui::Text("Valid RTL/MNA/render: %llu / %llu / %llu",
                    static_cast<unsigned long long>(
                        bridge_status.rtl_valid_frames),
                    static_cast<unsigned long long>(
                        bridge_status.mna_valid_frames),
                    static_cast<unsigned long long>(
                        bridge_status.rendered_frames));
        ImGui::Text("Stage ms RTL/MNA/render: %.3f / %.3f / %.3f",
                    bridge_status.rtl_time_ms, bridge_status.mna_time_ms,
                    bridge_status.render_time_ms);
        ImGui::Text("Stage p99 ms: %.3f / %.3f / %.3f",
                    bridge_status.rtl_p99_ms, bridge_status.mna_p99_ms,
                    bridge_status.render_p99_ms);
        ImGui::Text("Full pipeline current/p99: %.3f / %.3f ms",
                    bridge_status.processing_time_ms,
                    bridge_status.processing_p99_ms);
        ImGui::Text("RMS input/MNA/render: %.6f / %.6f / %.6f",
                    bridge_status.input_rms, bridge_status.output_rms,
                    bridge_status.render_pcm_rms);
        ImGui::Text("Speaker endpoint: %.0f%% / %s | silence: %s",
                    bridge_status.output_endpoint_volume * 100.0f,
                    bridge_status.output_endpoint_muted ? "MUTED" : "active",
                    silence_reason);
        ImGui::Text("Capture overflow/discontinuity: %llu / %llu",
                    static_cast<unsigned long long>(
                        bridge_status.capture_overflow_frames),
                    static_cast<unsigned long long>(
                        bridge_status.capture_discontinuity_frames));
        ImGui::Text("RTL sequence loss / concealed: %llu / %llu frames",
                    static_cast<unsigned long long>(
                        bridge_status.rtl_sequence_loss_frames),
                    static_cast<unsigned long long>(
                        bridge_status.concealed_frames));
        if (bridge_status.captured_frames == 0) {
          ImGui::TextWrapped(
              "No Windows audio has reached VAC yet. Start playback "
              "after starting the simulation.");
        }
      } else if (!bridge_status.error.empty()) {
        ImGui::TextWrapped("Audio engine: %s", bridge_status.error.c_str());
      }
      ImGui::Separator();
      float minimum = ui_settings_.audio_min_volume_percent;
      float maximum = ui_settings_.audio_max_volume_percent;
      const bool minimum_submitted = ImGui::InputFloat(
          "Minimum (%)", &minimum, 1.0f, 5.0f, "%.1f",
          ImGuiInputTextFlags_EnterReturnsTrue);
      if (minimum_submitted || ImGui::IsItemDeactivatedAfterEdit()) {
        ui_settings_.audio_min_volume_percent =
            std::clamp(minimum, 0.0f,
                       ui_settings_.audio_max_volume_percent);
        SaveUiSettings(ui_settings_);
      }
      const bool maximum_submitted = ImGui::InputFloat(
          "Maximum (%)", &maximum, 1.0f, 5.0f, "%.1f",
          ImGuiInputTextFlags_EnterReturnsTrue);
      if (maximum_submitted || ImGui::IsItemDeactivatedAfterEdit()) {
        ui_settings_.audio_max_volume_percent =
            std::clamp(maximum, ui_settings_.audio_min_volume_percent,
                       100.0f);
        SaveUiSettings(ui_settings_);
      }
      static int buffer_ms_draft = 80;
      static int last_saved_buffer_ms = -1;
      if (last_saved_buffer_ms != ui_settings_.audio_buffer_ms &&
          !ImGui::IsAnyItemActive()) {
        buffer_ms_draft = ui_settings_.audio_buffer_ms;
        last_saved_buffer_ms = ui_settings_.audio_buffer_ms;
      }
      const bool buffer_submitted = ImGui::InputInt(
          "Audio buffer (ms)", &buffer_ms_draft, 10, 50,
          ImGuiInputTextFlags_EnterReturnsTrue);
      if (buffer_submitted || ImGui::IsItemDeactivatedAfterEdit()) {
        buffer_ms_draft = std::clamp(buffer_ms_draft, 10, 500);
        if (ui_settings_.audio_buffer_ms != buffer_ms_draft) {
          ui_settings_.audio_buffer_ms = buffer_ms_draft;
          SaveUiSettings(ui_settings_);
        }
        last_saved_buffer_ms = ui_settings_.audio_buffer_ms;
      }
      ImGui::TextDisabled(
          "10-500 ms (80 ms default). Higher values reduce crackling but add latency.");
      ImGui::TextDisabled("Disconnected circuits are always muted.");
      ImGui::EndMenu();
    }
    RenderUiSettingsMenu();
    if (IsLanguageRestartRequired()) {
      ImGui::TextDisabled(
          "%s", TR("lang.restart_required", "Restart required to apply."));
      if (ImGui::MenuItem(TR("lang.restart_now", "Restart now..."))) {
        show_restart_popup_ = true;
      }
    }
    ImGui::EndMenu();
  }

  ImGui::EndMainMenuBar();

  const char* restart_popup_title =
      TR("lang.restart_title", "Restart Application");
  if (show_restart_popup_) {
    ImGui::OpenPopup(restart_popup_title);
    show_restart_popup_ = false;
  }

  bool restart_popup_open = true;
  if (ImGui::BeginPopupModal(restart_popup_title, &restart_popup_open,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    ImGui::TextUnformatted(
        TR("lang.restart_prompt", "Restart application now?"));
    ImGui::Spacing();
    if (ImGui::Button(TR("lang.restart_yes", "Restart"), ImVec2(120, 0))) {
      RestartApplication();
      ImGui::CloseCurrentPopup();
    }
    ImGui::SameLine();
    if (ImGui::Button(TR("lang.restart_no", "Cancel"), ImVec2(120, 0))) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
}

void Application::RenderUiSettingsMenu() {
  if (!ImGui::BeginMenu(TR("menu.ui_settings", "UI"))) {
    return;
  }

  float ui_scale = ui_settings_.ui_scale;
  float font_scale = ui_settings_.font_scale;
  float layout_scale = ui_settings_.layout_scale;
  bool vsync_enabled = ui_settings_.vsync_enabled;
  bool frame_limit_enabled = ui_settings_.frame_limit_enabled;
  bool changed = false;
  bool restart_required = false;

  if (ImGui::SliderFloat(TR("ui.settings.ui_scale", "UI Scale"), &ui_scale,
                         0.75f, 1.5f, "%.2f")) {
    ui_settings_.ui_scale = ui_scale;
    changed = true;
    restart_required = true;
  }
  if (ImGui::SliderFloat(TR("ui.settings.font_scale", "Font Scale"), &font_scale,
                         0.75f, 1.5f, "%.2f")) {
    ui_settings_.font_scale = font_scale;
    changed = true;
    restart_required = true;
  }
  if (ImGui::SliderFloat(TR("ui.settings.layout_scale", "Layout Scale"),
                         &layout_scale, 0.75f, 1.5f, "%.2f")) {
    ui_settings_.layout_scale = layout_scale;
    changed = true;
    restart_required = true;
  }
  if (ImGui::Checkbox(TR("ui.settings.vsync", "VSync"), &vsync_enabled)) {
    ui_settings_.vsync_enabled = vsync_enabled;
    glfwSwapInterval(vsync_enabled ? 1 : 0);
    render_time_initialized_ = false;
    SetHighPrecisionTimer(ui_settings_.frame_limit_enabled &&
                              !ui_settings_.vsync_enabled,
                          &high_precision_timer_active_);
    changed = true;
  }
  if (ImGui::Checkbox(TR("ui.settings.frame_limit",
                         "Frame Limit (Monitor)"),
                      &frame_limit_enabled)) {
    ui_settings_.frame_limit_enabled = frame_limit_enabled;
    render_time_initialized_ = false;
    SetHighPrecisionTimer(ui_settings_.frame_limit_enabled &&
                              !ui_settings_.vsync_enabled,
                          &high_precision_timer_active_);
    changed = true;
  }
  if (monitor_refresh_rate_ > 1.0) {
    char refresh_buf[64] = {0};
    FormatString(refresh_buf, sizeof(refresh_buf),
                 "ui.settings.refresh_rate_fmt",
                 "Monitor Refresh: %d Hz",
                 static_cast<int>(monitor_refresh_rate_ + 0.5));
    ImGui::TextDisabled("%s", refresh_buf);
  }
  char resolution_buf[96] = {0};
  FormatString(resolution_buf, sizeof(resolution_buf),
               "ui.settings.auto_resolution_scale_fmt",
               "Auto Resolution Scale: %.2fx (1920x1080 base)",
               GetResolutionScale());
  ImGui::TextDisabled("%s", resolution_buf);

  if (ImGui::Button(TR("ui.settings.reset_defaults", "Reset Defaults"))) {
    SetDefaultUiSettings(&ui_settings_);
    SaveUiSettings(ui_settings_);
    ui_settings_.restart_required = true;
    changed = false;
    glfwSwapInterval(ui_settings_.vsync_enabled ? 1 : 0);
    render_time_initialized_ = false;
    SetHighPrecisionTimer(ui_settings_.frame_limit_enabled &&
                              !ui_settings_.vsync_enabled,
                          &high_precision_timer_active_);
  }

  if (changed) {
    SaveUiSettings(ui_settings_);
    if (restart_required) {
      MarkUiSettingsRestartRequired(&ui_settings_);
    }
  }

  if (ui_settings_.restart_required) {
    ImGui::TextDisabled("%s", TR("ui.settings.restart_required",
                                 "Restart required to apply UI changes."));
    if (ImGui::Button(TR("ui.settings.restart_now", "Restart now..."))) {
      show_restart_popup_ = true;
    }
  }

  ImGui::EndMenu();
}

void Application::RenderHeader() {
  const float layout_scale = GetLayoutScale();
  ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(1.0f, 1.0f, 1.0f, 1.0f));

  ImGui::PushStyleVar(ImGuiStyleVar_ChildBorderSize, 1.0f);

  if (ImGui::BeginChild("Header", ImVec2(0, 80 * layout_scale), true,
                        ImGuiWindowFlags_NoScrollbar)) {
    {
      ImVec2 window_min = ImGui::GetWindowPos();
      ImVec2 window_size = ImGui::GetWindowSize();
      ImVec2 window_max(window_min.x + window_size.x, window_min.y + window_size.y);
      RegisterOverlayInputCaptureRect(
          window_min, window_max,
          ImGui::IsWindowHovered(ImGuiHoveredFlags_RootAndChildWindows) ||
              ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows));
    }
    ImGui::Columns(3, "HeaderColumns", false);

    ImGui::SetColumnWidth(0, 300 * layout_scale);

    ImGui::SetCursorPos(ImVec2(20 * layout_scale, 25 * layout_scale));

    ImGui::PushStyleColor(ImGuiCol_Text, ImVec4(0.1f, 0.1f, 0.1f, 1.0f));

    ImGui::Text("%s", TR("ui.header.title", "Audio Circuit Simulator"));

    ImGui::PopStyleColor();

    ImGui::NextColumn();

    float centerX =
        ImGui::GetCursorPosX() + ImGui::GetColumnWidth() / 2 -
        115 * layout_scale;

    ImGui::SetCursorPosX(centerX);

    ImGui::SetCursorPosY(25 * layout_scale);

    bool isWiringMode = (current_mode_ == Mode::WIRING);

    ImGui::PushStyleColor(ImGuiCol_Button,
                          isWiringMode ? ImVec4(0.2f, 0.5f, 0.8f, 1.0f)
                                       : ImVec4(0.92f, 0.92f, 0.92f, 1.0f));

    ImGui::PushStyleColor(ImGuiCol_Text, isWiringMode
                                             ? ImVec4(1.0f, 1.0f, 1.0f, 1.0f)
                                             : ImVec4(0.2f, 0.2f, 0.2f, 1.0f));

    if (ImGui::Button(TR("ui.header.mode_wiring", "Wiring"),
                      ImVec2(100 * layout_scale, 30 * layout_scale))) {
      current_mode_ = Mode::WIRING;
    }

    ImGui::PopStyleColor(2);

    ImGui::SameLine();

    ImGui::PushStyleColor(ImGuiCol_Button,
                          !isWiringMode ? ImVec4(0.2f, 0.5f, 0.8f, 1.0f)
                                        : ImVec4(0.92f, 0.92f, 0.92f, 1.0f));

    ImGui::PushStyleColor(ImGuiCol_Text, !isWiringMode
                                             ? ImVec4(1.0f, 1.0f, 1.0f, 1.0f)
                                             : ImVec4(0.2f, 0.2f, 0.2f, 1.0f));

    if (ImGui::Button(TR("ui.header.mode_programming", "Programming"),
                      ImVec2(120 * layout_scale, 30 * layout_scale))) {
      current_mode_ = Mode::PROGRAMMING;
    }

    ImGui::PopStyleColor(2);

    ImGui::NextColumn();

    const float right_column_x = ImGui::GetCursorPosX();
    const float right_column_width = ImGui::GetColumnWidth();
    const float header_padding = 12.0f * layout_scale;
    const AudioStreamBridgeStatus bridge_status =
        GetAudioStreamBridgeStatus();
    const bool engine_loading =
        is_plc_running_ &&
        (bridge_status.starting ||
         (!bridge_status.running && bridge_status.error.empty()) ||
         (bridge_status.running && bridge_status.captured_frames > 0 &&
          bridge_status.processed_frames == 0));
    const char* run_button_label =
        engine_loading
            ? TR("ui.header.btn_loading", "LOADING...")
            : (is_plc_running_ ? TR("ui.header.btn_stop", "STOP")
                               : TR("ui.header.btn_run", "RUN"));
    const float run_button_width = std::max(
        80.0f * layout_scale,
        ImGui::CalcTextSize(run_button_label).x +
            ImGui::GetStyle().FramePadding.x * 2.0f);
    const float run_button_height = 30.0f * layout_scale;
    const float run_button_x = std::max(
        header_padding,
        right_column_x + right_column_width - header_padding -
            run_button_width);

    ImVec4 statusColor =
        engine_loading
            ? ImVec4(0.90f, 0.58f, 0.10f, 1.0f)
            : (is_plc_running_ ? ImVec4(0.2f, 0.7f, 0.2f, 1.0f)
                               : ImVec4(0.8f, 0.2f, 0.2f, 1.0f));

    const AudioCircuitStatus& audio_status = GetAudioCircuitStatus();
    char audio_status_text[64] = {0};
    if (is_plc_running_) {
      std::snprintf(audio_status_text, sizeof(audio_status_text),
                    "[L %.1f%% | R %.1f%%]",
                    audio_status.applied_left_scalar * 100.0f,
                    audio_status.applied_right_scalar * 100.0f);
    }
    const char* statusText = is_plc_running_
                                 ? audio_status_text
                                 : TR("ui.header.status_stop", "[STOP]");
    const float status_gap = ImGui::GetStyle().ItemSpacing.x;
    const float status_width = ImGui::CalcTextSize(statusText).x;
    const float status_x = run_button_x - status_gap - status_width;

    ImGui::SetCursorPos(
        ImVec2(status_x, 31.0f * layout_scale));

    ImGui::PushStyleColor(ImGuiCol_Text, statusColor);

    ImGui::TextUnformatted(statusText);

    ImGui::PopStyleColor();

    ImGui::SetCursorPos(
        ImVec2(run_button_x, 25.0f * layout_scale));

    if (ImGui::Button(run_button_label,
                      ImVec2(run_button_width, run_button_height))) {
      is_plc_running_ = !is_plc_running_;
      std::cout << (is_plc_running_
                        ? "[Audio] Physical speaker output engaged."
                        : "[Audio] Output disengaged; restoring volume.")
                << std::endl;
    }

    ImGui::Columns(1);
  }

  ImGui::EndChild();

  ImGui::PopStyleVar();

  ImGui::PopStyleColor();
}

void Application::RenderToolbar() {
  const float layout_scale = GetLayoutScale();
  ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.96f, 0.96f, 0.96f, 1.0f));

  if (ImGui::BeginChild("Toolbar", ImVec2(0, 60 * layout_scale), true,
                        ImGuiWindowFlags_NoScrollbar)) {
    {
      ImVec2 window_min = ImGui::GetWindowPos();
      ImVec2 window_size = ImGui::GetWindowSize();
      ImVec2 window_max(window_min.x + window_size.x, window_min.y + window_size.y);
      RegisterOverlayInputCaptureRect(
          window_min, window_max,
          ImGui::IsWindowHovered(ImGuiHoveredFlags_RootAndChildWindows) ||
              ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows));
    }
    ImGui::SetCursorPos(ImVec2(20 * layout_scale, 15 * layout_scale));

    const char* toolNames[] = {
        TR("ui.toolbar.tool_select", "Select"),
        TR("ui.toolbar.tool_electric", "Electric"),
        TR("ui.toolbar.tool_tag", "Tag"),
    };

    const ToolType toolTypes[] = {ToolType::SELECT, ToolType::ELECTRIC,
                                  ToolType::TAG};

    for (int i = 0; i < 3; i++) {
      if (i > 0) {
        ImGui::SameLine();
        ImGui::SetCursorPosX(ImGui::GetCursorPosX() + 10);
      }

      bool is_selected = (current_tool_ == toolTypes[i]);

      ImGui::PushStyleColor(ImGuiCol_Button,
                            is_selected ? ImVec4(0.2f, 0.5f, 0.8f, 1.0f)
                                        : ImVec4(1.0f, 1.0f, 1.0f, 1.0f));

      ImGui::PushStyleColor(ImGuiCol_Text,
                            is_selected ? ImVec4(1.0f, 1.0f, 1.0f, 1.0f)
                                        : ImVec4(0.2f, 0.2f, 0.2f, 1.0f));

      ImVec2 button_size(180 * layout_scale, 30 * layout_scale);
      if (toolTypes[i] == ToolType::TAG) {
        if (ImGui::Button(toolNames[i], button_size)) {
          current_tool_ = toolTypes[i];
        }
      } else {
        if (ImGui::Button(toolNames[i], button_size)) {
          current_tool_ = toolTypes[i];
        }
      }

      ImGui::PopStyleColor(2);
    }

    ImGui::SameLine();

    const float wiring_button_width = 140.0f * layout_scale;
    const float wiring_button_height = 30.0f * layout_scale;
    const float wiring_button_spacing = 10.0f * layout_scale;
    const float wiring_total_width =
        wiring_button_width * 2 + wiring_button_spacing;
    float wiring_right_start =
        ImGui::GetWindowWidth() - wiring_total_width - 20.0f * layout_scale;
    if (wiring_right_start < ImGui::GetCursorPosX()) {
      wiring_right_start = ImGui::GetCursorPosX();
    }
    ImGui::SetCursorPosX(wiring_right_start);

    if (ImGui::Button(TR("ui.toolbar.save_layout", "Save"),
                      ImVec2(wiring_button_width, wiring_button_height))) {
#ifdef _WIN32
      OPENFILENAMEA ofn;
      CHAR szFile[260] = "audio_circuit.acproj";
      ZeroMemory(&ofn, sizeof(ofn));
      ofn.lStructSize = sizeof(ofn);
      ofn.hwndOwner = glfwGetWin32Window(window_);
      ofn.lpstrFile = szFile;
      ofn.nMaxFile = sizeof(szFile);
      ofn.lpstrFilter =
          "Audio Circuit Project (*.acproj)\0*.acproj\0All Files (*.*)\0*.*\0";
      ofn.nFilterIndex = 1;
      ofn.lpstrDefExt = "acproj";
      ofn.lpstrFileTitle = NULL;
      ofn.nMaxFileTitle = 0;
      ofn.lpstrInitialDir = NULL;
      ofn.Flags = OFN_OVERWRITEPROMPT | OFN_PATHMUSTEXIST | OFN_NOCHANGEDIR;
      if (GetSaveFileNameA(&ofn) == TRUE) {
        std::string save_path = ofn.lpstrFile;
        if (save_path.size() < 7 ||
            save_path.substr(save_path.size() - 7) != ".acproj") {
          save_path += ".acproj";
        }
        bool success = SaveProjectPackage(save_path, "Audio Circuit Project");
        if (success) {
          std::cout << "[INFO] Project saved: " << save_path << std::endl;
        } else {
          std::cout << "[ERROR] Project save failed: " << save_path
                    << std::endl;
        }
      }
#else
      bool success =
          SaveProjectPackage("audio_circuit.acproj", "Audio Circuit Project");
      if (success) {
        std::cout << "[INFO] Project saved: audio_circuit.acproj" << std::endl;
      } else {
        std::cout << "[ERROR] Project save failed: audio_circuit.acproj"
                  << std::endl;
      }
#endif
    }

    ImGui::SameLine();

    if (ImGui::Button(TR("ui.toolbar.load_layout", "Load"),
                      ImVec2(wiring_button_width, wiring_button_height))) {
#ifdef _WIN32
      OPENFILENAMEA ofn;
      CHAR szFile[260] = {0};
      ZeroMemory(&ofn, sizeof(ofn));
      ofn.lStructSize = sizeof(ofn);
      ofn.hwndOwner = glfwGetWin32Window(window_);
      ofn.lpstrFile = szFile;
      ofn.nMaxFile = sizeof(szFile);
      ofn.lpstrFilter =
          "Audio Circuit Project (*.acproj)\0*.acproj\0All Files (*.*)\0*.*\0";
      ofn.nFilterIndex = 1;
      ofn.lpstrDefExt = "acproj";
      ofn.lpstrFileTitle = NULL;
      ofn.nMaxFileTitle = 0;
      ofn.lpstrInitialDir = NULL;
      ofn.Flags = OFN_PATHMUSTEXIST | OFN_FILEMUSTEXIST | OFN_NOCHANGEDIR;
      if (GetOpenFileNameA(&ofn) == TRUE) {
        bool success = LoadProjectPackage(ofn.lpstrFile);
        if (success) {
          std::cout << "[INFO] Project loaded: " << ofn.lpstrFile << std::endl;
        } else {
          std::cout << "[ERROR] Project load failed: " << ofn.lpstrFile
                    << std::endl;
        }
      }
#else
      bool success = LoadProjectPackage("audio_circuit.acproj");
      if (success) {
        std::cout << "[INFO] Project loaded: audio_circuit.acproj" << std::endl;
      } else {
        std::cout << "[ERROR] Project load failed: audio_circuit.acproj"
                  << std::endl;
      }
#endif
    }
  }

  ImGui::EndChild();

  ImGui::PopStyleColor();
}

void Application::RenderMainArea() {
  const float layout_scale = GetLayoutScale();
  const float status_bar_height = 25.0f * layout_scale;
  ImVec2 available = ImGui::GetContentRegionAvail();
  float main_area_height = available.y - status_bar_height;
  if (main_area_height < 0.0f)
    main_area_height = 0.0f;

  if (ImGui::BeginChild("WiringMainArea", ImVec2(0, main_area_height), false,
                        ImGuiWindowFlags_NoScrollbar |
                            ImGuiWindowFlags_NoScrollWithMouse)) {
    ImGui::Columns(2, "MainColumns", true);

    ImGui::SetColumnWidth(0, ImGui::GetWindowWidth() * 0.75f);

    RenderWiringCanvas();

    ImGui::NextColumn();

    RenderComponentList();

    ImGui::Columns(1);
  }
  ImGui::EndChild();

  ImGui::PushStyleColor(ImGuiCol_ChildBg, ImVec4(0.94f, 0.94f, 0.94f, 1.0f));

  if (ImGui::BeginChild("StatusBar", ImVec2(0, status_bar_height), true,
                        ImGuiWindowFlags_NoScrollbar)) {
    {
      ImVec2 window_min = ImGui::GetWindowPos();
      ImVec2 window_size = ImGui::GetWindowSize();
      ImVec2 window_max(window_min.x + window_size.x, window_min.y + window_size.y);
      RegisterOverlayInputCaptureRect(
          window_min, window_max,
          ImGui::IsWindowHovered(ImGuiHoveredFlags_RootAndChildWindows) ||
              ImGui::IsWindowFocused(ImGuiFocusedFlags_RootAndChildWindows));
    }
    ImGui::SetCursorPosY(5 * layout_scale);
    ImGui::SetCursorPosX(10 * layout_scale);

    char status_buf[256] = {0};
    FormatString(status_buf, sizeof(status_buf), "ui.status.zoom_info",
                 "Zoom %.1fx | Pos: (%.0f, %.0f) | Components: %zu | Wires: %zu",
                 camera_zoom_, camera_offset_.x, camera_offset_.y,
                 placed_components_.size(), wires_.size());
    ImGui::TextUnformatted(status_buf);

    ImGui::SameLine();

    ImGui::SetCursorPosX(ImGui::GetWindowWidth() - 200 * layout_scale);

    const char* tool_name = "";

    switch (current_tool_) {
      case ToolType::SELECT:
        tool_name = "Select";
        break;
      case ToolType::TAG:
        tool_name = "Tag";
        break;
      case ToolType::PNEUMATIC:
        tool_name = "Pneumatic";
        break;
      case ToolType::ELECTRIC:
        tool_name = "Electric";
        break;
    }

    char tool_buf[128] = {0};
    FormatString(tool_buf, sizeof(tool_buf), "ui.status.tool_fmt", "Tool: %s",
                 tool_name);
    ImGui::TextUnformatted(tool_buf);
  }

  ImGui::EndChild();

  ImGui::PopStyleColor();
}

void Application::RenderShortcutHelpDialog() {
  const char* popup_id = "Shortcut Guide";
  static std::string secret_buffer;
  constexpr const char* kSecret = "ironhero";
  if (show_shortcut_help_popup_) {
    ImGui::OpenPopup(popup_id);
    show_shortcut_help_popup_ = false;
    secret_buffer.clear();
  }

  bool popup_open = true;
  if (ImGui::BeginPopupModal(popup_id, &popup_open,
                             ImGuiWindowFlags_AlwaysAutoResize)) {
    {
      ImVec2 window_min = ImGui::GetWindowPos();
      ImVec2 window_size = ImGui::GetWindowSize();
      ImVec2 window_max(window_min.x + window_size.x, window_min.y + window_size.y);
      RegisterOverlayInputCaptureRect(window_min, window_max, true);
    }
    ImGui::TextUnformatted(
        TR("ui.help.shortcuts.title", "Shortcut Guide"));
    ImGui::Separator();
    ImGui::TextUnformatted(
        TR("ui.help.shortcuts.open_help", "Open this help"));
    ImGui::BulletText("%s",
                      TR("ui.help.shortcuts.open_1", "Ctrl + ?"));
    ImGui::BulletText("%s",
                      TR("ui.help.shortcuts.open_2", "Ctrl + /"));

    ImGui::Spacing();
    ImGui::TextUnformatted(TR("ui.help.shortcuts.common", "Common"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.common_run_stop",
                               "RUN/STOP: Use the header button"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.common_switch_mode",
                               "Switch mode: Wiring / Programming buttons"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.common_save_load",
                               "Mode toolbars: Wiring JSON / Programming CSV"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.common_save_project",
                               "Ctrl+S: Save project package (.acproj)"));

    ImGui::Spacing();
    ImGui::TextUnformatted(
        TR("ui.help.shortcuts.quick_start", "Quick Start"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.quick_wiring_drag",
                               "Drag components from the list to the canvas"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.quick_wiring_connect",
                               "Connect ports by dragging from one port to another"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.quick_tools",
                               "Tool buttons: Select / Pneumatic / Electric / Tag"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.quick_tool_select",
                               "Select: select/move components and wires"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.quick_tool_pneumatic",
                               "Pneumatic: create pneumatic wires"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.quick_tool_electric",
                               "Electric: create electrical wires"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.quick_tool_tag",
                               "Tag: add/edit wire tags"));

    ImGui::Spacing();
    ImGui::TextUnformatted(
        TR("ui.help.shortcuts.wiring_mode", "Wiring mode"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.wiring_rotate",
                               "Rotate component: R (Shift+R: reverse)"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.wiring_delete",
                               "Delete selected: Delete"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.wiring_select_toggle",
                               "Select tool: Ctrl+Click toggles components/wires"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.wiring_select_box",
                               "Select tool: Drag empty canvas to box-select"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.wiring_select_box_add",
                               "Select tool: Ctrl+Drag empty canvas adds/removes by box"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.wiring_tool_cycle",
                               "Q: cycle tools (Select -> Pneumatic -> Electric -> Tag)"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.wiring_tag",
                               "Tag tool: click wire to add/edit wire tag"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.wiring_zoom",
                               "Zoom: Mouse wheel / Trackpad pinch / Touch pinch"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.wiring_pan",
                               "Pan: Middle drag / Alt + Right drag / Shift + Wheel (horizontal)"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.wiring_trackpad_pan",
                               "Trackpad: Two-finger scroll pans / Touch: drag pans"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.wiring_source_sink",
                               "PLC context menu: choose Sink/Source input and output modes"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.wiring_z_order",
                               "Z-order: use component context menu"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.wiring_z_order_items",
                               "Bring to Front / Send to Back / Bring Forward / Send Backward"));

    ImGui::Spacing();
    ImGui::TextUnformatted(
        TR("ui.help.shortcuts.programming_mode", "Programming mode"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.prog_f2_f3",
                               "F2/F3: Monitor/Edit"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.prog_f5_f6_f7",
                               "F5/F6/F7: XIC/XIO/Coil"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.prog_f9",
                               "F9 / Shift+F9: Add/Remove vertical"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.prog_edit_keys",
                               "Delete / Insert / Enter"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.prog_undo_redo",
                               "Ctrl+Z / Ctrl+Y: Undo/Redo"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.prog_ctrl_arrow",
                               "Ctrl+Arrow: Toggle line path"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.prog_addr_example",
                               "Address example: Y0, T1 K10, C2 K5, SET M0, RST Y0"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.prog_compile",
                               "Use Save / Load / Compile buttons on the top bar"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.prog_save_project",
                               "Ctrl+S: Save project package (.acproj)"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.prog_monitor",
                               "Monitor mode: inspect X/Y/M/T/C states in real time"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.prog_rst_bkrst",
                               "RST resets one device, BKRST resets a device range"));

    ImGui::Spacing();
    ImGui::TextUnformatted(TR("ui.help.shortcuts.rtl_mode", "RTL"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.rtl_open",
                               "RTL menu: open RTL Library and Toolchain"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.rtl_runtime",
                               "RTL Runtime menu: configure logic family, power, clock, and reset pins"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.rtl_import_export",
                               "RTL Library: import/export reusable .plccomp component packages"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.rtl_runtime_package",
                               "Source-less RTL packages run from the bundled runtime and cannot be edited"));
    ImGui::BulletText("%s", TR("ui.help.shortcuts.rtl_logs",
                               "Analyze, build, and testbench results are shown in the RTL Library log panel"));

    // Hidden easter egg: type "ironhero" while this popup is open.
    ImGuiIO& io = ImGui::GetIO();
    if (ImGui::IsKeyPressed(ImGuiKey_Backspace, false) &&
        !secret_buffer.empty()) {
      secret_buffer.pop_back();
    }
    for (int i = 0; i < io.InputQueueCharacters.Size; ++i) {
      ImWchar wc = io.InputQueueCharacters[i];
      if (wc <= 127 &&
          std::isalpha(static_cast<unsigned char>(wc))) {
        secret_buffer.push_back(
            static_cast<char>(std::tolower(static_cast<unsigned char>(wc))));
      }
    }
    if (secret_buffer.size() > 64) {
      secret_buffer.erase(0, secret_buffer.size() - 64);
    }
    if (secret_buffer.size() >= std::strlen(kSecret)) {
      std::string tail =
          secret_buffer.substr(secret_buffer.size() - std::strlen(kSecret));
      if (EqualsIgnoreCase(tail.c_str(), kSecret)) {
        OpenExternalUrl("https://github.com/ironhero1544");
        secret_buffer.clear();
        ImGui::CloseCurrentPopup();
      }
    }

    ImGui::Spacing();
    if (ImGui::Button(TR("ui.help.shortcuts.close", "Close"),
                      ImVec2(120.0f, 0.0f))) {
      ImGui::CloseCurrentPopup();
    }
    ImGui::EndPopup();
  }
}

}  // namespace plc
