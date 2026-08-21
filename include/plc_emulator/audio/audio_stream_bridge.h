#ifndef PLC_EMULATOR_AUDIO_AUDIO_STREAM_BRIDGE_H_
#define PLC_EMULATOR_AUDIO_AUDIO_STREAM_BRIDGE_H_

#include <string>
#include <cstdint>

namespace plc {

enum class AudioBridgeSilenceReason : int {
  NONE = 0,
  RTL_WAIT,
  MNA_FAILURE,
  RENDER_STARVATION,
  PHYSICAL_ZERO_OUTPUT,
};

struct AudioStreamBridgeStatus {
  bool starting = false;
  bool running = false;
  bool windows_routing_active = false;
  std::uint64_t captured_frames = 0;
  std::uint64_t processed_frames = 0;
  std::uint64_t underrun_frames = 0;
  std::uint64_t dropped_frames = 0;
  std::uint64_t rtl_valid_frames = 0;
  std::uint64_t mna_valid_frames = 0;
  std::uint64_t rendered_frames = 0;
  std::uint64_t concealed_frames = 0;
  std::uint64_t capture_overflow_frames = 0;
  std::uint64_t capture_discontinuity_frames = 0;
  std::uint64_t rtl_sequence_loss_frames = 0;
  std::uint32_t capture_queued_frames = 0;
  std::uint32_t queued_frames = 0;
  std::uint32_t shared_period_frames = 0;
  int buffer_ms = 0;
  float rtl_time_ms = 0.0f;
  float rtl_p99_ms = 0.0f;
  float mna_time_ms = 0.0f;
  float mna_p99_ms = 0.0f;
  float processing_time_ms = 0.0f;
  float processing_p99_ms = 0.0f;
  float render_time_ms = 0.0f;
  float render_p99_ms = 0.0f;
  float input_rms = 0.0f;
  float output_rms = 0.0f;
  float render_pcm_rms = 0.0f;
  float output_endpoint_volume = 0.0f;
  bool output_endpoint_muted = false;
  AudioBridgeSilenceReason silence_reason = AudioBridgeSilenceReason::NONE;
  std::string error;
};

void UpdateAudioStreamBridge(bool enabled,
                             const std::string& input_device_id,
                             const std::string& output_device_id,
                             int buffer_ms,
                             float maximum_volume_percent);
AudioStreamBridgeStatus GetAudioStreamBridgeStatus();
void ShutdownAudioStreamBridge();

}  // namespace plc

#endif
