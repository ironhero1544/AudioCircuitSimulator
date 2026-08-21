#ifndef PLC_EMULATOR_RTL_AUDIO_RTL_WSL_TOOLCHAIN_H_
#define PLC_EMULATOR_RTL_AUDIO_RTL_WSL_TOOLCHAIN_H_

#include <cstdint>
#include <optional>
#include <string>
#include <utility>
#include <vector>

namespace plc {

// Structured invocation of one Linux program inside WSL. Arguments are kept
// separate so HDL paths and module names never become shell source.
struct AudioRtlWslCommand {
  std::optional<std::wstring> distribution;
  std::optional<std::wstring> working_directory;
  std::wstring program;
  std::vector<std::wstring> arguments;
  std::vector<std::pair<std::wstring, std::wstring>> environment;
  bool run_as_root = false;
};

struct AudioRtlProcessResult {
  bool started = false;
  bool timed_out = false;
  std::uint32_t exit_code = 1;
  std::string output;
  std::string error;

  [[nodiscard]] bool Succeeded() const {
    return started && !timed_out && exit_code == 0;
  }
};

struct AudioRtlWslStatus {
  bool wsl_found = false;
  bool verilator_found = false;
  bool compiler_found = false;
  bool make_found = false;
  std::string distribution;
  std::string verilator_version;
  std::string diagnosis;
};

class AudioRtlWslToolchain final {
 public:
  [[nodiscard]] static AudioRtlProcessResult Run(
      const AudioRtlWslCommand& command, std::uint32_t timeout_ms);
  [[nodiscard]] static AudioRtlWslStatus Detect();
  [[nodiscard]] static AudioRtlProcessResult Install();
  [[nodiscard]] static AudioRtlProcessResult InstallVerilator();
  [[nodiscard]] static AudioRtlProcessResult Remove();
  [[nodiscard]] static std::string WindowsPathToWsl(const std::string& path);
  [[nodiscard]] static std::string BuildLaunchCommand(
      const AudioRtlWslCommand& command);
};

}  // namespace plc

#endif  // PLC_EMULATOR_RTL_AUDIO_RTL_WSL_TOOLCHAIN_H_
