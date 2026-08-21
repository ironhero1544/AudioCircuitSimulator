#ifndef PLC_EMULATOR_INCLUDE_PLC_EMULATOR_RTL_RTL_RUNTIME_MANAGER_H_
#define PLC_EMULATOR_INCLUDE_PLC_EMULATOR_RTL_RTL_RUNTIME_MANAGER_H_

#include "plc_emulator/core/data_types.h"
#include "plc_emulator/physics/component_physics_adapter.h"
#include "plc_emulator/rtl/rtl_project_manager.h"

#include <map>
#include <memory>
#include <mutex>
#include <set>
#include <string>
#include <cstdint>

// PLC_RTL_WSL_BACKEND: Windows + WSL2 기반 Verilator 런타임 백엔드.
// CMakeLists.txt의 target_compile_definitions 에서 활성화됩니다.
// 미정의 시 RTL 컴포넌트는 로드되지만 시뮬레이션은 비활성화됩니다.

namespace plc {

class RtlRuntimeManager {
 public:
  RtlRuntimeManager();
  ~RtlRuntimeManager();

  void SetProjectManager(const RtlProjectManager* project_manager);
  void SyncComponentInstances(const std::vector<PlacedComponent>& components);
  void InvalidateModule(const std::string& module_id);
  void ShutdownAll();

  bool EvaluateComponent(const PlacedComponent& comp,
                         const std::map<PortRef, float>& voltages,
                         std::map<int, RtlLogicValue>* output_values,
                         std::string* diagnostics);
  bool ProcessAudioDacBlock(const PlacedComponent& comp,
                            float* interleaved_stereo,
                            size_t frame_count,
                            std::string* diagnostics);
  void ConfigureAudioDac(const PlacedComponent& comp);
  bool WarmupAudioDac(std::string* diagnostics);
  void ResetAudioPipeline();
  bool SubmitDacFrames(std::uint64_t sequence,
                       const float* interleaved_stereo,
                       size_t frame_count);
  bool TryReceiveDacCodes(std::uint64_t sequence,
                          float* interleaved_stereo,
                          size_t frame_capacity,
                          size_t* received_frames);
  // Compatibility wrappers for callers that do not manage block sequence.
  bool SubmitDacFrames(const float* interleaved_stereo, size_t frame_count);
  bool TryReceiveDacCodes(float* interleaved_stereo, size_t frame_capacity,
                          size_t* received_frames);
  std::string GetAudioDacDiagnostics() const;
  bool AudioDacWorkerHealthy() const;

 private:
  struct WorkerProcess;
  struct AudioAsyncState;

  const RtlProjectManager* project_manager_ = nullptr;
  std::map<int, WorkerProcess> processes_;
  std::mutex processes_mutex_;
  std::unique_ptr<AudioAsyncState> audio_async_;

  bool EnsureProcessStarted(const PlacedComponent& comp,
                            std::string* diagnostics);
  bool SendEvalRequest(WorkerProcess* process,
                       const PlacedComponent& comp,
                       const std::map<PortRef, float>& voltages,
                       std::map<int, RtlLogicValue>* output_values,
                       std::string* diagnostics);
  static std::string BuildEvalCommand(
      const PlacedComponent& comp,
      const std::map<PortRef, float>& voltages);
};

}  // namespace plc

#endif  // PLC_EMULATOR_INCLUDE_PLC_EMULATOR_RTL_RTL_RUNTIME_MANAGER_H_
