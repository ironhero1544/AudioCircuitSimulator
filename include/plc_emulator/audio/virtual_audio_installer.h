#ifndef PLC_EMULATOR_AUDIO_VIRTUAL_AUDIO_INSTALLER_H_
#define PLC_EMULATOR_AUDIO_VIRTUAL_AUDIO_INSTALLER_H_

#include "plc_emulator/rtl/audio_rtl_wsl_toolchain.h"

namespace plc {

bool IsVirtualAudioDriverInstalled();
AudioRtlProcessResult InstallVirtualAudioDriver();
AudioRtlProcessResult RemoveVirtualAudioDriver();

}  // namespace plc

#endif
