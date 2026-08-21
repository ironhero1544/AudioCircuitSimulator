#ifndef PLC_EMULATOR_COMPONENTS_AUDIO_COMPONENTS_DEF_H_
#define PLC_EMULATOR_COMPONENTS_AUDIO_COMPONENTS_DEF_H_

#include "plc_emulator/components/component_definition.h"

#include <string>

namespace plc {
const ComponentDefinition* GetAudioSourceDefinition();
const ComponentDefinition* GetAudioAdcDefinition();
const ComponentDefinition* GetAudioResistorDefinition();
const ComponentDefinition* GetAudioPotentiometerDefinition();
const ComponentDefinition* GetAudioCapacitorDefinition();
const ComponentDefinition* GetAudioInductorDefinition();
const ComponentDefinition* GetAudioDiodeDefinition();
const ComponentDefinition* GetAudioBjtNpnDefinition();
const ComponentDefinition* GetAudioBjtPnpDefinition();
const ComponentDefinition* GetAudioOpAmpDefinition();
const ComponentDefinition* GetAudioDacDefinition();
const ComponentDefinition* GetAudioSpeakerDefinition();
const ComponentDefinition* GetAudioGroundDefinition();
const ComponentDefinition* GetAudioDcSourceDefinition();
const ComponentDefinition* GetAudioAcSourceDefinition();
const ComponentDefinition* GetAudioPulseSourceDefinition();
const ComponentDefinition* GetAudioRtlShellAppearance(
    const std::string& module_id);
}  // namespace plc

#endif
