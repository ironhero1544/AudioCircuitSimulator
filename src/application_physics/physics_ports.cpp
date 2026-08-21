// physics_ports.cpp
//
// Port mapping helpers for the physics layer.

#include "plc_emulator/core/application.h"

#include <utility>
#include <vector>

namespace plc {

std::vector<std::pair<int, bool>> Application::GetPortsForComponent(
    const PlacedComponent& comp) {
  std::vector<std::pair<int, bool>> ports;
  int max_ports = 0;
  switch (comp.type) {
    case ComponentType::PLC:
      max_ports = 32;
      break;
    case ComponentType::FRL:
      max_ports = 1;
      break;
    case ComponentType::MANIFOLD:
      max_ports = 5;
      break;
    case ComponentType::LIMIT_SWITCH:
      max_ports = 3;
      break;
    case ComponentType::SENSOR:
      max_ports = 3;
      break;
    case ComponentType::CYLINDER:
      max_ports = 2;
      break;
    case ComponentType::VALVE_SINGLE:
      max_ports = 5;
      break;
    case ComponentType::VALVE_DOUBLE:
      max_ports = 7;
      break;
    case ComponentType::BUTTON_UNIT:
      max_ports = 15;
      break;
    case ComponentType::POWER_SUPPLY:
      max_ports = 2;
      break;
    case ComponentType::WORKPIECE_METAL:
    case ComponentType::WORKPIECE_NONMETAL:
      max_ports = 0;
      break;
    case ComponentType::RING_SENSOR:
      max_ports = 3;
      break;
    case ComponentType::METER_VALVE:
      max_ports = 2;
      break;
    case ComponentType::INDUCTIVE_SENSOR:
      max_ports = 3;
      break;
    case ComponentType::CONVEYOR:
      max_ports = 2;
      break;
    case ComponentType::PROCESSING_CYLINDER:
      max_ports = 7;
      break;
    case ComponentType::BOX:
      max_ports = 0;
      break;
    case ComponentType::TOWER_LAMP:
      max_ports = 4;
      break;
    case ComponentType::EMERGENCY_STOP:
      max_ports = 4;
      break;
    case ComponentType::RTL_MODULE:
      for (const auto& port : comp.runtimePorts) {
        ports.push_back({port.id, port.isInput});
      }
      return ports;
    case ComponentType::AUDIO_ADC:
    case ComponentType::AUDIO_RESISTOR:
    case ComponentType::AUDIO_CAPACITOR:
    case ComponentType::AUDIO_INDUCTOR:
    case ComponentType::AUDIO_DAC:
    case ComponentType::AUDIO_DC_SOURCE:
    case ComponentType::AUDIO_AC_SOURCE:
    case ComponentType::AUDIO_PULSE_SOURCE:
      max_ports = 2;
      break;
    case ComponentType::AUDIO_DIODE:
      max_ports = 2;
      break;
    case ComponentType::AUDIO_BJT_NPN:
    case ComponentType::AUDIO_BJT_PNP:
      max_ports = 3;
      break;
    case ComponentType::AUDIO_OP_AMP:
      max_ports = 5;
      break;
    case ComponentType::AUDIO_SOURCE:
      max_ports = 3;
      break;
    case ComponentType::AUDIO_POTENTIOMETER:
      max_ports = 3;
      break;
    case ComponentType::AUDIO_SPEAKER:
      max_ports = 4;
      break;
    case ComponentType::AUDIO_GROUND:
      max_ports = 1;
      break;
    default:
      max_ports = 0;
      break;
  }
  for (int i = 0; i < max_ports; ++i) {
    ports.push_back({i, true});
  }
  return ports;
}


}  // namespace plc
