#include "plc_emulator/components/audio_components_def.h"

#include "imgui.h"
#include "plc_emulator/components/state_keys.h"

#include <algorithm>
#include <cfloat>
#include <cmath>
#include <cstdio>
#include <string>

namespace plc {
namespace {

constexpr Color kSignalColor{0.95f, 0.25f, 0.22f, 1.0f};
constexpr Color kReturnColor{0.12f, 0.20f, 0.27f, 1.0f};
constexpr Color kPowerColor{0.95f, 0.70f, 0.08f, 1.0f};

ImU32 ToU32(Color c) {
  return IM_COL32(static_cast<int>(c.r * 255), static_cast<int>(c.g * 255),
                  static_cast<int>(c.b * 255), static_cast<int>(c.a * 255));
}

void DrawPorts(ImDrawList* draw, const PlacedComponent& component,
               const ComponentPortDef* ports, int count, ImVec2 pos,
               float zoom) {
  if (component.type == ComponentType::RTL_MODULE) {
    for (const Port& port : component.runtimePorts) {
      ImVec2 p{pos.x + port.relativePos.x * zoom,
               pos.y + port.relativePos.y * zoom};
      draw->AddCircleFilled(p, 6.0f * zoom, ToU32(port.color));
      draw->AddCircle(p, 6.0f * zoom, IM_COL32(25, 25, 25, 255), 0,
                      1.2f * zoom);
      if (component.rtlModuleId == "audio_shell_dac" && zoom > 0.5f &&
          !port.role.empty()) {
        ImFont* font = ImGui::GetFont();
        const float font_size = 9.0f * std::min(zoom, 1.5f);
        const ImVec2 text_size =
            font->CalcTextSizeA(font_size, FLT_MAX, 0.0f, port.role.c_str());
        const float text_x =
            port.isInput ? pos.x + 9.0f * zoom
                         : pos.x + component.size.width * zoom -
                               text_size.x - 9.0f * zoom;
        draw->AddText(font, font_size,
                      {text_x, p.y - text_size.y * 0.5f},
                      IM_COL32(35, 38, 45, 255), port.role.c_str());
      }
    }
    return;
  }
  for (int i = 0; i < count; ++i) {
    const auto& port = ports[i];
    ImVec2 p{pos.x + port.rel_pos.x * zoom, pos.y + port.rel_pos.y * zoom};
    draw->AddCircleFilled(p, 6.0f * zoom, ToU32(port.color));
    draw->AddCircle(p, 6.0f * zoom, IM_COL32(25, 25, 25, 255), 0, 1.2f * zoom);
  }
}

void DrawBody(ImDrawList* draw, ImVec2 pos, Size size, float zoom,
              ImU32 fill, const char* title) {
  ImVec2 end{pos.x + size.width * zoom, pos.y + size.height * zoom};
  draw->AddRectFilled(pos, end, fill, 5.0f * zoom);
  draw->AddRect(pos, end, IM_COL32(45, 50, 55, 255), 5.0f * zoom, 0,
                2.0f * zoom);
  if (zoom > 0.42f) {
    draw->AddText(ImGui::GetFont(), 14.0f * std::min(zoom, 1.5f),
                  {pos.x + 10.0f * zoom, pos.y + 8.0f * zoom},
                  IM_COL32(25, 28, 31, 255), title);
  }
}

const ComponentPortDef kTwoPort[] = {
    {0, {0.0f, 45.0f}, PortType::ELECTRIC, true, kSignalColor, "AUDIO_IN"},
    {1, {130.0f, 45.0f}, PortType::ELECTRIC, false, kSignalColor, "AUDIO_OUT"},
};
const ComponentPortDef kPotPorts[] = {
    {0, {0.0f, 45.0f}, PortType::ELECTRIC, true, kSignalColor, "END_A"},
    {1, {130.0f, 45.0f}, PortType::ELECTRIC, false, kSignalColor, "WIPER"},
    {2, {65.0f, 95.0f}, PortType::ELECTRIC, true, kReturnColor, "END_B"},
};
const ComponentPortDef kOpAmpPorts[] = {
    {0, {0.0f, 40.0f}, PortType::ELECTRIC, true, kSignalColor, "IN-"},
    {1, {0.0f, 66.0f}, PortType::ELECTRIC, true, kSignalColor, "IN+"},
    {2, {68.0f, 0.0f}, PortType::ELECTRIC, true, kPowerColor, "V+"},
    {3, {68.0f, 105.0f}, PortType::ELECTRIC, true, kReturnColor, "V-"},
    {4, {145.0f, 53.0f}, PortType::ELECTRIC, false, kSignalColor, "OUT"},
};
const ComponentPortDef kBjtPorts[] = {
    {0, {0.0f, 53.0f}, PortType::ELECTRIC, true, kSignalColor, "BASE"},
    {1, {130.0f, 25.0f}, PortType::ELECTRIC, true, kPowerColor, "COLLECTOR"},
    {2, {130.0f, 81.0f}, PortType::ELECTRIC, false, kSignalColor, "EMITTER"},
};
const ComponentPortDef kSourcePorts[] = {
    {0, {150.0f, 28.0f}, PortType::ELECTRIC, false, kPowerColor, "I2S_BCLK"},
    {1, {150.0f, 55.0f}, PortType::ELECTRIC, false, kSignalColor, "I2S_LRCLK"},
    {2, {150.0f, 82.0f}, PortType::ELECTRIC, false, kSignalColor, "I2S_SDATA"},
};
const ComponentPortDef kSpeakerPorts[] = {
    {0, {0.0f, 28.0f}, PortType::ELECTRIC, true, kSignalColor, "LEFT_POS"},
    {1, {0.0f, 52.0f}, PortType::ELECTRIC, true, kReturnColor, "LEFT_NEG"},
    {2, {0.0f, 76.0f}, PortType::ELECTRIC, true, kSignalColor, "RIGHT_POS"},
    {3, {0.0f, 100.0f}, PortType::ELECTRIC, true, kReturnColor, "RIGHT_NEG"},
};
const ComponentPortDef kGroundPorts[] = {
    {0, {45.0f, 0.0f}, PortType::ELECTRIC, true, kReturnColor, "AUDIO_GND"},
};
const ComponentPortDef kSupplyPorts[] = {
    {0, {140.0f, 36.0f}, PortType::ELECTRIC, false, kPowerColor, "POSITIVE"},
    {1, {140.0f, 79.0f}, PortType::ELECTRIC, false, kReturnColor, "RETURN"},
};

void InitSource(PlacedComponent* c) { c->internalStates[state_keys::kAudioLevel] = 1.0f; }
void InitAdc(PlacedComponent* c) { c->internalStates[state_keys::kBitDepth] = 16.0f; }
void InitResistor(PlacedComponent* c) {
  c->internalStates[state_keys::kResistanceOhms] = 1000.0f;
  c->internalStates[state_keys::kTolerancePercent] = 1.0f;
}
void InitPot(PlacedComponent* c) {
  c->internalStates[state_keys::kResistanceOhms] = 10000.0f;
  c->internalStates[state_keys::kWiperPosition] = 0.5f;
}
void InitCap(PlacedComponent* c) { c->internalStates[state_keys::kCapacitanceUf] = 10.0f; }
void InitInductor(PlacedComponent* c) { c->internalStates[state_keys::kInductanceMh] = 10.0f; }
void InitOpAmp(PlacedComponent* c) {
  c->internalStates[state_keys::kGain] = 10.0f;
  c->internalStates[state_keys::kOpenLoopGain] = 100000.0f;
  c->internalStates[state_keys::kGainBandwidthHz] = 1000000.0f;
  c->internalStates[state_keys::kSlewRateVoltsPerUs] = 0.5f;
  c->internalStates[state_keys::kInputOffsetMillivolts] = 2.0f;
  c->internalStates[state_keys::kOutputCurrentLimitAmps] = 0.025f;
  c->internalStates[state_keys::kOutputResistanceOhms] = 50.0f;
  c->internalStates[state_keys::kOutputHeadroomVolts] = 1.0f;
}
void InitDiode(PlacedComponent* c) {
  c->internalStates[state_keys::kForwardVoltageVolts] = 0.65f;
  c->internalStates[state_keys::kDynamicResistanceOhms] = 2.0f;
  c->internalStates[state_keys::kThermalCoupling] = 0.0f;
}
void InitBjt(PlacedComponent* c) {
  c->internalStates[state_keys::kCurrentGainBeta] = 100.0f;
  c->internalStates[state_keys::kBaseEmitterVoltageVolts] = 0.65f;
  c->internalStates[state_keys::kSaturationVoltageVolts] = 0.2f;
  c->internalStates[state_keys::kMaximumCollectorCurrentAmps] = 1.5f;
  c->internalStates[state_keys::kMaximumPowerWatts] = 1.0f;
  c->internalStates[state_keys::kThermalResistanceCPerW] = 62.5f;
  c->internalStates[state_keys::kOutputResistanceOhms] = 0.2f;
}
void InitDac(PlacedComponent* c) { c->internalStates[state_keys::kBitDepth] = 16.0f; }
void InitSpeaker(PlacedComponent* c) {
  c->internalStates[state_keys::kImpedanceOhms] = 8.0f;
  c->internalStates[state_keys::kMaximumPowerWatts] = 100.0f;
  c->internalStates[state_keys::kTransducerEq1FrequencyHz] = 125.0f;
  c->internalStates[state_keys::kTransducerEq1GainDb] = 0.0f;
  c->internalStates[state_keys::kTransducerEq1Q] = 1.0f;
  c->internalStates[state_keys::kTransducerEq2FrequencyHz] = 500.0f;
  c->internalStates[state_keys::kTransducerEq2GainDb] = 0.0f;
  c->internalStates[state_keys::kTransducerEq2Q] = 1.0f;
  c->internalStates[state_keys::kTransducerEq3FrequencyHz] = 4000.0f;
  c->internalStates[state_keys::kTransducerEq3GainDb] = 0.0f;
  c->internalStates[state_keys::kTransducerEq3Q] = 1.0f;
  c->internalStates[state_keys::kTransducerEq4FrequencyHz] = 10000.0f;
  c->internalStates[state_keys::kTransducerEq4GainDb] = 0.0f;
  c->internalStates[state_keys::kTransducerEq4Q] = 1.0f;
}
void InitDcSource(PlacedComponent* c) {
  c->internalStates[state_keys::kVoltageVolts] = 5.0f;
  c->internalStates[state_keys::kCurrentLimitAmps] = 1.0f;
}
void InitAcSource(PlacedComponent* c) {
  c->internalStates[state_keys::kVoltageVolts] = 1.0f;
  c->internalStates[state_keys::kFrequencyHz] = 1000.0f;
  c->internalStates[state_keys::kFrequencyMinHz] = 20.0f;
  c->internalStates[state_keys::kFrequencyMaxHz] = 20000.0f;
  c->internalStates[state_keys::kPhaseDegrees] = 0.0f;
}
void InitPulseSource(PlacedComponent* c) {
  c->internalStates[state_keys::kPulseLowVolts] = 0.0f;
  c->internalStates[state_keys::kPulseHighVolts] = 5.0f;
  c->internalStates[state_keys::kFrequencyHz] = 1000.0f;
  c->internalStates[state_keys::kFrequencyMinHz] = 20.0f;
  c->internalStates[state_keys::kFrequencyMaxHz] = 20000.0f;
  c->internalStates[state_keys::kDutyCyclePercent] = 50.0f;
}

void RenderSource(ImDrawList* d, const PlacedComponent& c, ImVec2 p, float z) {
  DrawBody(d, p, c.size, z, IM_COL32(187, 220, 241, 255), "PC DIGITAL AUDIO");
  ImVec2 center{p.x + 70*z, p.y + 58*z};
  d->AddCircle(center, 24*z, IM_COL32(35, 76, 105, 255), 0, 2*z);
  for (int i = 0; i < 18; ++i) {
    float x0 = center.x - 18*z + i*2*z;
    float y = center.y + std::sin(i*0.9f)*9*z;
    d->AddCircleFilled({x0,y}, 1.5f*z, IM_COL32(35, 76, 105, 255));
  }
  DrawPorts(d, c, kSourcePorts,
            static_cast<int>(sizeof(kSourcePorts) / sizeof(kSourcePorts[0])),
            p, z);
}

void RenderConverter(ImDrawList* d, const PlacedComponent& c, ImVec2 p, float z,
                     const char* label) {
  DrawBody(d, p, c.size, z, IM_COL32(210, 202, 235, 255), label);
  d->AddRect({p.x+28*z,p.y+31*z},{p.x+102*z,p.y+70*z},IM_COL32(70,55,105,255),2*z,0,2*z);
  for (int i=0;i<6;++i) d->AddLine({p.x+(35+i*12)*z,p.y+31*z},{p.x+(35+i*12)*z,p.y+23*z},IM_COL32(70,55,105,255),2*z);
  DrawPorts(d, c, kTwoPort, 2, p, z);
}
void RenderAdc(ImDrawList* d,const PlacedComponent& c,ImVec2 p,float z){RenderConverter(d,c,p,z,"ADC");}
void RenderDac(ImDrawList* d,const PlacedComponent& c,ImVec2 p,float z){RenderConverter(d,c,p,z,"DAC");}

std::string FormatSmdResistanceCode(float resistance_ohms) {
  const float resistance = std::max(0.0f, resistance_ohms);
  char code[16] = {};
  if (resistance < 0.005f) {
    return "000";
  }
  if (resistance < 1.0f) {
    std::snprintf(code, sizeof(code), "R%02d",
                  static_cast<int>(std::round(resistance * 100.0f)));
    return code;
  }
  if (resistance < 10.0f) {
    const int whole = static_cast<int>(resistance);
    const int tenth = static_cast<int>(std::round(
        (resistance - static_cast<float>(whole)) * 10.0f));
    std::snprintf(code, sizeof(code), "%dR%d", whole,
                  std::clamp(tenth, 0, 9));
    return code;
  }

  int multiplier = std::max(
      0, static_cast<int>(std::floor(std::log10(resistance))) - 1);
  int significant = static_cast<int>(
      std::round(resistance / std::pow(10.0f, multiplier)));
  if (significant >= 100) {
    significant /= 10;
    ++multiplier;
  }
  std::snprintf(code, sizeof(code), "%02d%d",
                std::clamp(significant, 0, 99),
                std::clamp(multiplier, 0, 9));
  return code;
}

void DrawSmdResistor(ImDrawList* draw, const PlacedComponent& component,
                     ImVec2 position, float zoom, bool show_wiper) {
  const float resistance =
      component.internalStates.count(state_keys::kResistanceOhms)
          ? component.internalStates.at(state_keys::kResistanceOhms)
          : (show_wiper ? 10000.0f : 1000.0f);
  const std::string code = FormatSmdResistanceCode(resistance);
  const float center_y = position.y + 55.0f * zoom;

  draw->AddLine({position.x, center_y},
                {position.x + 25.0f * zoom, center_y},
                IM_COL32(92, 78, 57, 255), 3.0f * zoom);
  draw->AddLine({position.x + 105.0f * zoom, center_y},
                {position.x + 130.0f * zoom, center_y},
                IM_COL32(92, 78, 57, 255), 3.0f * zoom);
  draw->AddRectFilled({position.x + 23.0f * zoom,
                       position.y + 35.0f * zoom},
                      {position.x + 107.0f * zoom,
                       position.y + 72.0f * zoom},
                      IM_COL32(45, 47, 48, 255), 4.0f * zoom);
  draw->AddRectFilled({position.x + 23.0f * zoom,
                       position.y + 35.0f * zoom},
                      {position.x + 35.0f * zoom,
                       position.y + 72.0f * zoom},
                      IM_COL32(181, 184, 180, 255), 3.0f * zoom);
  draw->AddRectFilled({position.x + 95.0f * zoom,
                       position.y + 35.0f * zoom},
                      {position.x + 107.0f * zoom,
                       position.y + 72.0f * zoom},
                      IM_COL32(181, 184, 180, 255), 3.0f * zoom);

  const float code_font_size = 17.0f * std::min(zoom, 1.5f);
  ImVec2 code_size = ImGui::GetFont()->CalcTextSizeA(
      code_font_size, 1000.0f, 0.0f, code.c_str());
  draw->AddText(ImGui::GetFont(), code_font_size,
                {position.x + 65.0f * zoom - code_size.x * 0.5f,
                 position.y + (show_wiper ? 39.0f : 44.0f) * zoom},
                IM_COL32(242, 242, 235, 255), code.c_str());

  if (show_wiper) {
    const float wiper = std::clamp(
        component.internalStates.count(state_keys::kWiperPosition)
            ? component.internalStates.at(state_keys::kWiperPosition)
            : 0.5f,
        0.0f, 1.0f);
    char wiper_text[16] = {};
    std::snprintf(wiper_text, sizeof(wiper_text), "W%02d",
                  static_cast<int>(std::round(wiper * 100.0f)));
    const float wiper_font_size = 10.0f * std::min(zoom, 1.5f);
    ImVec2 wiper_size = ImGui::GetFont()->CalcTextSizeA(
        wiper_font_size, 1000.0f, 0.0f, wiper_text);
    draw->AddText(ImGui::GetFont(), wiper_font_size,
                  {position.x + 65.0f * zoom - wiper_size.x * 0.5f,
                   position.y + 58.0f * zoom},
                  IM_COL32(188, 198, 205, 255), wiper_text);
  }
}

void RenderResistor(ImDrawList* d, const PlacedComponent& c, ImVec2 p, float z) {
  const char* title = c.customLabel.empty() ? "RESISTOR" : c.customLabel.c_str();
  DrawBody(d,p,c.size,z,IM_COL32(239,224,181,255),title);
  DrawSmdResistor(d, c, p, z, false);
  DrawPorts(d,c,kTwoPort,2,p,z);
}
void RenderPot(ImDrawList* d, const PlacedComponent& c, ImVec2 p, float z) {
  const char* title =
      c.customLabel.empty() ? "POTENTIOMETER" : c.customLabel.c_str();
  DrawBody(d,p,c.size,z,IM_COL32(218,226,234,255),title);

  const float wiper = std::clamp(
      c.internalStates.count(state_keys::kWiperPosition)
          ? c.internalStates.at(state_keys::kWiperPosition)
          : 0.5f,
      0.0f, 1.0f);
  const float resistance =
      c.internalStates.count(state_keys::kResistanceOhms)
          ? c.internalStates.at(state_keys::kResistanceOhms)
          : 10000.0f;
  const std::string resistance_code = FormatSmdResistanceCode(resistance);

  const ImVec2 housing_min{p.x + 27.0f * z, p.y + 25.0f * z};
  const ImVec2 housing_max{p.x + 103.0f * z, p.y + 86.0f * z};
  d->AddRectFilled(housing_min, housing_max, IM_COL32(29, 93, 177, 255),
                   7.0f * z);
  d->AddRect(housing_min, housing_max, IM_COL32(17, 54, 112, 255),
             7.0f * z, 0, 2.0f * z);
  d->AddRectFilled({housing_min.x + 4.0f * z, housing_min.y + 4.0f * z},
                   {housing_max.x - 4.0f * z, housing_min.y + 10.0f * z},
                   IM_COL32(48, 124, 207, 255), 3.0f * z);

  const ImVec2 center{p.x + 65.0f * z, p.y + 53.0f * z};
  const float dial_radius = 19.0f * z;
  d->AddCircleFilled(center, dial_radius + 3.0f * z,
                     IM_COL32(12, 53, 113, 255), 32);
  d->AddCircleFilled(center, dial_radius, IM_COL32(186, 192, 195, 255), 32);
  d->AddCircle(center, dial_radius, IM_COL32(83, 91, 98, 255), 32,
               1.5f * z);

  const float angle = (-135.0f + 270.0f * wiper) *
                      3.14159265358979323846f / 180.0f;
  const ImVec2 axis_a{std::cos(angle), std::sin(angle)};
  const ImVec2 axis_b{-axis_a.y, axis_a.x};
  const float slot_half = 14.0f * z;
  const float slot_width = 5.0f * z;
  d->AddLine({center.x - axis_a.x * slot_half,
              center.y - axis_a.y * slot_half},
             {center.x + axis_a.x * slot_half,
              center.y + axis_a.y * slot_half},
             IM_COL32(78, 83, 87, 255), slot_width);
  d->AddLine({center.x - axis_b.x * slot_half,
              center.y - axis_b.y * slot_half},
             {center.x + axis_b.x * slot_half,
              center.y + axis_b.y * slot_half},
             IM_COL32(78, 83, 87, 255), slot_width);
  // An asymmetric index on one arm makes the cross rotation readable.
  d->AddCircleFilled({center.x + axis_a.x * 13.0f * z,
                      center.y + axis_a.y * 13.0f * z},
                     2.2f * z, IM_COL32(226, 230, 231, 255), 12);

  const float code_font_size = 10.0f * std::min(z, 1.5f);
  ImVec2 code_size = ImGui::GetFont()->CalcTextSizeA(
      code_font_size, 1000.0f, 0.0f, resistance_code.c_str());
  d->AddText(ImGui::GetFont(), code_font_size,
             {p.x + 65.0f * z - code_size.x * 0.5f, p.y + 75.0f * z},
             IM_COL32(225, 238, 250, 255), resistance_code.c_str());

  DrawPorts(d,c,kPotPorts,3,p,z);
}
void RenderCap(ImDrawList* d,const PlacedComponent& c,ImVec2 p,float z){
  const char* title = c.customLabel.empty() ? "CAPACITOR" : c.customLabel.c_str();
  DrawBody(d,p,c.size,z,IM_COL32(203,231,225,255),title);
  d->AddLine({p.x+55*z,p.y+27*z},{p.x+55*z,p.y+72*z},IM_COL32(30,85,75,255),4*z);
  d->AddLine({p.x+75*z,p.y+27*z},{p.x+75*z,p.y+72*z},IM_COL32(30,85,75,255),4*z); DrawPorts(d,c,kTwoPort,2,p,z);
}
void RenderInductor(ImDrawList* d,const PlacedComponent& c,ImVec2 p,float z){
  const char* title = c.customLabel.empty() ? "INDUCTOR" : c.customLabel.c_str();
  DrawBody(d,p,c.size,z,IM_COL32(219,230,190,255),title);
  for (int i = 0; i < 4; ++i) {
    d->AddCircle({p.x + (44 + i * 14) * z, p.y + 53 * z}, 10 * z,
                 IM_COL32(55, 90, 35, 255), 20, 2.5f * z);
  }
  DrawPorts(d, c, kTwoPort, 2, p, z);
}
void RenderDiode(ImDrawList* d,const PlacedComponent& c,ImVec2 p,float z){
  const char* title = c.customLabel.empty() ? "DIODE" : c.customLabel.c_str();
  DrawBody(d,p,c.size,z,IM_COL32(244,224,192,255),title);
  const ImU32 color = IM_COL32(78,58,35,255);
  const float y = p.y + 53.0f*z;
  d->AddLine({p.x,y},{p.x+38*z,y},color,2.5f*z);
  d->AddTriangleFilled({p.x+39*z,p.y+35*z},{p.x+39*z,p.y+71*z},
                       {p.x+83*z,y},color);
  d->AddLine({p.x+84*z,p.y+34*z},{p.x+84*z,p.y+72*z},color,3.0f*z);
  d->AddLine({p.x+84*z,y},{p.x+130*z,y},color,2.5f*z);
  DrawPorts(d,c,kTwoPort,2,p,z);
}
void RenderBjt(ImDrawList* d,const PlacedComponent& c,ImVec2 p,float z,
               bool npn){
  const char* fallback = npn ? "NPN BJT" : "PNP BJT";
  const char* title = c.customLabel.empty() ? fallback : c.customLabel.c_str();
  DrawBody(d,p,c.size,z,npn ? IM_COL32(211,229,242,255)
                            : IM_COL32(237,218,234,255),title);
  const ImU32 color = IM_COL32(40,55,70,255);
  const ImVec2 base_a{p.x+50*z,p.y+35*z};
  const ImVec2 base_b{p.x+50*z,p.y+72*z};
  const float center_y = p.y + 53.0f*z;
  d->AddLine({p.x,center_y},{p.x+50*z,center_y},color,2.5f*z);
  d->AddLine(base_a,base_b,color,3.0f*z);
  d->AddLine({p.x+50*z,p.y+42*z},{p.x+93*z,p.y+25*z},color,2.5f*z);
  d->AddLine({p.x+50*z,p.y+65*z},{p.x+93*z,p.y+81*z},color,2.5f*z);
  d->AddLine({p.x+93*z,p.y+25*z},{p.x+130*z,p.y+25*z},color,2.5f*z);
  d->AddLine({p.x+93*z,p.y+81*z},{p.x+130*z,p.y+81*z},color,2.5f*z);
  const ImVec2 arrow_tip = npn ? ImVec2{p.x+91*z,p.y+80*z}
                               : ImVec2{p.x+60*z,p.y+68*z};
  const ImVec2 arrow_a = npn ? ImVec2{p.x+76*z,p.y+78*z}
                             : ImVec2{p.x+74*z,p.y+65*z};
  const ImVec2 arrow_b = npn ? ImVec2{p.x+84*z,p.y+68*z}
                             : ImVec2{p.x+67*z,p.y+77*z};
  d->AddTriangleFilled(arrow_tip,arrow_a,arrow_b,color);
  DrawPorts(d,c,kBjtPorts,3,p,z);
}
void RenderNpn(ImDrawList* d,const PlacedComponent& c,ImVec2 p,float z){
  RenderBjt(d,c,p,z,true);
}
void RenderPnp(ImDrawList* d,const PlacedComponent& c,ImVec2 p,float z){
  RenderBjt(d,c,p,z,false);
}
void RenderOpAmp(ImDrawList* d,const PlacedComponent& c,ImVec2 p,float z){
  const char* title = c.customLabel.empty() ? "OP-AMP" : c.customLabel.c_str();
  DrawBody(d,p,c.size,z,IM_COL32(205,225,239,255),title);

  const ImVec2 upper_input{p.x + 18.0f * z, p.y + 40.0f * z};
  const ImVec2 lower_input{p.x + 18.0f * z, p.y + 66.0f * z};
  const ImVec2 triangle_upper{p.x + 38.0f * z, p.y + 27.0f * z};
  const ImVec2 triangle_lower{p.x + 38.0f * z, p.y + 78.0f * z};
  const ImVec2 output{p.x + 118.0f * z, p.y + 53.0f * z};
  const ImU32 symbol_color = IM_COL32(25, 65, 95, 255);

  // IEEE/ANSI operational-amplifier symbol: triangle, inverting input on
  // top, non-inverting input below, and output at the apex.
  d->AddTriangle(triangle_upper, triangle_lower, output, symbol_color,
                 2.5f * z);
  d->AddLine(upper_input, {p.x + 38.0f * z, upper_input.y}, symbol_color,
             2.0f * z);
  d->AddLine(lower_input, {p.x + 38.0f * z, lower_input.y}, symbol_color,
             2.0f * z);
  d->AddLine(output, {p.x + 145.0f * z, output.y}, symbol_color, 2.0f * z);
  d->AddLine({p.x+67*z,p.y+34*z},{p.x+67*z,p.y},symbol_color,2.0f*z);
  d->AddLine({p.x+67*z,p.y+72*z},{p.x+67*z,p.y+105*z},symbol_color,2.0f*z);

  const float sign_font_size = 17.0f * std::min(z, 1.5f);
  d->AddText(ImGui::GetFont(), sign_font_size,
             {p.x + 43.0f * z, p.y + 31.0f * z}, symbol_color, "-");
  d->AddText(ImGui::GetFont(), sign_font_size,
             {p.x + 42.0f * z, p.y + 56.0f * z}, symbol_color, "+");
  DrawPorts(d,c,kOpAmpPorts,5,p,z);
}
void RenderSpeaker(ImDrawList* d,const PlacedComponent& c,ImVec2 p,float z){
  DrawBody(d,p,c.size,z,IM_COL32(235,218,205,255),"SPEAKER");
  d->AddCircleFilled({p.x+78*z,p.y+58*z},30*z,IM_COL32(45,48,52,255));
  d->AddCircle({p.x+78*z,p.y+58*z},21*z,IM_COL32(185,190,195,255),0,3*z);
  d->AddText({p.x+112*z,p.y+30*z},IM_COL32(75,78,82,255),"L");
  d->AddText({p.x+112*z,p.y+76*z},IM_COL32(75,78,82,255),"R");
  DrawPorts(d,c,kSpeakerPorts,4,p,z);
}
void RenderGround(ImDrawList* d,const PlacedComponent& c,ImVec2 p,float z){
  (void)c; d->AddLine({p.x+45*z,p.y},{p.x+45*z,p.y+28*z},IM_COL32(35,45,50,255),3*z);
  for (int i = 0; i < 3; ++i) {
    d->AddLine({p.x + (20 + i * 7) * z, p.y + (30 + i * 8) * z},
               {p.x + (70 - i * 7) * z, p.y + (30 + i * 8) * z},
               IM_COL32(35, 45, 50, 255), 3 * z);
  }
  DrawPorts(d, c, kGroundPorts, 1, p, z);
}
void RenderDcSource(ImDrawList* d, const PlacedComponent& c, ImVec2 p, float z) {
  DrawBody(d, p, c.size, z, IM_COL32(250, 225, 154, 255), "DC POWER");
  const float voltage = c.internalStates.count(state_keys::kVoltageVolts)
                            ? c.internalStates.at(state_keys::kVoltageVolts)
                            : 5.0f;
  char value[24] = {};
  std::snprintf(value, sizeof(value), "%.2f V", voltage);
  d->AddLine({p.x + 38*z,p.y + 43*z},{p.x + 38*z,p.y + 78*z},IM_COL32(80,60,20,255),2*z);
  d->AddLine({p.x + 52*z,p.y + 35*z},{p.x + 52*z,p.y + 86*z},IM_COL32(80,60,20,255),4*z);
  d->AddText({p.x + 68*z,p.y + 52*z}, IM_COL32(70,55,20,255), value);
  DrawPorts(d, c, kSupplyPorts, 2, p, z);
}
void RenderAcSource(ImDrawList* d, const PlacedComponent& c, ImVec2 p, float z) {
  DrawBody(d, p, c.size, z, IM_COL32(198, 229, 246, 255), "AC / EQ SWEEP");
  const ImVec2 center{p.x + 48*z, p.y + 60*z};
  d->AddCircle(center, 25*z, IM_COL32(30,80,115,255), 24, 2*z);
  ImVec2 previous{center.x - 18*z, center.y};
  for (int i = 1; i <= 18; ++i) {
    const float x = center.x + (-18.0f + i*2.0f)*z;
    const float y = center.y - std::sin(i*3.14159265f/9.0f)*9.0f*z;
    d->AddLine(previous, {x,y}, IM_COL32(30,80,115,255), 1.7f*z);
    previous = {x,y};
  }
  const float frequency = c.internalStates.count(state_keys::kFrequencyHz)
                              ? c.internalStates.at(state_keys::kFrequencyHz)
                              : 1000.0f;
  char value[24] = {};
  std::snprintf(value, sizeof(value), "%.0f Hz", frequency);
  d->AddText({p.x + 79*z,p.y + 52*z}, IM_COL32(30,70,100,255), value);
  DrawPorts(d, c, kSupplyPorts, 2, p, z);
}
void RenderPulseSource(ImDrawList* d, const PlacedComponent& c, ImVec2 p, float z) {
  DrawBody(d, p, c.size, z, IM_COL32(222, 211, 244, 255), "PULSE GENERATOR");
  const ImU32 color = IM_COL32(75,45,115,255);
  const float x = p.x + 27*z, y = p.y + 68*z;
  d->AddLine({x,y},{x+15*z,y},color,2*z);
  d->AddLine({x+15*z,y},{x+15*z,y-27*z},color,2*z);
  d->AddLine({x+15*z,y-27*z},{x+48*z,y-27*z},color,2*z);
  d->AddLine({x+48*z,y-27*z},{x+48*z,y},color,2*z);
  d->AddLine({x+48*z,y},{x+65*z,y},color,2*z);
  const float duty = c.internalStates.count(state_keys::kDutyCyclePercent)
                         ? c.internalStates.at(state_keys::kDutyCyclePercent)
                         : 50.0f;
  char value[24] = {};
  std::snprintf(value, sizeof(value), "%.1f%%", duty);
  d->AddText({p.x + 98*z,p.y + 52*z}, color, value);
  DrawPorts(d, c, kSupplyPorts, 2, p, z);
}

#define AUDIO_DEF(name, type_value, key, desc, w, h, ports, render, init, icon, shortname) \
  const ComponentDefinition name = {ComponentType::type_value, key, desc, {w,h}, ports, static_cast<int>(sizeof(ports)/sizeof(ports[0])), render, init, ComponentCategory::BOTH, icon, shortname}

AUDIO_DEF(kSource, AUDIO_SOURCE, "component.audio_source.name", "component.audio_source.desc", 150,110,kSourcePorts,&RenderSource,&InitSource,"icon.audio_source","PC Out");
AUDIO_DEF(kAdc, AUDIO_ADC, "component.adc.name", "component.adc.desc", 130,95,kTwoPort,&RenderAdc,&InitAdc,"icon.adc","ADC");
AUDIO_DEF(kResistor, AUDIO_RESISTOR, "component.resistor.name", "component.resistor.desc",130,95,kTwoPort,&RenderResistor,&InitResistor,"icon.resistor","R");
AUDIO_DEF(kPot, AUDIO_POTENTIOMETER, "component.potentiometer.name", "component.potentiometer.desc",130,95,kPotPorts,&RenderPot,&InitPot,"icon.pot","POT");
AUDIO_DEF(kCap, AUDIO_CAPACITOR, "component.capacitor.name", "component.capacitor.desc",130,95,kTwoPort,&RenderCap,&InitCap,"icon.capacitor","C");
AUDIO_DEF(kInductor, AUDIO_INDUCTOR, "component.inductor.name", "component.inductor.desc",130,95,kTwoPort,&RenderInductor,&InitInductor,"icon.inductor","L");
AUDIO_DEF(kDiode, AUDIO_DIODE, "component.diode.name", "component.diode.desc",130,105,kTwoPort,&RenderDiode,&InitDiode,"icon.diode","D");
AUDIO_DEF(kBjtNpn, AUDIO_BJT_NPN, "component.bjt_npn.name", "component.bjt_npn.desc",130,106,kBjtPorts,&RenderNpn,&InitBjt,"icon.bjt_npn","NPN");
AUDIO_DEF(kBjtPnp, AUDIO_BJT_PNP, "component.bjt_pnp.name", "component.bjt_pnp.desc",130,106,kBjtPorts,&RenderPnp,&InitBjt,"icon.bjt_pnp","PNP");
AUDIO_DEF(kOpAmp, AUDIO_OP_AMP, "component.op_amp.name", "component.op_amp.desc",145,105,kOpAmpPorts,&RenderOpAmp,&InitOpAmp,"icon.op_amp","OP AMP");
AUDIO_DEF(kDac, AUDIO_DAC, "component.dac.name", "component.dac.desc",130,95,kTwoPort,&RenderDac,&InitDac,"icon.dac","DAC");
AUDIO_DEF(kSpeaker, AUDIO_SPEAKER, "component.speaker.name", "component.speaker.desc",140,115,kSpeakerPorts,&RenderSpeaker,&InitSpeaker,"icon.speaker","Speaker");
AUDIO_DEF(kGround, AUDIO_GROUND, "component.audio_ground.name", "component.audio_ground.desc",90,65,kGroundPorts,&RenderGround,nullptr,"icon.ground","GND");
AUDIO_DEF(kDcSource, AUDIO_DC_SOURCE, "component.dc_source.name", "component.dc_source.desc",140,115,kSupplyPorts,&RenderDcSource,&InitDcSource,"icon.dc_source","DC");
AUDIO_DEF(kAcSource, AUDIO_AC_SOURCE, "component.ac_source.name", "component.ac_source.desc",140,115,kSupplyPorts,&RenderAcSource,&InitAcSource,"icon.ac_source","AC");
AUDIO_DEF(kPulseSource, AUDIO_PULSE_SOURCE, "component.pulse_source.name", "component.pulse_source.desc",140,115,kSupplyPorts,&RenderPulseSource,&InitPulseSource,"icon.pulse_source","PULSE");
#undef AUDIO_DEF
}  // namespace

const ComponentDefinition* GetAudioSourceDefinition(){return &kSource;}
const ComponentDefinition* GetAudioAdcDefinition(){return &kAdc;}
const ComponentDefinition* GetAudioResistorDefinition(){return &kResistor;}
const ComponentDefinition* GetAudioPotentiometerDefinition(){return &kPot;}
const ComponentDefinition* GetAudioCapacitorDefinition(){return &kCap;}
const ComponentDefinition* GetAudioInductorDefinition(){return &kInductor;}
const ComponentDefinition* GetAudioDiodeDefinition(){return &kDiode;}
const ComponentDefinition* GetAudioBjtNpnDefinition(){return &kBjtNpn;}
const ComponentDefinition* GetAudioBjtPnpDefinition(){return &kBjtPnp;}
const ComponentDefinition* GetAudioOpAmpDefinition(){return &kOpAmp;}
const ComponentDefinition* GetAudioDacDefinition(){return &kDac;}
const ComponentDefinition* GetAudioSpeakerDefinition(){return &kSpeaker;}
const ComponentDefinition* GetAudioGroundDefinition(){return &kGround;}
const ComponentDefinition* GetAudioDcSourceDefinition(){return &kDcSource;}
const ComponentDefinition* GetAudioAcSourceDefinition(){return &kAcSource;}
const ComponentDefinition* GetAudioPulseSourceDefinition(){return &kPulseSource;}
const ComponentDefinition* GetAudioRtlShellAppearance(
    const std::string& module_id) {
  if (module_id == "audio_shell_adc") return &kAdc;
  if (module_id == "audio_shell_dac") return &kDac;
  if (module_id == "audio_shell_amplifier") return &kOpAmp;
  return nullptr;
}
}  // namespace plc
