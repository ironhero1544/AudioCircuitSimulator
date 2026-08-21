#ifndef PLC_EMULATOR_INCLUDE_PLC_EMULATOR_COMPONENTS_STATE_KEYS_H_
#define PLC_EMULATOR_INCLUDE_PLC_EMULATOR_COMPONENTS_STATE_KEYS_H_

namespace plc {
namespace state_keys {
constexpr const char* kPosition = "position";
constexpr const char* kVelocity = "velocity";
constexpr const char* kPressureA = "pressure_a";
constexpr const char* kPressureB = "pressure_b";
constexpr const char* kStatus = "status";
constexpr const char* kIsPressed = "is_pressed";
constexpr const char* kIsPressedManual = "is_pressed_manual";
constexpr const char* kIsDetected = "is_detected";
constexpr const char* kIsPowered = "is_powered";
constexpr const char* kAirPressure = "air_pressure";
constexpr const char* kSolenoidAActive = "solenoid_a_active";
constexpr const char* kSolenoidBActive = "solenoid_b_active";
constexpr const char* kLastActiveSolenoid = "last_active_solenoid";
constexpr const char* kLampOnPrefix = "lamp_on_";
constexpr const char* kPressedPrefix = "is_pressed_";
constexpr const char* kIsMetal = "is_metal";
constexpr const char* kIsProcessed = "is_processed";
constexpr const char* kMotorActive = "motor_active";
constexpr const char* kMotorOn = "motor_on";
constexpr const char* kRotationAngle = "rot_angle";
constexpr const char* kFlowSetting = "flow_setting";
constexpr const char* kMeterMode = "meter_mode";
constexpr const char* kMeterMenuOpen = "meter_menu_open";
constexpr const char* kUserText = "user_text";
constexpr const char* kLampRed = "lamp_r";
constexpr const char* kLampYellow = "lamp_y";
constexpr const char* kLampGreen = "lamp_g";
constexpr const char* kIsManualDrag = "is_manual_drag";
constexpr const char* kIsContacted = "is_contacted";
constexpr const char* kIsStuckBox = "is_stuck_box";
constexpr const char* kVelX = "vel_x";
constexpr const char* kVelY = "vel_y";
constexpr const char* kPlcRunning = "is_running";
constexpr const char* kPlcError = "plc_error";
constexpr const char* kPlcXPrefix = "plc_x_";
constexpr const char* kPlcYPrefix = "plc_y_";
constexpr const char* kPlcInputMode = "plc_input_mode";
constexpr const char* kPlcOutputMode = "plc_output_mode";
constexpr const char* kSensorOutputMode = "sensor_output_mode";
constexpr const char* kAudioLevel = "audio_level";
constexpr const char* kBitDepth = "bit_depth";
constexpr const char* kResistanceOhms = "resistance_ohms";
constexpr const char* kTolerancePercent = "tolerance_percent";
constexpr const char* kWiperPosition = "wiper_position";
constexpr const char* kCapacitanceUf = "capacitance_uf";
constexpr const char* kEsrOhms = "esr_ohms";
constexpr const char* kLeakageResistanceOhms = "leakage_resistance_ohms";
constexpr const char* kVoltageRatingVolts = "voltage_rating_volts";
constexpr const char* kInductanceMh = "inductance_mh";
constexpr const char* kDcrOhms = "dcr_ohms";
constexpr const char* kSaturationCurrentAmps = "saturation_current_amps";
constexpr const char* kGain = "gain";
constexpr const char* kGainBandwidthHz = "gain_bandwidth_hz";
constexpr const char* kOutputResistanceOhms = "output_resistance_ohms";
constexpr const char* kOutputHeadroomVolts = "output_headroom_volts";
constexpr const char* kOpenLoopGain = "open_loop_gain";
constexpr const char* kSlewRateVoltsPerUs = "slew_rate_volts_per_us";
constexpr const char* kInputOffsetMillivolts = "input_offset_millivolts";
constexpr const char* kOutputCurrentLimitAmps = "output_current_limit_amps";
constexpr const char* kForwardVoltageVolts = "forward_voltage_volts";
constexpr const char* kDynamicResistanceOhms = "dynamic_resistance_ohms";
constexpr const char* kThermalCoupling = "thermal_coupling";
constexpr const char* kCurrentGainBeta = "current_gain_beta";
constexpr const char* kBaseEmitterVoltageVolts = "base_emitter_voltage_volts";
constexpr const char* kSaturationVoltageVolts = "saturation_voltage_volts";
constexpr const char* kMaximumCollectorCurrentAmps =
    "maximum_collector_current_amps";
constexpr const char* kMaximumPowerWatts = "maximum_power_watts";
constexpr const char* kThermalResistanceCPerW = "thermal_resistance_c_per_w";
constexpr const char* kImpedanceOhms = "impedance_ohms";
constexpr const char* kTransducerEq1FrequencyHz =
    "transducer_eq1_frequency_hz";
constexpr const char* kTransducerEq1GainDb = "transducer_eq1_gain_db";
constexpr const char* kTransducerEq1Q = "transducer_eq1_q";
constexpr const char* kTransducerEq2FrequencyHz =
    "transducer_eq2_frequency_hz";
constexpr const char* kTransducerEq2GainDb = "transducer_eq2_gain_db";
constexpr const char* kTransducerEq2Q = "transducer_eq2_q";
constexpr const char* kTransducerEq3FrequencyHz =
    "transducer_eq3_frequency_hz";
constexpr const char* kTransducerEq3GainDb = "transducer_eq3_gain_db";
constexpr const char* kTransducerEq3Q = "transducer_eq3_q";
constexpr const char* kTransducerEq4FrequencyHz =
    "transducer_eq4_frequency_hz";
constexpr const char* kTransducerEq4GainDb = "transducer_eq4_gain_db";
constexpr const char* kTransducerEq4Q = "transducer_eq4_q";
constexpr const char* kVoltageVolts = "voltage_volts";
constexpr const char* kCurrentLimitAmps = "current_limit_amps";
constexpr const char* kInternalResistanceOhms = "internal_resistance_ohms";
constexpr const char* kSpeakerReOhms = "speaker_re_ohms";
constexpr const char* kSpeakerLeMh = "speaker_le_mh";
constexpr const char* kSpeakerBlTeslaMeters = "speaker_bl_tesla_meters";
constexpr const char* kSpeakerMmsGrams = "speaker_mms_grams";
constexpr const char* kSpeakerCmsMmPerNewton =
    "speaker_cms_mm_per_newton";
constexpr const char* kSpeakerRmsNewtonSecondsPerMeter =
    "speaker_rms_newton_seconds_per_meter";
constexpr const char* kFrequencyHz = "frequency_hz";
constexpr const char* kFrequencyMinHz = "frequency_min_hz";
constexpr const char* kFrequencyMaxHz = "frequency_max_hz";
constexpr const char* kPhaseDegrees = "phase_degrees";
constexpr const char* kDutyCyclePercent = "duty_cycle_percent";
constexpr const char* kPulseLowVolts = "pulse_low_volts";
constexpr const char* kPulseHighVolts = "pulse_high_volts";
}
}  /* namespace plc */

#endif  /* PLC_EMULATOR_INCLUDE_PLC_EMULATOR_COMPONENTS_STATE_KEYS_H_ */
