/*
This battery has - 12 temperature sensors
                 - 72 cells

CAN ID	ECU	Notes
0x125	VCU → Dashboard	Speed, torque request, status
0x127	MCU/Inverter	Motor speed, torque, temperatures
0x134	MCU/Inverter	Phase currents, voltages
0x135	MCU/Inverter	Motor RPM, torque
0x1EA	BMS heartbeat	1‑byte alive counter
0x231	VCU	Accelerator, brake, torque limits
0x240	VCU	Torque commands
0x251	BMS	Pack voltage, current, SOC
0x261 / 0x262	BMS	Cell block voltages, temperatures (bank1, bank2)
0x263 BMS balancing bitmask
0x3D3 / 3D5 / 3D6 / 3D7 / 3D8	ADAS / BCM / EPS	Body signals, steering, lights
0x43C / 0x431	ABS / ESP	Wheel speeds, stability control
0x442 / 0x443 — Brake / pedal / ESP status
0x511 — Charging / energy / DC‑DC related
0x59B cluster/BCM  status frame

ID	Meaning
0x261	Cell voltages (block A)
0x262	Cell voltages / temps / balancing (block B)
0x231	Min/max/avg cell summary
0x240	Temperature sensors
0x251	Pack voltage, current, SOC
0x1EA	Heartbeat

Received from BAT
CAN ID	ECU	  Meaning	                 Battery‑related?
0x5A9	  EPS	  Steering torque / assist  	   ❌
0x5AB	  BCM	  Keep‑alive / heartbeat	       ❌
0x5C8	ABS/ESP	Wheel speed / stability	       ❌
*/
#ifndef CMFA_EV_BATTERY_H
#define CMFA_EV_BATTERY_H

#include "../datalayer/datalayer.h"
#include "../datalayer/datalayer_extended.h"
#include "CMFA-EV-HTML.h"
#include "CanBattery.h"

// UDS Multi-Frame Reception Context
struct CMFA_UDS_CONTEXT {
  uint8_t UDS_buffer[512];            // Buffer to store multi-frame UDS data
  uint16_t UDS_bytesReceived;         // Track number of bytes received
  uint16_t UDS_expectedLength;        // Total expected length from first frame
  uint8_t UDS_sequenceNumber;         // Expected sequence number for consecutive frames (for IX non-batch mode)
  bool UDS_inProgress;                // Flag indicating if we're in the middle of receiving
  uint8_t UDS_sessionID;              // Session ID responding - SID (0x19 for DTC, 0x22 for values)
  uint8_t UDS_subfuncID;              // SubFunction ID responding (0xF4 for battery)
  uint16_t UDS_dataID;                // Data ID responding - DID (0xF190 for VIN, 0x9021 battery cell 1 voltage)
  uint8_t receivedInBatch;            // Number of CFs received in current batch (for PHEV batch mode)
  unsigned long UDS_lastFrameMillis;  // Timestamp of last frame for timeout detection
};

class CmfaEvBattery : public CanBattery {
 public:
  // Use this constructor for the second battery.
  CmfaEvBattery(DATALAYER_BATTERY_TYPE* datalayer_ptr, DATALAYER_INFO_CMFAEV* extended, CAN_Interface targetCan)
      : CanBattery(targetCan),renderer(*this) {
    datalayer_battery = datalayer_ptr;
    allows_contactor_closing = nullptr;
    datalayer_cmfa = extended;

    cells_voltage_sum = 0;
  }

  // Use the default constructor to create the first or single battery.
  CmfaEvBattery() : renderer(*this) {
    datalayer_battery = &datalayer.battery;
    allows_contactor_closing = &datalayer.system.status.battery_allows_contactor_closing;
    datalayer_cmfa = &datalayer_extended.CMFAEV;
  }

  bool supports_read_DTC() { return true; }
  void read_DTC() { UserRequestDTCRead = true; }
  bool supports_reset_DTC() { return true; }
  void reset_DTC() { UserRequestDTCclear = true; }

  bool get_end_of_charge() const;
  uint8_t get_hvil_status() const;
  uint8_t get_hvil1_raw_state() const;
  uint8_t get_hvil2_raw_state() const;
  uint8_t get_hvil3_raw_state() const;
  uint8_t get_hvil4_raw_state() const;
  bool get_dc_charge_support() const;
  uint16_t get_dc_res_charge() const;
  uint16_t get_dc_res_discharge() const;
  uint16_t get_complete_charge_counter() const;
  uint16_t get_incomplete_charge_counter() const;
  uint32_t get_busbar_fail() const;
  uint32_t get_slaves_fail() const;
  uint32_t get_battery_mileage_km() const;
  uint32_t get_pack_lifetime_mnt() const;
  uint32_t get_balance_charge_total() const;
  uint32_t get_balance_time_total() const;
  uint32_t get_balance_charge_total_sleep() const;
  uint32_t get_balance_time_total_sleep() const;
  bool get_balance_any_cell() const;

  virtual void setup(void);
  virtual void handle_incoming_can_frame(CAN_frame rx_frame);
  virtual void update_values();
  virtual void transmit_can(unsigned long currentMillis);
  static constexpr const char* Name = "CMFA platform, 27 kWh battery";

  BatteryHtmlRenderer& get_status_renderer() { return renderer; }

 private:
  CmfaEvHtmlRenderer renderer;

  DATALAYER_BATTERY_TYPE* datalayer_battery;
  DATALAYER_INFO_CMFAEV* datalayer_cmfa;

  // If not null, this battery decides when the contactor can be closed and writes the value here.
  bool* allows_contactor_closing;

  bool UserRequestDTCRead = false;
  bool UserRequestDTCclear = false;

  uint16_t rescale_raw_SOC(uint32_t raw_SOC);

  // UDS Multi-Frame Helpers
  void startUDSMultiFrameReception(uint16_t totalLength, uint8_t subfuncID, uint16_t dataID);
  bool storeUDSPayload(const uint8_t* payload, uint8_t length);
  bool isUDSMessageComplete();
  void parseDTCResponse();
  void parseDIDResponse();
  void handleISOTPFrame(CAN_frame& rx_frame);
  void processCompletedUDSResponse();
  int  find_last_data_byte(const uint8_t *buf, uint8_t len);

  static const int MAX_PACK_VOLTAGE_DV = 3040;  // 5000 = 500.0V
  static const int MIN_PACK_VOLTAGE_DV = 2185;
  static const int MAX_CELL_DEVIATION_MV = 100;
  static const int MAX_CELL_VOLTAGE_MV = 4250;  // Emergency stop if above
  static const int MIN_CELL_VOLTAGE_MV = 2700;  // Emergency stop if below

  // OBD2 PID polls
  static const int PID_POLL_SOCZ = 0x9001; // (x + 6) * 0.5 = 49%
  static const int PID_POLL_USOC = 0x9002; // x * 0.01 = 40.14%
  static const int PID_POLL_SOH_AVERAGE = 0x9003; // x * 0.01 = 85.77%
  static const int PID_POLL_PACK_VOLTAGE_CAN = 0x9005; // x * 0.1 = 263.9V
  static const int PID_POLL_CELLS_VOLTAGE_SUM = 0x9006; // x * 0.0009765625 = 263.9375 V
  static const int PID_POLL_HIGHEST_CELL_VOLTAGE = 0x9007; // x * 0.000976563 = 3.6689 V
  static const int PID_POLL_CELL_NUMBER_HIGHEST_VOLTAGE = 0x9008;
  static const int PID_POLL_LOWEST_CELL_VOLTAGE = 0x9009;  // x * 0.000976563 = 3.6640 V
  static const int PID_POLL_CELL_NUMBER_LOWEST_VOLTAGE = 0x900A;
  static const int PID_POLL_CURRENT_OFFSET = 0x900C; // (x - 32640) * 0.03125 = N/A A
  static const int PID_POLL_INSTANT_CURRENT = 0x900D; // (x - 48000) * 0.025 = 0.05 A
  static const int PID_POLL_MAX_REGEN = 0x900E; // x * 0.01 = 42.19kW
  static const int PID_POLL_MAX_DISCHARGE_POWER = 0x900F; // x * 0.01 = 42.19kW
  static const int PID_POLL_MAX_GENERATED_POWER_AFTER_RESTRICTION = 0x900E; // Same
  static const int PID_POLL_MAX_AVAILABLE_POWER_AFTER_RESTRICTION = 0x900F; // Same
  static const int PID_POLL_12V_BATTERY = 0x9011; // x * 0.000976563 = 11.7636 V
  static const int PID_POLL_AVERAGE_TEMPERATURE = 0x9012; // (x - 640) * 0.0625 = 11.1875 C
  static const int PID_POLL_MIN_TEMPERATURE = 0x9013; // (x - 640) * 0.0625 = 11.0 C
  static const int PID_POLL_MAX_TEMPERATURE = 0x9014; // (x - 640) * 0.0625 = 13.0 C
  static const int PID_POLL_MAX_CHARGE_POWER = 0x9018; // x * 0.01 = 23.9 KW
  static const int PID_POLL_END_OF_CHARGE_FLAG = 0x9019; // 0:Charging OK;1:Charging Stop
  static const int PID_POLL_INTERLOCK_FLAG = 0x901A; // 0:Not used;1:Opened;2:Closed;3:Unavailable
  static const int PID_POLL_BATTERY_ID = 0x901B; // Battery Identification Number BIN
  static const int PID_POLL_BATTERY_TRACEABILITY_ID = 0x901C; // Traceability Identification Number TIN (not used / All 0)
  static const int PID_POLL_BATTERY_FUNCTIONAL_ID = 0x901D; // Functional Identification Number FIN (not used / All 0)


  static const int PID_POLL_CELL_1 = 0x9021;
  static const int PID_POLL_CELL_2 = 0x9022;
  static const int PID_POLL_CELL_3 = 0x9023;
  static const int PID_POLL_CELL_4 = 0x9024;
  static const int PID_POLL_CELL_5 = 0x9025;
  static const int PID_POLL_CELL_6 = 0x9026;
  static const int PID_POLL_CELL_7 = 0x9027;
  static const int PID_POLL_CELL_8 = 0x9028;
  static const int PID_POLL_CELL_9 = 0x9029;
  static const int PID_POLL_CELL_10 = 0x902A;
  static const int PID_POLL_CELL_11 = 0x902B;
  static const int PID_POLL_CELL_12 = 0x902C;
  static const int PID_POLL_CELL_13 = 0x902D;
  static const int PID_POLL_CELL_14 = 0x902E;
  static const int PID_POLL_CELL_15 = 0x902F;
  static const int PID_POLL_CELL_16 = 0x9030;
  static const int PID_POLL_CELL_17 = 0x9031;
  static const int PID_POLL_CELL_18 = 0x9032;
  static const int PID_POLL_CELL_19 = 0x9033;
  static const int PID_POLL_CELL_20 = 0x9034;
  static const int PID_POLL_CELL_21 = 0x9035;
  static const int PID_POLL_CELL_22 = 0x9036;
  static const int PID_POLL_CELL_23 = 0x9037;
  static const int PID_POLL_CELL_24 = 0x9038;
  static const int PID_POLL_CELL_25 = 0x9039;
  static const int PID_POLL_CELL_26 = 0x903A;
  static const int PID_POLL_CELL_27 = 0x903B;
  static const int PID_POLL_CELL_28 = 0x903C;
  static const int PID_POLL_CELL_29 = 0x903D;
  static const int PID_POLL_CELL_30 = 0x903E;
  static const int PID_POLL_CELL_31 = 0x903F;

  static const int PID_POLL_DIDS_SUPPORTED_IN_RANGE_9041_9060 = 0x9040;

  static const int PID_POLL_CELL_32 = 0x9041;
  static const int PID_POLL_CELL_33 = 0x9042;
  static const int PID_POLL_CELL_34 = 0x9043;
  static const int PID_POLL_CELL_35 = 0x9044;
  static const int PID_POLL_CELL_36 = 0x9045;
  static const int PID_POLL_CELL_37 = 0x9046;
  static const int PID_POLL_CELL_38 = 0x9047;
  static const int PID_POLL_CELL_39 = 0x9048;
  static const int PID_POLL_CELL_40 = 0x9049;
  static const int PID_POLL_CELL_41 = 0x904A;
  static const int PID_POLL_CELL_42 = 0x904B;
  static const int PID_POLL_CELL_43 = 0x904C;
  static const int PID_POLL_CELL_44 = 0x904D;
  static const int PID_POLL_CELL_45 = 0x904E;
  static const int PID_POLL_CELL_46 = 0x904F;
  static const int PID_POLL_CELL_47 = 0x9050;
  static const int PID_POLL_CELL_48 = 0x9051;
  static const int PID_POLL_CELL_49 = 0x9052;
  static const int PID_POLL_CELL_50 = 0x9053;
  static const int PID_POLL_CELL_51 = 0x9054;
  static const int PID_POLL_CELL_52 = 0x9055;
  static const int PID_POLL_CELL_53 = 0x9056;
  static const int PID_POLL_CELL_54 = 0x9057;
  static const int PID_POLL_CELL_55 = 0x9058;
  static const int PID_POLL_CELL_56 = 0x9059;
  static const int PID_POLL_CELL_57 = 0x905A;
  static const int PID_POLL_CELL_58 = 0x905B;
  static const int PID_POLL_CELL_59 = 0x905C;
  static const int PID_POLL_CELL_60 = 0x905D;
  static const int PID_POLL_CELL_61 = 0x905E;
  static const int PID_POLL_CELL_62 = 0x905F;

  static const int PID_POLL_DIDS_SUPPORTED_IN_RANGE_9061_9080 = 0x9060;

  static const int PID_POLL_CELL_63 = 0x9061;
  static const int PID_POLL_CELL_64 = 0x9062;
  static const int PID_POLL_CELL_65 = 0x9063;
  static const int PID_POLL_CELL_66 = 0x9064;
  static const int PID_POLL_CELL_67 = 0x9065;
  static const int PID_POLL_CELL_68 = 0x9066;
  static const int PID_POLL_CELL_69 = 0x9067;
  static const int PID_POLL_CELL_70 = 0x9068;
  static const int PID_POLL_CELL_71 = 0x9069;
  static const int PID_POLL_CELL_72 = 0x906A;

  static const int PID_POLL_SLAVES_FAIL = 0x9129; // 0:No failure;1:Slave No.21 failure (21 bits)
  static const int PID_POLL_BALANCING_SW_STATUS = 0x912B; // 256 values? 32bytes
  static const int PID_POLL_FAN_SPEED_RPM = 0x912E; // x32
  static const int PID_POLL_TEMP_SENS_FAILURE = 0x912F; // 0:no failure;1:failure OC/SCW;2:SCG failure (20 bits)
  static const int PID_POLL_HV_CAN_CUR_SENS_VOLTAGE = 0x9130; // x32
  static const int PID_POLL_TEMP_SENS_VAL = 0x9131;// 9131-913F

  static const int PID_POLL_SOC_MIN_Z = 0x91B9; // (x - 300) * 0.01 =  %
  static const int PID_POLL_SOC_MAX_Z = 0x91BA; // (x - 300) * 0.01 =  %

  static const int PID_POLL_SOH_AVAILABLE_POWER_CALCULATION = 0x91BC; // (x * 0.1) = %
  static const int PID_POLL_SOH_GENERATED_POWER_CALCULATION = 0x91BD; // (x * 0.1) = %

  static const int PID_POLL_PACK_LIFE_TIME_MIN = 0x91C1; // minutes
  static const int PID_POLL_DC_RES_CHARGE = 0x91C6; // mOhm (x0.25) (per pack?)
  static const int PID_POLL_DC_RES_DISCHARGE = 0x91C7; // mOhm (x0.25) (per pack?)
  static const int PID_POLL_AVAILABLE_ENERGY = 0x91C8; // KW (x0.001)
  static const int PID_POLL_FAN_CONTROL_REQ = 0x91C9; // % (x0.5)
  static const int PID_POLL_MAIN_RELAY = 0x91CA; // 0:Openning control;1:Closing control
  static const int PID_POLL_PRECHARGE_RELAY = 0x91CB; // 0:Openning control;1:Closing control
  static const int PID_POLL_BUSBAR_FAIL = 0x91CC; //0:No failure;1:Busbar no.20 failure (20 bits)
  static const int PID_POLL_BATTERY_MILEAGE_KM = 0x91CF; // km

  static const int PID_POLL_INTERLOCK_1 = 0x91F6; // 3:SC+_ret;2:SC+;1:closed;0:open;4:SC_gnd
  static const int PID_POLL_INTERLOCK_2 = 0x91F7; // 3:SC+_ret;2:SC+;1:closed;0:open;4:SC_gnd
  static const int PID_POLL_INTERLOCK_3 = 0x91F8; // 3:SC+_ret;2:SC+;1:closed;0:open;4:SC_gnd
  static const int PID_POLL_INTERLOCK_4 = 0x91F9; // 3:SC+_ret;2:SC+;1:closed;0:open;4:SC_gnd

  static const int PID_POLL_DC_CHARGE_SUPPORTED = 0x920F; // 0:Without DC Charge;1:With DC Charge

  static const int PID_POLL_COMPLETE_CHARGE_COUNTER = 0x9210;
  static const int PID_POLL_INCOMPLETE_CHARGE_COUNTER = 0x9215;

// 7bb	24	31	1	0	0		229216	629216	ff	Carrier to save SafetyMode1 root causeZtx_sfty_apv

// 	7bb	31	31	1	0	0		229217	629217	ff	Relay 3 control RELAY3_CMDWbx_rly_3_ctl	0:open;1:close
// 7bb	31	31	1	0	0		229218	629218	ff	Relay 4 control RELAY4_CMDWbx_rly_4_ctl	0:open;1:close
// 7bb	31	31	1	0	0		229219	629219	ff	Relay 4 diag feedback RELAY4_DGWbx_rly_4_ctl_dg

// 	7bb	24	55	.001	0	3		229243	629243	1ff	Cumulated energy in charge Zxx_kwh_chg
// 7bb	24	55	.001	0	3		229244	629244	1ff	Cumulated energy in discharge CS modeZxx_kwh_dch_cs
// 7bb	24	55	.001	0	3		229245	629245	1ff	Cumulated energy in discharge CD modeZxx_kwh_dch_cd
// 7bb	24	55	.001	0	3		229246	629246	1ff	Cumulated energy in regen CS modeZxx_kwh_rgn_cs
// 7bb	24	55	.001	0	3		229247	629247	1ff	Cumulated energy in regen CD modeZxx_kwh_rgn_cd

// 7bb	24	39	.000976563	0	3	V	229254	629254	ff	cell minimal voltage 20ms Wxx_cell_v_min_20ms
// 7bb	24	39	.000976563	0	3	V	229255	629255	ff	cell minimal voltage 20ms Wxx_cell_v_max_20ms

// 7bb	24	47	1	0	0		22925E	62925E	ff	Vehicle IDZxx_id_vhc
// 7bb	24	55	.01	0	2	km	22925F	62925F	1ff	Distance Totalizer vehicle

  static const int PID_POLL_CUMULATIVE_ENERGY_WHEN_CHARGING = 0x9243;
  static const int PID_POLL_CUMULATIVE_ENERGY_WHEN_DISCHARGING = 0x9245;
  static const int PID_POLL_CUMULATIVE_ENERGY_IN_REGEN = 0x9247;

  static const int PID_POLL_BALANCE_CHARGE_TOTAL = 0x924F;
  static const int PID_POLL_BALANCE_TIME_TOTAL = 0x9250;
  static const int PID_POLL_BALANCE_CHARGE_TOTAL_SLEEP = 0x9251;
  static const int PID_POLL_BALANCE_TIME_TOTAL_SLEEP = 0x9252;

  static const int PID_POLL_VEHICLE_TOTAL_DISTANCE = 0x925F;

// 7bb	24	103	1	0	0		22F012	62F012	2ff	VehicleManufacturerECUSoftwareNumber Ref C
// 7bb	24	103	1	0	0		22F111	62F111	2ff	VehicleManufacturerECUHardwareNumber_DAI
// 7bb	24	183	1	0	0		22F121	62F121	2ff	VehicleManufacturerECUSoftwareNumber_DAI
// 7bb	24	39	1	0	0		22F180	62F180	2ff	Boot VersionbootSoftwareIdentification
// 7bb	24	39	1	0	0		22F182	62F182	4ff	Calibration number
// 7bb	24	103	1	0	0		22F187	62F187	2ff	vehicleManufacturerSparePartNumber
// 7bb	24	103	1	0	0		22F188	62F188	2ff	vehicleManufacturerECUSoftwareNumber
// 7bb	24	63	1	0	0		22F18A	62F18A	2ff	systemSupplierIdentifier
// 7bb	24	183	1	0	0		22F18C	62F18C	2ff	ECUSerialNumber
// 7bb	24	103	1	0	0		22F191	62F191	2ff	vehicleManufacturerECUHardwareNumber
// 7bb	24	55	1	0	0		22F194	62F194	2ff	systemSupplierECUSoftwareNumber
// 7bb	24	55	1	0	0		22F195	62F195	2ff	systemSupplierECUSoftwareVersionNumber
// 7bb	24	103	1	0	0		22F196	62F196	2ff	OBD Approval
// 7bb	24	279	1	0	0		22F197	62F197	2ff	OBD EngineType
// 7bb	24	31	1	0	0		22F1A0	62F1A0	4ff	diagnosticVersion
// 7bb	24	103	1	0	0		22F1A1	62F1A1	2ff	Vehicle Manufacturer Spare Part Number Nissan

// 	7bb	24	31	.392156863	0	1	%	22F404	62F404	ff	PID 04 Calculated LOAD value
// 7bb	24	31	1	        40	0	°C	22F405	62F405	ff	PID 05 Engine coolant temperature
// 7bb	24	39	.25	       0	1	rpm	22F40C	62F40C	ff	PID 0C Engine RPM
// 7bb	24	31	1	         0	0	km/h	22F40D	62F40D	ff	PID 0D Vehicle speed sensor

// 7bb	24	31	1	0	0	kPa	22F433	62F433	ff	PID33 Barometric Pressure // N/A until 2024??

  // BMS heartbeat only (1 byte, no data)
  CAN_frame CMFA_1EA = {.FD = false,
                        .ext_ID = false,
                        .DLC = 1,
                        .ID = 0x1EA,
                        .data = {0x00}};
  // VCU → cluster (Speed, torque request, status)
  CAN_frame CMFA_125 = {.FD = false,
                        .ext_ID = false,
                        .DLC = 7,
                        .ID = 0x125,
                        .data = {0x7D, 0x7D, 0x7D, 0x07, 0x82, 0x6A, 0x8A}};
  // MCU/Inverter	(DC bus voltage/current + temperatures)
  CAN_frame CMFA_134 = {.FD = false,
                        .ext_ID = false,
                        .DLC = 8,
                        .ID = 0x134,
                        .data = {0x90, 0x8A, 0x7E, 0x3E, 0xB2, 0x4C, 0x80, 0x00}};
  // MCU/Inverter (motor RPM + torque)
  CAN_frame CMFA_135 = {.FD = false,
                        .ext_ID = false,
                        .DLC = 5,
                        .ID = 0x135,
                        .data = {0xD5, 0x85, 0x38, 0x80, 0x01}};
  // ADAS/BCM/EPS	(Body signals, steering, lights )
  CAN_frame CMFA_3D3 = {.FD = false,
                        .ext_ID = false,
                        .DLC = 8,
                        .ID = 0x3D3,
                        .data = {0x47, 0x30, 0x00, 0x02, 0x5D, 0x80, 0x5D, 0xE7}};
  // cluster/BCM (status frame)
  CAN_frame CMFA_59B = {.FD = false,
                        .ext_ID = false,
                        .DLC = 3,
                        .ID = 0x59B,
                        .data = {0x00, 0x02, 0x00}};
  CAN_frame CMFA_ACK = {.FD = false,
                        .ext_ID = false,
                        .DLC = 8,
                        .ID = 0x79B,
                        .data = {0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00}};
  CAN_frame CMFA_ISOTP_FC = CMFA_ACK;
  CAN_frame CMFA_POLLING_FRAME = {.FD = false,
                                  .ext_ID = false,
                                  .DLC = 8,
                                  .ID = 0x79B,
                                  .data = {0x03, 0x22, 0x90, 0x01, 0x00, 0x00, 0x00, 0x00}};
  CAN_frame CMFA_READ_DTC  = {.FD = false,
                              .ext_ID = false,
                              .DLC = 8,
                              .ID = 0x79B,
                              .data = {0x03, 0x19, 0x02, 0xFF, 0x00, 0x00, 0x00, 0x00}};
  CAN_frame CMFA_CLEAR_DTC = {.FD = false,
                              .ext_ID = false,
                              .DLC = 8,
                              .ID = 0x79B,
                              .data = {0x04, 0x14, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00}};
  CAN_frame CMFA_EXT_DIAG  = {.FD = false,
                              .ext_ID = false,
                              .DLC = 8,
                              .ID = 0x79B,
                              .data = {0x02, 0x10, 0x03, 0x00, 0x00, 0x00, 0x00, 0x00}};

  bool end_of_charge = false;
  uint8_t hvil_status = false;

  uint16_t soc_min_z = 0;
  uint16_t soc_max_z = 0;
  uint32_t pack_lifetime_mnt = 1; // To avoid devide by 0
  uint16_t dc_res_charge = 0;
  uint16_t dc_res_discharge = 0;
  uint32_t available_energy = 0;
  bool main_relay_control = false;
  bool precharge_relay_control = false;
  uint32_t busbar_fail = 0;
  uint32_t slaves_fail = 0;
  uint32_t battery_mileage_km = 0;
  uint8_t hvil1_raw_state = 0;
  uint8_t hvil2_raw_state = 0;
  uint8_t hvil3_raw_state = 0;
  uint8_t hvil4_raw_state = 0; // Not used on 2021-2023 model
  bool dc_charge_support = false;
  uint16_t complete_charge_counter = 0;
  uint16_t incomplete_charge_counter = 0;
  uint32_t balance_charge_total = 0;
  uint32_t balance_time_total = 0;
  uint32_t balance_charge_total_sleep = 0;
  uint32_t balance_time_total_sleep = 0;
  bool balance_any_cell = false;

  uint16_t soc_z = 0;
  uint16_t soc_u = 0;
  uint16_t instant_current = 0;
  uint16_t max_regen_power = 0;
  uint16_t max_discharge_power = 0;
  int16_t average_temperature = 0;
  int16_t minimum_temperature = 0;
  int16_t maximum_temperature = 0;
  uint16_t maximum_charge_power = 0;
  uint16_t SOH_available_power = 0;
  uint16_t SOH_generated_power = 0;
  uint32_t cells_voltage_sum = 2700; // dV
  uint16_t highest_cell_voltage_mv = 3700; //mV
  uint16_t lowest_cell_voltage_mv = 3700; //mV
  uint16_t lead_acid_voltage = 12000; // mV
  uint8_t highest_cell_voltage_number = 0;
  uint8_t lowest_cell_voltage_number = 0;
  uint64_t cumulative_energy_when_discharging = 0;
  uint64_t cumulative_energy_when_charging = 0;
  uint64_t cumulative_energy_in_regen = 0;
  uint16_t soh_average = 10000; 
  uint16_t pack_voltage_can = 3000; // dV
  uint32_t poll_pid = PID_POLL_SOH_AVERAGE;
  uint16_t poll_pid2 = 0x8800;
  uint16_t pid_reply = 0;

  // UDS Multi-Frame Reception Context
  CMFA_UDS_CONTEXT gUDSContext = {
      {},     // UDS_buffer - zero-initialized array
      0,      // UDS_bytesReceived
      0,      // UDS_expectedLength
      0,      // UDS_sequenceNumber
      false,  // UDS_inProgress
      0,      // UDS_sessionID
      0,      // UDS_subfuncID
      0,      // UDS_dataID
      0,      // receivedInBatch
      0       // UDS_lastFrameMillis
  };

  uint8_t counter_10ms = 0;
  // VCU → cluster status
  uint8_t content_125[16] = {0x07, 0x0C, 0x01, 0x06, 0x0B, 0x00, 0x05, 0x0A,
                             0x0F, 0x04, 0x09, 0x0E, 0x03, 0x08, 0x0D, 0x02};
  // motor RPM + torque
  uint8_t content_135[16] = {0x85, 0xD5, 0x25, 0x75, 0xC5, 0x15, 0x65, 0xB5,
                             0x05, 0x55, 0xA5, 0xF5, 0x45, 0x95, 0xE5, 0x35};

  unsigned long previousMillis1000ms = 0;
  unsigned long previousMillis200ms = 0;
  unsigned long previousMillis100ms = 0;
  unsigned long previousMillis10ms = 0;

  static const int MAXSOC = 9000;  //90.00 Raw SOC displays this value when battery is at 100%
  static const int MINSOC = 500;   //5.00 Raw SOC displays this value when battery is at 0%

  static const int MAX_DTC_COUNT = 30;    // Maximum number of DTCs to store/display

  uint8_t heartbeat = 0;   //Alternates between 0x55 and 0xAA every 5th frame
  uint8_t heartbeat2 = 0;  //Alternates between 0x55 and 0xAA every 5th frame
  uint32_t SOC_raw = 0;
  uint16_t SOH = 99;
  int16_t current = 0;
  uint16_t pack_voltage = 500;
  int16_t highest_cell_temperature = 0;
  int16_t lowest_cell_temperature = 0;
  uint32_t discharge_power_w = 0;
  uint32_t charge_power_w = 0;
};

#endif
