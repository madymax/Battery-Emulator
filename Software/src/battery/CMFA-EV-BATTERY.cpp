#include "CMFA-EV-BATTERY.h"
#include <cstring>  //unit tests memcpy
#include "../communication/can/comm_can.h"
#include "../datalayer/datalayer.h"
#include "../datalayer/datalayer_extended.h"
#include "../devboard/utils/events.h"
#include "BATTERIES.h"

/* The raw SOC value sits at 90% when the battery is full, so we should report back 100% once this value is reached
Same goes for low point, when 10% is reached we report 0% */

uint16_t CmfaEvBattery::rescale_raw_SOC(uint32_t raw_SOC) {

  uint32_t calc_soc;
  calc_soc = (raw_SOC * 0.25);
  if (calc_soc > MAXSOC) {  //Constrain if needed
    calc_soc = MAXSOC;
  }
  if (calc_soc < MINSOC) {  //Constrain if needed
    calc_soc = MINSOC;
  }
  // Perform scaling between the two points
  calc_soc = 10000 * (calc_soc - MINSOC);
  calc_soc = calc_soc / (MAXSOC - MINSOC);

  return (uint16_t)calc_soc;
}

// UDS Multi-Frame Reception Helper Functions
void CmfaEvBattery::startUDSMultiFrameReception(uint16_t totalLength, uint8_t subfuncID, uint16_t dataID) {
  gUDSContext.UDS_inProgress = true;
  gUDSContext.UDS_expectedLength = totalLength;
  gUDSContext.UDS_bytesReceived = 0;
  gUDSContext.UDS_sequenceNumber = 1;  // Next expected sequence is 1
  gUDSContext.UDS_subfuncID = subfuncID;
  gUDSContext.UDS_dataID = dataID;
  memset(gUDSContext.UDS_buffer, 0, sizeof(gUDSContext.UDS_buffer));
  gUDSContext.UDS_lastFrameMillis = millis();  // Track timeout
}

bool CmfaEvBattery::storeUDSPayload(const uint8_t* payload, uint8_t length) {
  if (gUDSContext.UDS_bytesReceived + length > sizeof(gUDSContext.UDS_buffer)) {
    logging.println("UDS buffer overflow prevented");
    gUDSContext.UDS_inProgress = false;
    return false;
  }

  memcpy(&gUDSContext.UDS_buffer[gUDSContext.UDS_bytesReceived], payload, length);
  gUDSContext.UDS_bytesReceived += length;
  gUDSContext.UDS_lastFrameMillis = millis();

  // If we've reached or exceeded the expected length, mark complete
  if (gUDSContext.UDS_bytesReceived >= gUDSContext.UDS_expectedLength) {
    gUDSContext.UDS_inProgress = false;
  }
  return true;
}

bool CmfaEvBattery::isUDSMessageComplete() {
  return (!gUDSContext.UDS_inProgress && gUDSContext.UDS_bytesReceived > 0);
}

int CmfaEvBattery::find_last_data_byte(const uint8_t *buf, uint8_t len) {
    for (uint8_t i = len - 1; i > 0; i--) {
        if (buf[i] != 0xAA) {
            return i;   // last real data byte
        }
    }
    return -1;
}

void CmfaEvBattery::parseDTCResponse() {
  // Check for negative response
  if (gUDSContext.UDS_buffer[0] == 0x7F) {
    logging.print("DTC request rejected by battery. Reason code: 0x");
    logging.print(gUDSContext.UDS_buffer[2], HEX);
    logging.println();
    datalayer_extended.CMFAEV.dtc_read_failed = true;
    datalayer_extended.CMFAEV.dtc_read_in_progress = false;
    return;
  }
  // Check for DTC response
  if (gUDSContext.UDS_buffer[0] != 0x59 || gUDSContext.UDS_buffer[1] != 0x02) {
    logging.println("Invalid DTC response header");
    datalayer_extended.CMFAEV.dtc_read_failed = true;
    datalayer_extended.CMFAEV.dtc_read_in_progress = false;
    return;
  }

  int dtcStartIndex = 3;  // Skip 59 02 FF
  int availableBytes = gUDSContext.UDS_bytesReceived - dtcStartIndex;
  int maxDtcCount = availableBytes / 4;

  if (maxDtcCount > MAX_DTC_COUNT) {
    maxDtcCount = MAX_DTC_COUNT;
    logging.println("DTC count exceeds buffer, truncating");
  }

  int validDtcCount = 0;  // Track actual valid DTCs

  logging.print("Parsing DTCs (max ");
  logging.print(maxDtcCount);
  logging.println("):");

  for (int i = 0; i < maxDtcCount; i++) {
    int offset = dtcStartIndex + (i * 4);

    // Bounds check
    if (offset + 3 > gUDSContext.UDS_bytesReceived) {
      logging.println("DTC parsing: offset exceeds buffer, stopping");
      break;
    }

    // Combine 3 bytes into single uint32
    uint32_t dtcCode = ((uint32_t)gUDSContext.UDS_buffer[offset] << 16) |
                       ((uint32_t)gUDSContext.UDS_buffer[offset + 1] << 8) |
                       (uint32_t)gUDSContext.UDS_buffer[offset + 2];

    uint8_t dtcStatus = gUDSContext.UDS_buffer[offset + 3];

    // Skip invalid DTCs (0x000000 or status 0x00)
    if (dtcCode == 0x000000 || dtcStatus == 0x00) {
      logging.print("  Skipping invalid DTC at offset ");
      logging.println(offset);
      continue;  // Don't store this one
    }

    // Store valid DTC
    datalayer_extended.CMFAEV.dtc_codes[validDtcCount] = dtcCode;
    datalayer_extended.CMFAEV.dtc_status[validDtcCount] = dtcStatus;

    // Log each DTC for debugging
    logging.print("  DTC #");
    logging.print(validDtcCount + 1);
    logging.print(": 0x");
    if (dtcCode < 0x100000)
      logging.print("0");
    if (dtcCode < 0x10000)
      logging.print("0");
    if (dtcCode < 0x1000)
      logging.print("0");
    if (dtcCode < 0x100)
      logging.print("0");
    if (dtcCode < 0x10)
      logging.print("0");
    logging.print(dtcCode, HEX);
    logging.print(" Status: 0x");
    if (dtcStatus < 0x10)
      logging.print("0");
    logging.print(dtcStatus, HEX);
    logging.println();

    validDtcCount++;  // Increment only for valid DTCs
  }

  datalayer_extended.CMFAEV.dtc_count = validDtcCount;  // Store actual count

  logging.print("Total valid DTCs: ");
  logging.println(validDtcCount);

  datalayer_extended.CMFAEV.dtc_last_read_millis = millis();
  datalayer_extended.CMFAEV.dtc_read_failed = false;
  datalayer_extended.CMFAEV.dtc_read_in_progress = false;
}

void CmfaEvBattery::parseDIDResponse() {
  // Check for negative response
  if (gUDSContext.UDS_buffer[0] == 0x7F) {
    logging.print("DID request rejected by battery. Reason code: 0x");
    logging.print(gUDSContext.UDS_buffer[2], HEX);
    logging.println();
    return;
  }

  if (gUDSContext.UDS_sessionID != 0x62) {
    logging.println("Invalid DID response header");
    return;
  }

  logging.printf("Processing SID = %X SubF=%X DID=%X \r\n",
                gUDSContext.UDS_sessionID, gUDSContext.UDS_subfuncID, gUDSContext.UDS_dataID);

  uint8_t *buf = &gUDSContext.UDS_buffer[0];
  uint8_t len = gUDSContext.UDS_bytesReceived;
  uint8_t cellnumber = 0;
  int b = 0;
  pid_reply = gUDSContext.UDS_dataID;

  switch (pid_reply) {
    case PID_POLL_SOCZ:
      soc_z = ((uint16_t)(buf[3]) + 6) * 50; // 9900 = 99.00%
      break;
    case PID_POLL_USOC:
      soc_u = (uint16_t)((buf[3] << 8) | buf[4]); // 9900 = 99.00%
      break;
    case PID_POLL_SOH_AVERAGE:
      soh_average = (uint16_t)((buf[3] << 8) | buf[4]); // 9900 = 99.00%
      break;
    case PID_POLL_PACK_VOLTAGE_CAN:
      pack_voltage_can = (uint16_t)((buf[3] << 8) | buf[4]); // dV
      break;
    case PID_POLL_CELLS_VOLTAGE_SUM:
      cells_voltage_sum =
          (uint32_t)((buf[3] << 24) | (buf[4] << 16) | (buf[5] << 8) | (buf[6])) * 0.009765625; // dV
      break;
    case PID_POLL_HIGHEST_CELL_VOLTAGE:
      highest_cell_voltage_mv = (uint16_t)((buf[3] << 8) | buf[4]) * 0.976563; // mV
      break;
    case PID_POLL_CELL_NUMBER_HIGHEST_VOLTAGE:
      highest_cell_voltage_number = buf[3];
      break;
    case PID_POLL_LOWEST_CELL_VOLTAGE:
      lowest_cell_voltage_mv = (uint16_t)((buf[3] << 8) | buf[4]) * 0.976563; // mV
      break;
    case PID_POLL_CELL_NUMBER_LOWEST_VOLTAGE:
      lowest_cell_voltage_number = buf[3];
      break;
    case PID_POLL_CURRENT_OFFSET:
      //current_offset = // N/A
      break;
    case PID_POLL_INSTANT_CURRENT:
      instant_current = ((uint16_t)((buf[3] << 8) | buf[4]) - 48000) * 25; // mA
      break;
    case PID_POLL_MAX_REGEN:
      max_regen_power = (uint16_t)((buf[3] << 8) | buf[4]) * 10; // W
      break;
    case PID_POLL_MAX_DISCHARGE_POWER:
      max_discharge_power = (uint16_t)((buf[3] << 8) | buf[4]) * 10; // W
      break;
    case PID_POLL_12V_BATTERY:
      lead_acid_voltage = (uint16_t)((buf[3] << 8) | buf[4]) * 0.976563; // mV
      break;
    case PID_POLL_AVERAGE_TEMPERATURE:
      average_temperature = ((((buf[3] << 8) | buf[4]) - 640) * 0.625); // dC
      break;
    case PID_POLL_MIN_TEMPERATURE:
      minimum_temperature = ((((buf[3] << 8) | buf[4]) - 640) * 0.625); // dC
      break;
    case PID_POLL_MAX_TEMPERATURE:
      maximum_temperature = ((((buf[3] << 8) | buf[4]) - 640) * 0.625); // dC
      break;
    case PID_POLL_MAX_CHARGE_POWER:
      maximum_charge_power = (uint16_t)((buf[3] << 8) | buf[4]) * 10; // W
      break;
    case PID_POLL_END_OF_CHARGE_FLAG:
      end_of_charge = (bool)buf[3];
      break;
    case PID_POLL_INTERLOCK_FLAG:
      hvil_status = buf[3];
      break;
    case PID_POLL_SOH_AVAILABLE_POWER_CALCULATION:
      SOH_available_power = (uint16_t)((buf[3] << 8) | buf[4]);
      break;
    case PID_POLL_SOH_GENERATED_POWER_CALCULATION:
      SOH_generated_power = (uint16_t)((buf[3] << 8) | buf[4]);
      break;
    case PID_POLL_CUMULATIVE_ENERGY_WHEN_DISCHARGING:
      cumulative_energy_when_discharging = (uint64_t)((buf[3] << 24) | (buf[4] << 16) |
                                                      (buf[5] << 8) | (buf[6]));
      break;
    case PID_POLL_CUMULATIVE_ENERGY_WHEN_CHARGING:
      cumulative_energy_when_charging = (uint64_t)((buf[3] << 24) | (buf[4] << 16) |
                                                  (buf[5] << 8) | (buf[6]));
      break;
    case PID_POLL_CUMULATIVE_ENERGY_IN_REGEN:
      cumulative_energy_in_regen = (uint64_t)((buf[3] << 24) | (buf[4] << 16) |
                                              (buf[5] << 8) | (buf[6]));
      break;
    case PID_POLL_BALANCE_CHARGE_TOTAL:
      balance_charge_total = (((uint32_t)((buf[3] << 24) | (buf[4] << 16) |
                                          (buf[5] << 8) | (buf[6]))) - 2147483648) * 0.0009765625;
      break;
    case PID_POLL_BALANCE_TIME_TOTAL:
      balance_time_total = (((uint32_t)((buf[3] << 24) | (buf[4] << 16) |
                                          (buf[5] << 8) | (buf[6]))) - 2147483648) * 0.0009765625;
      break;
    case PID_POLL_BALANCE_CHARGE_TOTAL_SLEEP:
      balance_charge_total_sleep = (((uint32_t)((buf[3] << 24) | (buf[4] << 16) |
                                          (buf[5] << 8) | (buf[6]))) - 2147483648) * 0.0009765625;
      break;
    case PID_POLL_BALANCE_TIME_TOTAL_SLEEP:
      balance_time_total_sleep = (((uint32_t)((buf[3] << 24) | (buf[4] << 16) |
                                          (buf[5] << 8) | (buf[6]))) - 2147483648) * 0.0009765625;
      break;
    case PID_POLL_DIDS_SUPPORTED_IN_RANGE_9041_9060:
      break;
    case PID_POLL_SLAVES_FAIL: // 0:No failure;1:Slave no.21 failure (Lower 21 bits - 3 bytes):
      slaves_fail = (uint32_t)((buf[3] << 16) | (buf[4] << 8) | (buf[5]));
      break;
    case PID_POLL_BALANCING_SW_STATUS:
        b = find_last_data_byte(buf, len);
        if (b < 3) {
          break;
        }
        cellnumber = 0;
        balance_any_cell = false; // Reset the value before we start
        // walk bytes backwards: last real byte is buf[b]
        for (; b >= 0 && cellnumber < MAX_AMOUNT_CELLS; b--) {
          for (uint8_t bit = 0; bit < 8 && cellnumber < MAX_AMOUNT_CELLS; bit++) {
            datalayer_battery->status.cell_balancing_status[cellnumber++] = (buf[b] >> bit) & 1;   // LSB → MSB
            balance_any_cell |= datalayer_battery->status.cell_balancing_status[cellnumber - 1];
          }
        }
      break;
    case PID_POLL_SOC_MIN_Z: // x * 0.01 = %
      soc_min_z = ((uint16_t)((buf[3] << 8) | buf[4]) - 300);
      break;
    case PID_POLL_SOC_MAX_Z: // x * 0.01 = %
      soc_max_z = ((uint16_t)((buf[3] << 8) | buf[4]) - 300);
      break;
    case PID_POLL_PACK_LIFE_TIME_MIN: // minutes
      pack_lifetime_mnt = (uint32_t)((buf[3] << 16) | (buf[4] << 8) | (buf[5]));
      break;
    case PID_POLL_DC_RES_CHARGE: // mOhm (x0.25) (per pack?):
      dc_res_charge = ((uint16_t)((buf[3] << 8) | buf[4]) * 250 ); // uOhm
      break;
    case PID_POLL_DC_RES_DISCHARGE: // mOhm (x0.25) (per pack?):
      dc_res_discharge = ((uint16_t)((buf[3] << 8) | buf[4]) * 250 );  // uOhm
      break;
    case PID_POLL_AVAILABLE_ENERGY: // KW (x0.001):
      available_energy = (uint32_t)((buf[3] << 16) | (buf[4] << 8) | buf[5]); // W
      break;
    case PID_POLL_FAN_CONTROL_REQ: // % (x 0.5):
      break;
    case PID_POLL_MAIN_RELAY: // 0:Openning control;1:Closing control:
      main_relay_control = (bool)buf[3];
      break;
    case PID_POLL_PRECHARGE_RELAY: // 0:Openning control;1:Closing control:
      precharge_relay_control = (bool)buf[3];
      break;
    case PID_POLL_BUSBAR_FAIL: // 0:No failure;1:Busbar no.20 failure (Lower 20 bits - 3 bytes):
      busbar_fail = (uint32_t)((buf[3] << 16) | (buf[4] << 8) | (buf[5]));
      break;
    case PID_POLL_BATTERY_MILEAGE_KM:
      battery_mileage_km = (((uint32_t)((buf[3] << 24) | (buf[4] << 16) | (buf[5] << 8) | (buf[6]))) - 2147483648) * 0.03125;
      break;
    case PID_POLL_INTERLOCK_1: // 3:SC+_ret;2:SC+;1:closed;0:open;4:SC_gnd:
      hvil1_raw_state = buf[3];
      break;
    case PID_POLL_INTERLOCK_2: // 3:SC+_ret;2:SC+;1:closed;0:open;4:SC_gnd:
      hvil2_raw_state = buf[3];
      break;
    case PID_POLL_INTERLOCK_3: // 3:SC+_ret;2:SC+;1:closed;0:open;4:SC_gnd:
      hvil3_raw_state = buf[3];
      break;
    case PID_POLL_INTERLOCK_4: // 3:SC+_ret;2:SC+;1:closed;0:open;4:SC_gnd: // Not Used
      hvil4_raw_state = buf[3];
      break;
    case PID_POLL_DC_CHARGE_SUPPORTED: // 0:Without DC Charge;1:With DC Charge:
      dc_charge_support = (bool)buf[3];
      break;
    case PID_POLL_COMPLETE_CHARGE_COUNTER:
      complete_charge_counter = (uint16_t)((buf[3] << 8) | buf[4]);
      break;
    case PID_POLL_INCOMPLETE_CHARGE_COUNTER:
      incomplete_charge_counter = (uint16_t)((buf[3] << 8) | buf[4]);
      break;
    default:  // Unknown pid_reply, or a cellvoltage
      logging.printf("Pid poll default case %x\r\n" , pid_reply);
      cellnumber = 0;
      if (pid_reply >= PID_POLL_CELL_1 && pid_reply <= PID_POLL_CELL_31) {  // Cellvoltage PID reply
        cellnumber = (pid_reply - PID_POLL_CELL_1);
      } else if (pid_reply >= PID_POLL_CELL_32 && pid_reply <= PID_POLL_CELL_62) {
        cellnumber = (pid_reply - PID_POLL_CELL_1) - 1;
      } else if (pid_reply >= PID_POLL_CELL_63 && pid_reply <= PID_POLL_CELL_72) {
        cellnumber = (pid_reply - PID_POLL_CELL_1) - 2;
      } else {  // Unknown pid_reply
        break;
      }

      if (cellnumber < MAX_AMOUNT_CELLS) {  // Prevent out of bounds array write
        uint16_t cellvoltage_reading = (uint16_t)((buf[3] << 8) | buf[4]);
        if (cellvoltage_reading == 0) {
          // Blown fuse/celltap. Force value to 10mV so user sees this in cellmonitor page. Also fire event
          cellvoltage_reading = 10;
          set_event(EVENT_BATTERY_FUSE, cellnumber);
        }
        datalayer_battery->status.cell_voltages_mV[cellnumber] = cellvoltage_reading * 0.976563; // mV 
        logging.printf("CellVoltage cell %d\r\n" , cellnumber);
      }
      break;
  }
}

void CmfaEvBattery::handleISOTPFrame(CAN_frame& rx_frame) {
  uint8_t pciByte = rx_frame.data.u8[0];  // e.g., 0x10, 0x21, etc.
  uint8_t pciType = pciByte >> 4;         // top nibble => 0=SF,1=FF,2=CF,3=FC

  // // Only process multi-frame ISO-TP messages (FF and CF)
  // // Single-frame messages are handled directly in case 0x7BB
  // if (pciType != 0x1 && pciType != 0x2) {
  //   return;  // Not a multi-frame message we care about
  // }

  switch (pciType) {
    case 0x0: {
      // Single Frame (SF)
      // RX0 7BB [8] 05 62 90 21 0E A9 AA AA
      uint16_t totalLength = (uint16_t)pciByte & 0x0F;

      uint8_t serviceResponse = rx_frame.data.u8[1];  // Service response byte (0x59, 0x62, etc.)
      uint8_t subfuncID;
      uint16_t dataID;

      gUDSContext.UDS_sessionID = serviceResponse;

      // Determine which byte to use for sub function ID based on service response
      if (serviceResponse == 0x59) {
        // Standard UDS DTC response (0x19 -> 0x59)
        // Use sub-function byte as subfuncID
        subfuncID = rx_frame.data.u8[2];  // 0x02 for reportDTCByStatusMask
        dataID = 0;
      } else {
        // proprietary responses (0x22 -> 0x62)
        // Use parameter byte 2 and 3 as dataID
        subfuncID = 0;
        dataID = (uint16_t)((rx_frame.data.u8[2] << 8) | rx_frame.data.u8[3]);
      }

      // logging.print("FF arrived! subfuncID=0x");
      // logging.print(subfuncID, HEX);
      // logging.print(", totalLength=");
      // logging.println(totalLength);

      // Start the single-frame reception (reuse multi-frame function)
      startUDSMultiFrameReception(totalLength, subfuncID, dataID);
      gUDSContext.receivedInBatch = 0;  // Reset batch count

      // Store the FF payload (starts at data[1])
      const uint8_t* ffPayload = &rx_frame.data.u8[1];
      uint8_t ffPayloadSize = rx_frame.DLC - 1;
      storeUDSPayload(ffPayload, ffPayloadSize);
      isUDSMessageComplete();
      break;
    }

    case 0x1: {
      // First Frame (FF)
      // RX0 7BB [8] 10 CB 59 0A FF 1B 0C 98
      uint8_t pciLower = pciByte & 0x0F;
      uint16_t totalLength = ((uint16_t)pciLower << 8) | rx_frame.data.u8[1];

      uint8_t serviceResponse = rx_frame.data.u8[2];  // Service response byte (0x59, 0x62, etc.)
      uint8_t subfuncID;
      uint16_t dataID;

      gUDSContext.UDS_sessionID = serviceResponse;

      // Determine which byte to use for sub function ID based on service response
      if (serviceResponse == 0x59) {
        // Standard UDS DTC response (0x19 -> 0x59)
        // Use sub-function byte as subfuncID
        subfuncID = rx_frame.data.u8[3];  // 0x02 for reportDTCByStatusMask
        dataID = 0;
      } else {
        // proprietary responses (0x22 -> 0x62)
        // Use parameter byte 3 and 4 as dataID
        subfuncID = 0;
        dataID = (uint16_t)((rx_frame.data.u8[3] << 8) | rx_frame.data.u8[4]);
      }

      // logging.print("FF arrived! subfuncID=0x");
      // logging.print(subfuncID, HEX);
      // logging.print(", totalLength=");
      // logging.println(totalLength);

      // Start the multi-frame reception
      startUDSMultiFrameReception(totalLength, subfuncID, dataID);
      gUDSContext.receivedInBatch = 0;  // Reset batch count

      // Store the FF payload (starts at data[2])
      const uint8_t* ffPayload = &rx_frame.data.u8[2];
      uint8_t ffPayloadSize = rx_frame.DLC - 2;
      storeUDSPayload(ffPayload, ffPayloadSize);

      // Request continuation
      transmit_can_frame(&CMFA_ISOTP_FC);
      break;
    }

    case 0x2: {
      // Consecutive Frame (CF)
      // RX0 7BB [8] 21 00 1B 01 F1 00 1B 01 - (21-2F,21 again)
      if (!gUDSContext.UDS_inProgress) {
        logging.println("Unexpected CF - not in progress");
        return;  // Unexpected CF, ignore
      }

      uint8_t seq = pciByte & 0x0F;

      if (gUDSContext.UDS_sequenceNumber != seq) {
        logging.printf("Unexpected CF - incorrect sequence %x != %x\n", gUDSContext.UDS_sequenceNumber, seq);
        return;  // Unexpected CF, ignore
      }

      if(gUDSContext.UDS_sequenceNumber == 0xF){
          gUDSContext.UDS_sequenceNumber = 0x0;
      }else{
          gUDSContext.UDS_sequenceNumber++;
      }
      // logging.print("CF seq=0x");
      // logging.print(seq, HEX);
      // logging.print(" for subfuncID=0x");
      // logging.println(gUDSContext.UDS_subfuncID, HEX);

      // Store CF payload (starts at byte 1)
      storeUDSPayload(&rx_frame.data.u8[1], rx_frame.DLC - 1);

      // Increment batch counter
      gUDSContext.receivedInBatch++;

      // logging.print("After CF, UDS_bytesReceived=");
      // logging.println(gUDSContext.UDS_bytesReceived);

      // Check if batch is complete (iX uses 2 frames per batch based on 0x30 0x00 0x02)
      // MAX: TODO: Is this needed on CMFA? Not needed.
      // if (gUDSContext.receivedInBatch >= 2) {
      //   //logging.println("Batch complete - requesting continue frame...");
      //   transmit_can_frame(&CMFA_ISOTP_FC);
      //   gUDSContext.receivedInBatch = 0;
      // }
      break;
    }
  }
}

void CmfaEvBattery::processCompletedUDSResponse() {
  // uint8_t* buf = gUDSContext.UDS_buffer;
  uint16_t len = gUDSContext.UDS_bytesReceived;

  // Route based on subfuncID or DID (set during Single or First Frame reception)
  if ((gUDSContext.UDS_sessionID == 0x59) && (gUDSContext.UDS_subfuncID == 0x02)) {
    // DTC Response (0x19 0x02 -> 0x59 0x02)
    logging.println("=== DTC Response Received ===");
    logging.print("Total bytes: ");
    logging.println(len);
    parseDTCResponse();
  }
  else if (gUDSContext.UDS_sessionID == 0x62) {
    logging.println("=== DID Response Received ===");
    logging.print("Total bytes: ");
    logging.println(len);
    parseDIDResponse();
  }
  else if (gUDSContext.UDS_sessionID == 0x7F) { // Not real Session ID
    logging.println("=== NEGATIVE Response Received ===");
    logging.printf("SID: %x :", gUDSContext.UDS_buffer[1]);
    switch (gUDSContext.UDS_buffer[2])
    {
    case 0x31:
      logging.printf("NRC(%x) = Request Out Of Range\r\n", gUDSContext.UDS_buffer[2]);
      break;
    default:
      logging.printf("NRC(%x) = UNKNOWN\r\n", gUDSContext.UDS_buffer[2]);
      break;
    }
  }
  else {
    ; //unknown UDS message
    logging.println("=== UNKNOWN UDS Message Received ===");
    logging.print("Total bytes: ");
    logging.println(len);

    for (size_t i = 0; i < len; i++) {
        // %02X ensures two digits with leading zero if needed
        logging.printf("%02X", gUDSContext.UDS_buffer[i]);
        if (i < len - 1) {
            logging.printf(" "); // space between bytes
        }
    }
    logging.println("");
  }

  // madymax: TODO: Parse other messages

  // Reset buffer after processing
  gUDSContext.UDS_bytesReceived = 0;
}

void CmfaEvBattery::update_values() {  // This function maps all the values fetched via CAN to the correct parameters used for modbus
  datalayer_battery->status.soh_pptt = (SOH * 100);

  datalayer_battery->status.real_soc = rescale_raw_SOC(SOC_raw);

  datalayer_battery->status.current_dA = current * 10;

  datalayer_battery->status.voltage_dV = pack_voltage * 5;

  datalayer_battery->info.total_capacity_Wh = 27000;

  // Calculate the remaining Wh amount from SOC% and max Wh value.
  datalayer_battery->status.remaining_capacity_Wh = static_cast<uint32_t>(
      (static_cast<double>(datalayer_battery->status.real_soc) / 10000) * datalayer_battery->info.total_capacity_Wh);

  datalayer_battery->status.max_discharge_power_W = discharge_power_w;

  datalayer_battery->status.max_charge_power_W = charge_power_w;

  datalayer_battery->status.temperature_min_dC = (lowest_cell_temperature * 10);

  datalayer_battery->status.temperature_max_dC = (highest_cell_temperature * 10);

  datalayer_battery->status.cell_min_voltage_mV = lowest_cell_voltage_mv;

  datalayer_battery->status.cell_max_voltage_mV = highest_cell_voltage_mv;

  if (lead_acid_voltage < 11000) {  //11.000V
    set_event(EVENT_12V_LOW, lead_acid_voltage);
  }

  if (!battery2) {  // Avoid pointer crash on double bat, not sure why this wont work
    // Update webserver datalayer
    datalayer_cmfa->soc_u = soc_u;
    datalayer_cmfa->soc_z = soc_z;
    datalayer_cmfa->lead_acid_voltage = lead_acid_voltage;
    datalayer_cmfa->highest_cell_voltage_number = highest_cell_voltage_number;
    datalayer_cmfa->lowest_cell_voltage_number = lowest_cell_voltage_number;
    datalayer_cmfa->max_regen_power = max_regen_power;
    datalayer_cmfa->max_discharge_power = max_discharge_power;
    datalayer_cmfa->average_temperature = average_temperature;
    datalayer_cmfa->minimum_temperature = minimum_temperature;
    datalayer_cmfa->maximum_temperature = maximum_temperature;
    datalayer_cmfa->maximum_charge_power = maximum_charge_power;
    datalayer_cmfa->SOH_available_power = SOH_available_power;
    datalayer_cmfa->SOH_generated_power = SOH_generated_power;
    datalayer_cmfa->cumulative_energy_when_discharging = cumulative_energy_when_discharging;
    datalayer_cmfa->cumulative_energy_when_charging = cumulative_energy_when_charging;
    datalayer_cmfa->cumulative_energy_in_regen = cumulative_energy_in_regen;
    datalayer_cmfa->soh_average = soh_average;
    datalayer_cmfa->cells_voltage_sum = cells_voltage_sum;
  }
}

void CmfaEvBattery::handle_incoming_can_frame(CAN_frame rx_frame) {
  switch (rx_frame.ID) {  // These frames are transmitted by the battery
    // inverter electrical data
    case 0x127:           // 10ms , Same structure as old Zoe 0x155 message!
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      current = (((((rx_frame.data.u8[1] & 0x0F) << 8) | rx_frame.data.u8[2]) * 0.25) - 500);
      SOC_raw = ((rx_frame.data.u8[4] << 8) | rx_frame.data.u8[5]);
      pack_voltage = (((rx_frame.data.u8[6] & 0x03) << 8) | rx_frame.data.u8[7]);
      break;
    // steering, ABS, body
    case 0x3D6:  // 100ms, Same structure as old Zoe 0x424 message!
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      charge_power_w = rx_frame.data.u8[2] * 500;
      discharge_power_w = rx_frame.data.u8[3] * 500;
      lowest_cell_temperature = (rx_frame.data.u8[4] - 40);
      SOH = rx_frame.data.u8[5];
      heartbeat = rx_frame.data.u8[6];
      highest_cell_temperature = (rx_frame.data.u8[7] - 40);
      break;
      // steering, ABS, body
    case 0x3D7:  // 100ms
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
      // steering, ABS, body
    case 0x3D8:  // 100ms
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      // counter_3D8 = rx_frame.data.u8[3]; //?
      // CRC_3D8 = rx_frame.data.u8[4]; //?
      break;
    case 0x43C:  // 100ms
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      heartbeat2 = rx_frame.data.u8[2];  // Alternates between 0x55 and 0xAA every 5th frame
      break;
    case 0x431:  // 100ms
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      // byte0 9C always
      // byte1 40 always
      break;
    case 0x5A9:
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x5AB:
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x5C8:
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x5E1:
      datalayer_battery->status.CAN_battery_still_alive = CAN_STILL_ALIVE;
      break;
    case 0x7BB:                           // Reply from battery
      // madymax: TODO: Should we move this ?
      // Multi Frame UDS
      // //if ((rx_frame.data.u8[0] == 0x10) {
      // if ((rx_frame.data.u8[0] & 0xF0) > 0x00) {  //PID header
      //   transmit_can_frame(&CMFA_ACK);
      // }
      // Single Frame UDS

      // Handle ISO-TP single-frame and multi-frame messages
      handleISOTPFrame(rx_frame);

      // Check if complete UDS response is ready to process
      if (isUDSMessageComplete()) {
        processCompletedUDSResponse();
      }
      break;
    default:
      break;
  }
}

void CmfaEvBattery::transmit_can(unsigned long currentMillis) {
  // Send 10ms CAN Message
  if (currentMillis - previousMillis10ms >= INTERVAL_10_MS) {
    previousMillis10ms = currentMillis;
    // BMS heartbeat only (1 byte, no data)
    transmit_can_frame(&CMFA_1EA);
    // motor RPM + torque
    transmit_can_frame(&CMFA_135);
    // DC bus voltage/current + temperatures
    transmit_can_frame(&CMFA_134);
    // VCU → cluster status
    transmit_can_frame(&CMFA_125);

    CMFA_135.data.u8[1] = content_135[counter_10ms];
    CMFA_125.data.u8[3] = content_125[counter_10ms];
    counter_10ms = (counter_10ms + 1) % 16;  // counter_10ms cycles between 0-1-2-3..15-0-1...

    // ///- Testing Purposes (scan Bus) -///
    // CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(poll_pid2 >> 8);
    // CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)poll_pid2;
    // poll_pid2++;
    // transmit_can_frame(&CMFA_POLLING_FRAME);
    // ///

  }
  // Send 100ms CAN Message
  if (currentMillis - previousMillis100ms >= INTERVAL_100_MS) {
    previousMillis100ms = currentMillis;

    // cluster/BCM  (status frame)
    transmit_can_frame(&CMFA_59B);
    // ADAS / BCM / EPS	(Body signals, steering, lights)
    transmit_can_frame(&CMFA_3D3);
  }
  // Send 200ms message
  if (currentMillis - previousMillis200ms >= INTERVAL_200_MS) {
    previousMillis200ms = currentMillis;

    switch (poll_pid) {
      case PID_POLL_SOH_AVERAGE:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_SOH_AVERAGE >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_SOH_AVERAGE;
        poll_pid = PID_POLL_PACK_VOLTAGE_CAN;
        break;
      case PID_POLL_PACK_VOLTAGE_CAN:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_PACK_VOLTAGE_CAN >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_PACK_VOLTAGE_CAN;
        poll_pid = PID_POLL_CELLS_VOLTAGE_SUM;
        break;
      case PID_POLL_CELLS_VOLTAGE_SUM:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELLS_VOLTAGE_SUM >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELLS_VOLTAGE_SUM;
        poll_pid = PID_POLL_HIGHEST_CELL_VOLTAGE;
        break;
      case PID_POLL_HIGHEST_CELL_VOLTAGE:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_HIGHEST_CELL_VOLTAGE >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_HIGHEST_CELL_VOLTAGE;
        poll_pid = PID_POLL_LOWEST_CELL_VOLTAGE;
        break;
      case PID_POLL_LOWEST_CELL_VOLTAGE:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_LOWEST_CELL_VOLTAGE >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_LOWEST_CELL_VOLTAGE;
        poll_pid = PID_POLL_CELL_NUMBER_HIGHEST_VOLTAGE;
        break;
      case PID_POLL_CELL_NUMBER_HIGHEST_VOLTAGE:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_NUMBER_HIGHEST_VOLTAGE >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_NUMBER_HIGHEST_VOLTAGE;
        poll_pid = PID_POLL_CELL_NUMBER_LOWEST_VOLTAGE;
        break;
      case PID_POLL_CELL_NUMBER_LOWEST_VOLTAGE:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_NUMBER_LOWEST_VOLTAGE >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_NUMBER_LOWEST_VOLTAGE;
        poll_pid = PID_POLL_12V_BATTERY;
        break;
      case PID_POLL_12V_BATTERY:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_12V_BATTERY >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_12V_BATTERY;
        poll_pid = PID_POLL_SOH_AVAILABLE_POWER_CALCULATION;
        break;
      case PID_POLL_SOH_AVAILABLE_POWER_CALCULATION:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_SOH_AVAILABLE_POWER_CALCULATION >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_SOH_AVAILABLE_POWER_CALCULATION;
        poll_pid = PID_POLL_SOH_GENERATED_POWER_CALCULATION;
        break;
      case PID_POLL_SOH_GENERATED_POWER_CALCULATION:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_SOH_GENERATED_POWER_CALCULATION >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_SOH_GENERATED_POWER_CALCULATION;
        poll_pid = PID_POLL_CUMULATIVE_ENERGY_WHEN_CHARGING;
        break;
      case PID_POLL_CUMULATIVE_ENERGY_WHEN_CHARGING:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CUMULATIVE_ENERGY_WHEN_CHARGING >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CUMULATIVE_ENERGY_WHEN_CHARGING;
        poll_pid = PID_POLL_CUMULATIVE_ENERGY_WHEN_DISCHARGING;
        break;
      case PID_POLL_CUMULATIVE_ENERGY_WHEN_DISCHARGING:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CUMULATIVE_ENERGY_WHEN_DISCHARGING >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CUMULATIVE_ENERGY_WHEN_DISCHARGING;
        poll_pid = PID_POLL_CUMULATIVE_ENERGY_IN_REGEN;
        break;
      case PID_POLL_CUMULATIVE_ENERGY_IN_REGEN:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CUMULATIVE_ENERGY_IN_REGEN >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CUMULATIVE_ENERGY_IN_REGEN;
        poll_pid = PID_POLL_SOCZ;
        break;
      case PID_POLL_SOCZ:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_SOCZ >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_SOCZ;
        poll_pid = PID_POLL_USOC;
        break;
      case PID_POLL_USOC:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_USOC >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_USOC;
        poll_pid = PID_POLL_CURRENT_OFFSET;
        break;
      case PID_POLL_CURRENT_OFFSET:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CURRENT_OFFSET >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CURRENT_OFFSET;
        poll_pid = PID_POLL_INSTANT_CURRENT;
        break;
      case PID_POLL_INSTANT_CURRENT:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_INSTANT_CURRENT >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_INSTANT_CURRENT;
        poll_pid = PID_POLL_MAX_REGEN;
        break;
      case PID_POLL_MAX_REGEN:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_MAX_REGEN >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_MAX_REGEN;
        poll_pid = PID_POLL_MAX_DISCHARGE_POWER;
        break;
      case PID_POLL_MAX_DISCHARGE_POWER:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_MAX_DISCHARGE_POWER >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_MAX_DISCHARGE_POWER;
        poll_pid = PID_POLL_MAX_CHARGE_POWER;
        break;
      case PID_POLL_MAX_CHARGE_POWER:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_MAX_CHARGE_POWER >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_MAX_CHARGE_POWER;
        poll_pid = PID_POLL_AVERAGE_TEMPERATURE;
        break;
      case PID_POLL_AVERAGE_TEMPERATURE:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_AVERAGE_TEMPERATURE >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_AVERAGE_TEMPERATURE;
        poll_pid = PID_POLL_MIN_TEMPERATURE;
        break;
      case PID_POLL_MIN_TEMPERATURE:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_MIN_TEMPERATURE >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_MIN_TEMPERATURE;
        poll_pid = PID_POLL_MAX_TEMPERATURE;
        break;
      case PID_POLL_MAX_TEMPERATURE:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_MAX_TEMPERATURE >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_MAX_TEMPERATURE;
        poll_pid = PID_POLL_END_OF_CHARGE_FLAG;
        break;
      case PID_POLL_END_OF_CHARGE_FLAG:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_END_OF_CHARGE_FLAG >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_END_OF_CHARGE_FLAG;
        poll_pid = PID_POLL_INTERLOCK_FLAG;
        break;
      case PID_POLL_INTERLOCK_FLAG:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_INTERLOCK_FLAG >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_INTERLOCK_FLAG;
        poll_pid = PID_POLL_CELL_1;
        break;
      case PID_POLL_CELL_1:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_1 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_1;
        poll_pid = PID_POLL_CELL_2;
        break;
      case PID_POLL_CELL_2:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_2 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_2;
        poll_pid = PID_POLL_CELL_3;
        break;
      case PID_POLL_CELL_3:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_3 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_3;
        poll_pid = PID_POLL_CELL_4;
        break;
      case PID_POLL_CELL_4:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_4 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_4;
        poll_pid = PID_POLL_CELL_5;
        break;
      case PID_POLL_CELL_5:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_5 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_5;
        poll_pid = PID_POLL_CELL_6;
        break;
      case PID_POLL_CELL_6:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_6 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_6;
        poll_pid = PID_POLL_CELL_7;
        break;
      case PID_POLL_CELL_7:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_7 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_7;
        poll_pid = PID_POLL_CELL_8;
        break;
      case PID_POLL_CELL_8:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_8 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_8;
        poll_pid = PID_POLL_CELL_9;
        break;
      case PID_POLL_CELL_9:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_9 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_9;
        poll_pid = PID_POLL_CELL_10;
        break;
      case PID_POLL_CELL_10:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_10 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_10;
        poll_pid = PID_POLL_CELL_11;
        break;
      case PID_POLL_CELL_11:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_11 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_11;
        poll_pid = PID_POLL_CELL_12;
        break;
      case PID_POLL_CELL_12:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_12 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_12;
        poll_pid = PID_POLL_CELL_13;
        break;
      case PID_POLL_CELL_13:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_13 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_13;
        poll_pid = PID_POLL_CELL_14;
        break;
      case PID_POLL_CELL_14:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_14 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_14;
        poll_pid = PID_POLL_CELL_15;
        break;
      case PID_POLL_CELL_15:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_15 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_15;
        poll_pid = PID_POLL_CELL_16;
        break;
      case PID_POLL_CELL_16:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_16 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_16;
        poll_pid = PID_POLL_CELL_17;
        break;
      case PID_POLL_CELL_17:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_17 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_17;
        poll_pid = PID_POLL_CELL_18;
        break;
      case PID_POLL_CELL_18:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_18 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_18;
        poll_pid = PID_POLL_CELL_19;
        break;
      case PID_POLL_CELL_19:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_19 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_19;
        poll_pid = PID_POLL_CELL_20;
        break;
      case PID_POLL_CELL_20:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_20 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_20;
        poll_pid = PID_POLL_CELL_21;
        break;
      case PID_POLL_CELL_21:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_21 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_21;
        poll_pid = PID_POLL_CELL_22;
        break;
      case PID_POLL_CELL_22:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_22 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_22;
        poll_pid = PID_POLL_CELL_23;
        break;
      case PID_POLL_CELL_23:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_23 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_23;
        poll_pid = PID_POLL_CELL_24;
        break;
      case PID_POLL_CELL_24:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_24 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_24;
        poll_pid = PID_POLL_CELL_25;
        break;
      case PID_POLL_CELL_25:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_25 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_25;
        poll_pid = PID_POLL_CELL_26;
        break;
      case PID_POLL_CELL_26:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_26 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_26;
        poll_pid = PID_POLL_CELL_27;
        break;
      case PID_POLL_CELL_27:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_27 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_27;
        poll_pid = PID_POLL_CELL_28;
        break;
      case PID_POLL_CELL_28:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_28 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_28;
        poll_pid = PID_POLL_CELL_29;
        break;
      case PID_POLL_CELL_29:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_29 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_29;
        poll_pid = PID_POLL_CELL_30;
        break;
      case PID_POLL_CELL_30:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_30 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_30;
        poll_pid = PID_POLL_CELL_31;
        break;
      case PID_POLL_CELL_31:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_31 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_31;
        poll_pid = PID_POLL_CELL_32;
        break;
      case PID_POLL_CELL_32:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_32 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_32;
        poll_pid = PID_POLL_CELL_33;
        break;
      case PID_POLL_CELL_33:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_33 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_33;
        poll_pid = PID_POLL_CELL_34;
        break;
      case PID_POLL_CELL_34:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_34 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_34;
        poll_pid = PID_POLL_CELL_35;
        break;
      case PID_POLL_CELL_35:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_35 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_35;
        poll_pid = PID_POLL_CELL_36;
        break;
      case PID_POLL_CELL_36:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_36 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_36;
        poll_pid = PID_POLL_CELL_37;
        break;
      case PID_POLL_CELL_37:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_37 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_37;
        poll_pid = PID_POLL_CELL_38;
        break;
      case PID_POLL_CELL_38:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_38 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_38;
        poll_pid = PID_POLL_CELL_39;
        break;
      case PID_POLL_CELL_39:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_39 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_39;
        poll_pid = PID_POLL_CELL_40;
        break;
      case PID_POLL_CELL_40:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_40 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_40;
        poll_pid = PID_POLL_CELL_41;
        break;
      case PID_POLL_CELL_41:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_41 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_41;
        poll_pid = PID_POLL_CELL_42;
        break;
      case PID_POLL_CELL_42:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_42 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_42;
        poll_pid = PID_POLL_CELL_43;
        break;
      case PID_POLL_CELL_43:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_43 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_43;
        poll_pid = PID_POLL_CELL_44;
        break;
      case PID_POLL_CELL_44:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_44 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_44;
        poll_pid = PID_POLL_CELL_45;
        break;
      case PID_POLL_CELL_45:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_45 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_45;
        poll_pid = PID_POLL_CELL_46;
        break;
      case PID_POLL_CELL_46:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_46 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_46;
        poll_pid = PID_POLL_CELL_47;
        break;
      case PID_POLL_CELL_47:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_47 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_47;
        poll_pid = PID_POLL_CELL_48;
        break;
      case PID_POLL_CELL_48:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_48 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_48;
        poll_pid = PID_POLL_CELL_49;
        break;
      case PID_POLL_CELL_49:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_49 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_49;
        poll_pid = PID_POLL_CELL_50;
        break;
      case PID_POLL_CELL_50:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_50 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_50;
        poll_pid = PID_POLL_CELL_51;
        break;
      case PID_POLL_CELL_51:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_51 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_51;
        poll_pid = PID_POLL_CELL_52;
        break;
      case PID_POLL_CELL_52:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_52 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_52;
        poll_pid = PID_POLL_CELL_53;
        break;
      case PID_POLL_CELL_53:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_53 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_53;
        poll_pid = PID_POLL_CELL_54;
        break;
      case PID_POLL_CELL_54:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_54 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_54;
        poll_pid = PID_POLL_CELL_55;
        break;
      case PID_POLL_CELL_55:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_55 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_55;
        poll_pid = PID_POLL_CELL_56;
        break;
      case PID_POLL_CELL_56:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_56 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_56;
        poll_pid = PID_POLL_CELL_57;
        break;
      case PID_POLL_CELL_57:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_57 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_57;
        poll_pid = PID_POLL_CELL_58;
        break;
      case PID_POLL_CELL_58:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_58 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_58;
        poll_pid = PID_POLL_CELL_59;
        break;
      case PID_POLL_CELL_59:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_59 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_59;
        poll_pid = PID_POLL_CELL_60;
        break;
      case PID_POLL_CELL_60:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_60 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_60;
        poll_pid = PID_POLL_CELL_61;
        break;
      case PID_POLL_CELL_61:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_61 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_61;
        poll_pid = PID_POLL_CELL_62;
        break;
      case PID_POLL_CELL_62:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_62 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_62;
        poll_pid = PID_POLL_CELL_63;
        break;
      case PID_POLL_CELL_63:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_63 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_63;
        poll_pid = PID_POLL_CELL_64;
        break;
      case PID_POLL_CELL_64:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_64 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_64;
        poll_pid = PID_POLL_CELL_65;
        break;
      case PID_POLL_CELL_65:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_65 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_65;
        poll_pid = PID_POLL_CELL_66;
        break;
      case PID_POLL_CELL_66:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_66 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_66;
        poll_pid = PID_POLL_CELL_67;
        break;
      case PID_POLL_CELL_67:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_67 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_67;
        poll_pid = PID_POLL_CELL_68;
        break;
      case PID_POLL_CELL_68:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_68 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_68;
        poll_pid = PID_POLL_CELL_69;
        break;
      case PID_POLL_CELL_69:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_69 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_69;
        poll_pid = PID_POLL_CELL_70;
        break;
      case PID_POLL_CELL_70:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_70 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_70;
        poll_pid = PID_POLL_CELL_71;
        break;
      case PID_POLL_CELL_71:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_71 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_71;
        poll_pid = PID_POLL_CELL_72;
        break;
      case PID_POLL_CELL_72:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_CELL_72 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_CELL_72;
        poll_pid = PID_POLL_SLAVES_FAIL;
        break;
      case PID_POLL_SLAVES_FAIL:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_SLAVES_FAIL >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_SLAVES_FAIL;
        poll_pid = PID_POLL_BALANCING_SW_STATUS;
        break;
      case PID_POLL_BALANCING_SW_STATUS:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_BALANCING_SW_STATUS >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_BALANCING_SW_STATUS;
        poll_pid = PID_POLL_SOC_MIN_Z;
        break;
      case PID_POLL_SOC_MIN_Z:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_SOC_MIN_Z >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_SOC_MIN_Z;
        poll_pid = PID_POLL_SOC_MAX_Z;
        break;
      case PID_POLL_SOC_MAX_Z:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_SOC_MAX_Z >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_SOC_MAX_Z;
        poll_pid = PID_POLL_PACK_LIFE_TIME_MIN;
        break;
      case PID_POLL_PACK_LIFE_TIME_MIN:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_PACK_LIFE_TIME_MIN >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_PACK_LIFE_TIME_MIN;
        poll_pid = PID_POLL_DC_RES_CHARGE;
        break;
      case PID_POLL_DC_RES_CHARGE:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_DC_RES_CHARGE >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_DC_RES_CHARGE;
        poll_pid = PID_POLL_DC_RES_DISCHARGE;
        break;
      case PID_POLL_DC_RES_DISCHARGE:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_DC_RES_DISCHARGE >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_DC_RES_DISCHARGE;
        poll_pid = PID_POLL_AVAILABLE_ENERGY;
        break;
      case PID_POLL_AVAILABLE_ENERGY:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_AVAILABLE_ENERGY >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_AVAILABLE_ENERGY;
        poll_pid = PID_POLL_FAN_CONTROL_REQ;
        break;
      case PID_POLL_FAN_CONTROL_REQ:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_FAN_CONTROL_REQ >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_FAN_CONTROL_REQ;
        poll_pid = PID_POLL_MAIN_RELAY;
        break;
      case PID_POLL_MAIN_RELAY:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_MAIN_RELAY >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_MAIN_RELAY;
        poll_pid = PID_POLL_PRECHARGE_RELAY;
        break;
      case PID_POLL_PRECHARGE_RELAY:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_PRECHARGE_RELAY >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_PRECHARGE_RELAY;
        poll_pid = PID_POLL_BUSBAR_FAIL;
        break;
      case PID_POLL_BUSBAR_FAIL:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_BUSBAR_FAIL >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_BUSBAR_FAIL;
        poll_pid = PID_POLL_BATTERY_MILEAGE_KM;
        break;
      case PID_POLL_BATTERY_MILEAGE_KM:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_BATTERY_MILEAGE_KM >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_BATTERY_MILEAGE_KM;
        poll_pid = PID_POLL_INTERLOCK_1;
        break;
      case PID_POLL_INTERLOCK_1:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_INTERLOCK_1 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_INTERLOCK_1;
        poll_pid = PID_POLL_INTERLOCK_2;
        break;
      case PID_POLL_INTERLOCK_2:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_INTERLOCK_2 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_INTERLOCK_2;
        poll_pid = PID_POLL_INTERLOCK_3;
        break;
      case PID_POLL_INTERLOCK_3:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_INTERLOCK_3 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_INTERLOCK_3;
        poll_pid = PID_POLL_INTERLOCK_4;
        break;
      case PID_POLL_INTERLOCK_4:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_INTERLOCK_4 >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_INTERLOCK_4;
        poll_pid = PID_POLL_DC_CHARGE_SUPPORTED;
        break;
      case PID_POLL_DC_CHARGE_SUPPORTED:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_DC_CHARGE_SUPPORTED >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_DC_CHARGE_SUPPORTED;
        poll_pid = PID_POLL_COMPLETE_CHARGE_COUNTER;
        break;
      case PID_POLL_COMPLETE_CHARGE_COUNTER:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_COMPLETE_CHARGE_COUNTER >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_COMPLETE_CHARGE_COUNTER;
        poll_pid = PID_POLL_INCOMPLETE_CHARGE_COUNTER;
        break;
      case PID_POLL_INCOMPLETE_CHARGE_COUNTER:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_INCOMPLETE_CHARGE_COUNTER >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_INCOMPLETE_CHARGE_COUNTER;
        poll_pid = PID_POLL_BALANCE_CHARGE_TOTAL;
        break;
      case PID_POLL_BALANCE_CHARGE_TOTAL:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_BALANCE_CHARGE_TOTAL >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_BALANCE_CHARGE_TOTAL;
        poll_pid = PID_POLL_BALANCE_TIME_TOTAL;
        break;
      case PID_POLL_BALANCE_TIME_TOTAL:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_BALANCE_TIME_TOTAL >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_BALANCE_TIME_TOTAL;
        poll_pid = PID_POLL_BALANCE_CHARGE_TOTAL_SLEEP;
        break;
      case PID_POLL_BALANCE_CHARGE_TOTAL_SLEEP:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_BALANCE_CHARGE_TOTAL_SLEEP >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_BALANCE_CHARGE_TOTAL_SLEEP;
        poll_pid = PID_POLL_BALANCE_TIME_TOTAL_SLEEP;
        break;
      case PID_POLL_BALANCE_TIME_TOTAL_SLEEP:
        CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(PID_POLL_BALANCE_TIME_TOTAL_SLEEP >> 8);
        CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)PID_POLL_BALANCE_TIME_TOTAL_SLEEP;
        poll_pid = PID_POLL_SOH_AVERAGE;
        break;
      }

    // ///- Testing purposes (scan PID) -///
    // CMFA_POLLING_FRAME.data.u8[2] = (uint8_t)(poll_pid2 >> 8);
    // CMFA_POLLING_FRAME.data.u8[3] = (uint8_t)poll_pid2;
    // poll_pid2++;
    // ///

    //else {  //Normal PID polling
      transmit_can_frame(&CMFA_POLLING_FRAME);
    //}
  }

  // Send 1000ms CAN Message
  if (currentMillis - previousMillis1000ms >= INTERVAL_1_S) {
    previousMillis1000ms = currentMillis;

    if (UserRequestDTCclear) {
      transmit_can_frame(&CMFA_EXT_DIAG);
      transmit_can_frame(&CMFA_CLEAR_DTC);
      logging.println("Clearing DTC");
      UserRequestDTCclear = false;
    } else if (UserRequestDTCRead) {
      transmit_can_frame(&CMFA_READ_DTC);
      logging.println("Reading DTC");
      UserRequestDTCRead = false;
    }
  }
}

uint8_t CmfaEvBattery::get_hvil_status() const {
  return hvil_status;
}

uint8_t CmfaEvBattery::get_hvil1_raw_state() const {
  return hvil1_raw_state;
}
uint8_t CmfaEvBattery::get_hvil2_raw_state() const {
  return hvil2_raw_state;
}
uint8_t CmfaEvBattery::get_hvil3_raw_state() const {
  return hvil3_raw_state;
}
uint8_t CmfaEvBattery::get_hvil4_raw_state() const { // Not used on 2021-2023 model
  return hvil4_raw_state;
}

bool CmfaEvBattery::get_end_of_charge() const {
  return end_of_charge;
}

bool CmfaEvBattery::get_dc_charge_support() const {
  return dc_charge_support;
}

uint16_t CmfaEvBattery::get_dc_res_charge() const {
  return dc_res_charge;
}

uint16_t CmfaEvBattery::get_dc_res_discharge() const {
  return dc_res_discharge;
}

uint16_t CmfaEvBattery::get_complete_charge_counter() const {
  return complete_charge_counter;
}

uint16_t CmfaEvBattery::get_incomplete_charge_counter() const {
  return incomplete_charge_counter;
}

uint32_t CmfaEvBattery::get_busbar_fail() const {
  return busbar_fail;
}

uint32_t CmfaEvBattery::get_slaves_fail() const {
  return slaves_fail;
}

uint32_t CmfaEvBattery::get_battery_mileage_km() const {
  return battery_mileage_km;
}

uint32_t CmfaEvBattery::get_pack_lifetime_mnt() const {
  return pack_lifetime_mnt;
}

uint32_t CmfaEvBattery::get_balance_charge_total() const{
  return balance_charge_total;
}

uint32_t CmfaEvBattery::get_balance_time_total() const{
  return balance_time_total;
}

uint32_t CmfaEvBattery::get_balance_charge_total_sleep() const{
  return balance_charge_total_sleep;
}

uint32_t CmfaEvBattery::get_balance_time_total_sleep() const{
  return balance_time_total_sleep;
}

bool CmfaEvBattery::get_balance_any_cell() const{
  return balance_any_cell;
}

void CmfaEvBattery::setup(void) {  // Performs one time setup at startup
  strncpy(datalayer.system.info.battery_protocol, Name, 63);
  datalayer.system.info.battery_protocol[63] = '\0';
  datalayer.system.status.battery_allows_contactor_closing = true;
  datalayer_battery->info.number_of_cells = 72;
  datalayer_battery->info.max_design_voltage_dV = MAX_PACK_VOLTAGE_DV;
  datalayer_battery->info.min_design_voltage_dV = MIN_PACK_VOLTAGE_DV;
  datalayer_battery->info.max_cell_voltage_mV = MAX_CELL_VOLTAGE_MV;
  datalayer_battery->info.min_cell_voltage_mV = MIN_CELL_VOLTAGE_MV;
  datalayer_battery->info.max_cell_voltage_deviation_mV = MAX_CELL_DEVIATION_MV;
}
