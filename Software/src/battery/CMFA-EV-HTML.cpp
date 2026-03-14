/*
DTCs (Sunwoda)

Prefix	Meaning
1B01xx	Cell voltage monitoring
1B07xx	Temperature sensors
1B12xx	Thermal model plausibility
1B16xx	Thermal model internal logic
1B0Cxx	Current sensors
1B0Dxx	Contactors / precharge
1B04xx	HV isolation
1B05xx	Charging sensors
1B00xx	EEPROM / configuration
1B03xx	Internal logic
1B1Exx	Thermal model consistency
1B21xx	Safety manager faults
*/

#include <Arduino.h>
#include "../datalayer/datalayer.h"
#include "../datalayer/datalayer_extended.h"
#include "../devboard/webserver/BatteryHtmlRenderer.h"
#include "CMFA-EV-HTML.h"
#include "CMFA-EV-BATTERY.h"

  String CmfaEvHtmlRenderer::getDTCDescription(uint32_t code) {
    switch (code) {
#ifndef SMALL_FLASH_DEVICE
    // Voltage / Cell Monitoring
    case 0x1B0112: return "BMS: Internal logic fault (voltage monitoring state machine)";
    case 0x1B0113: return "BMS: Internal logic fault (voltage monitoring)";
    case 0x1B0116: return "BMS: Configuration parameter invalid (voltage calibration)";
    case 0x1B0117: return "BMS: EEPROM checksum mismatch (voltage block)";
    case 0x1B0121: return "BMS: Pack voltage plausibility error";
    case 0x1B0150: return "BMS: Cell voltage sensor performance";
    case 0x1B0162: return "BMS: Cell voltage plausibility (ADC reference / sense line)";
    case 0x1B01F1: return "BMS: Cell voltage calibration not completed";
    case 0x1B01F2: return "BMS: Cell voltage measurement performance";

    // Temperature Sensors (physical sensors)
    case 0x1B0711: return "BMS: Temperature sensor #1 plausibility (physical sensor)";
    case 0x1B0712: return "BMS: Temperature sensor #2 plausibility (physical sensor)";
    case 0x1B0713: return "BMS: Temperature sensor #3 plausibility (physical sensor)";
    case 0x1B0714: return "BMS: Temperature sensor #4 plausibility (physical sensor)";
    case 0x1B0715: return "BMS: Temperature sensor #5 plausibility (physical sensor)";
    case 0x1B0716: return "BMS: Temperature sensor #6 plausibility (physical sensor)";
    case 0x1B0717: return "BMS: Temperature sensor #7 plausibility (physical sensor)";
    case 0x1B0718: return "BMS: Temperature sensor #8 plausibility (physical sensor)";
    case 0x1B0719: return "BMS: Temperature sensor #9 plausibility (physical sensor)";
    case 0x1B071A: return "BMS: Temperature sensor #10 plausibility (physical sensor)";
    case 0x1B071B: return "BMS: Temperature sensor #11 plausibility (physical sensor)";
    case 0x1B071C: return "BMS: Temperature sensor #12 plausibility (physical sensor)";
    case 0x1B071D: return "BMS: Temperature sensor #13 plausibility (physical sensor)";
    case 0x1B071E: return "BMS: Temperature sensor #14 plausibility (physical sensor)";

    // Temperature Sensors (physical sensors)
    case 0x1B0791: return "BMS: Temperature sensor #1 correlation/model plausibility (enhanced diagnostic)";
    case 0x1B0792: return "BMS: Temperature sensor #2 correlation/model plausibility (enhanced diagnostic)";
    case 0x1B0793: return "BMS: Temperature sensor #3 correlation/model plausibility (enhanced diagnostic)";
    case 0x1B0794: return "BMS: Temperature sensor #4 correlation/model plausibility (enhanced diagnostic)";
    case 0x1B0795: return "BMS: Temperature sensor #5 correlation/model plausibility (enhanced diagnostic)";
    case 0x1B0796: return "BMS: Temperature sensor #6 correlation/model plausibility (enhanced diagnostic)";
    case 0x1B0797: return "BMS: Temperature sensor #7 correlation/model plausibility (enhanced diagnostic)";
    case 0x1B0798: return "BMS: Temperature sensor #8 correlation/model plausibility (enhanced diagnostic)";
    case 0x1B0799: return "BMS: Temperature sensor #9 correlation/model plausibility (enhanced diagnostic)";
    case 0x1B079A: return "BMS: Temperature sensor #10 correlation/model plausibility (enhanced diagnostic)";
    case 0x1B079B: return "BMS: Temperature sensor #11 correlation/model plausibility (enhanced diagnostic)";
    case 0x1B079C: return "BMS: Temperature sensor #12 correlation/model plausibility (enhanced diagnostic)";
    case 0x1B079D: return "BMS: Temperature sensor #13 correlation/model plausibility (enhanced diagnostic)";
    case 0x1B079E: return "BMS: Temperature sensor #14 correlation/model plausibility (enhanced diagnostic)";

    // Thermal Model / Gradient / Delta-T
    case 0x1B1204: return "BMS: Temperature delta between sensors too large";
    case 0x1B1208: return "BMS: Temperature gradient too high";
    case 0x1B1212: return "BMS: Temperature drop rate abnormal";
    case 0x1B1213: return "BMS: Temperature rise rate abnormal";
    case 0x1B1224: return "BMS: Thermal model internal fault";
    case 0x1B1238: return "BMS: Temperature correlation error";
    case 0x1B1249: return "BMS: Thermal model mismatch";
    case 0x1B1296: return "BMS: Thermal model calibration not completed";

    // Contactor / Precharge / Current
    case 0x1B0C11: return "BMS: Current sensor plausibility";
    case 0x1B0C15: return "BMS: Current sensor range/performance";
    case 0x1B0C62: return "BMS: Current sensor offset not learned";
    case 0x1B0C98: return "BMS: Contactor feedback mismatch";
    case 0x1B0CF4: return "BMS: Precharge resistor performance";
    case 0x1B0D41: return "BMS: Main positive contactor stuck open";
    case 0x1B0D44: return "BMS: Main negative contactor stuck open";
    case 0x1B0D47: return "BMS: Precharge timeout";

    // Charging / HV / Isolation
    case 0x1B0411: return "BMS: HV isolation sensor plausibility";
    case 0x1B0415: return "BMS: HV isolation resistance too low";
    case 0x1B0462: return "BMS: HV isolation measurement not completed";
    case 0x1B0511: return "BMS: Charging voltage sensor plausibility";
    case 0x1B0515: return "BMS: Charging temperature sensor plausibility";
    case 0x1B0562: return "BMS: Charging current sensor plausibility";

    // Internal Logic / EEPROM / Configuration
    case 0x1B0016: return "BMS: Configuration invalid";
    case 0x1B0017: return "BMS: EEPROM checksum error";
    case 0x1B0301: return "BMS: Internal BMS logic fault";
    case 0x1B1F87: return "BMS: Internal BMS error (general fault)";
    case 0x1BEA0D: return "BMS: Internal logic fault (extended block)";
    case 0x1B16F4: return "BMS: Internal thermal model logic fault";
    case 0x1B1698: return "BMS:	Thermal Model Drift / Internal Model Mismatch";
    case 0x1B2143: return "BMS: Internal logic fault (safety manager)";
    default:
        return "Unknown DTC";
  #else
      default:
        return "Current BE hardware does not support details. Please upgrade to large flash BE";  // Detailed DTC descriptions not available on 4MB low flash devices. The above text takes up a massive amount of flash!
  #endif
    }
  }

  String CmfaEvHtmlRenderer::get_status_html() {
    String content;

    content += "<h4>SOC U: " + String((float)datalayer_extended.CMFAEV.soc_u/100) + " %</h4>";
    content += "<h4>SOC Z: " + String((float)datalayer_extended.CMFAEV.soc_z/100) + " %</h4>";
    content += "<h4>SOH Average: " + String((float)datalayer_extended.CMFAEV.soh_average/100) + "%</h4>";
    content += "<h4>12V voltage: " + String((float)datalayer_extended.CMFAEV.lead_acid_voltage/1000) + "V</h4>";
    content += "<h4>Highest cell number: " + String(datalayer_extended.CMFAEV.highest_cell_voltage_number) + "</h4>";
    content += "<h4>Lowest cell number: " + String(datalayer_extended.CMFAEV.lowest_cell_voltage_number) + "</h4>";
    content += "<h4>Sum of cell voltages: " + String((float)datalayer_extended.CMFAEV.cells_voltage_sum/10) + " V</h4>";
    content += "<h4>Max regen power: " + String((float)datalayer_extended.CMFAEV.max_regen_power/1000) + " kW</h4>";
    content += "<h4>Max discharge power: " + String((float)datalayer_extended.CMFAEV.max_discharge_power/1000) + " kW</h4>";
    content += "<h4>Max charge power: " + String((float)datalayer_extended.CMFAEV.maximum_charge_power/1000) + " kW</h4>";
    content += "<h4>SOH available power: " + String((float)datalayer_extended.CMFAEV.SOH_available_power/10) + " %</h4>";
    content += "<h4>SOH generated power: " + String((float)datalayer_extended.CMFAEV.SOH_generated_power/10) + " %</h4>";
    content += "<h4>Average temperature: " + String((float)datalayer_extended.CMFAEV.average_temperature/10) + " C</h4>";
    content += "<h4>Maximum temperature: " + String((float)datalayer_extended.CMFAEV.maximum_temperature/10) + " C</h4>";
    content += "<h4>Minimum temperature: " + String((float)datalayer_extended.CMFAEV.minimum_temperature/10) + " C</h4>";
    content += "<h4>Cumulative energy discharged: " + String((float)datalayer_extended.CMFAEV.cumulative_energy_when_discharging/1000) + "kWh</h4>";
    content += "<h4>Cumulative energy charged: " + String((float)datalayer_extended.CMFAEV.cumulative_energy_when_charging/1000) + "kWh</h4>";
    content += "<h4>Cumulative energy regen: " + String((float)datalayer_extended.CMFAEV.cumulative_energy_in_regen/1000) + "kWh</h4>";

    // Battery Balancing Section
    content += 
        "<h3 style='color: #86b82a; border-bottom: 2px solid  #86b82a; padding-bottom: 5px;'>⚖️ Battery Balancing</h3>";
    content += "<div style='margin-left: 15px;'>";
    content += "<h4>Balancing Active(any cell): " + String(batt.get_balance_any_cell()?"Yes":"No") + "</h4>";
    content += "<h4>Total balancing charge: " + String(batt.get_balance_charge_total()) + " Ah</h4>";
    content += "<h4>Total balancing time: " + String(batt.get_balance_time_total()) + " h</h4>";
    content += "<h4>Total balancing charge during sleep: " + String(batt.get_balance_charge_total_sleep()) + " Ah</h4>";
    content += "<h4>Total balancing time during sleep: " + String(batt.get_balance_time_total_sleep()) + " h</h4>";

    content += "</div>";

    // Battery Health Section
    content +=
        "<h3 style='color: #aa08ca; border-bottom: 2px solid  #aa08ca; padding-bottom: 5px;'>❤️ Battery Health</h3>";
    content += "<div style='margin-left: 15px;'>";
    content += "<h4>DC Charge Support: " + String(batt.get_dc_charge_support()?"Yes":"No") + "</h4>";
    content += "<h4>Internal Resistance while charging: " + String((float)batt.get_dc_res_charge()/1000) + " mΩ</h4>";
    content += "<h4>Internal Resistance while discharging: " + String((float)batt.get_dc_res_discharge()/1000) + " mΩ</h4>";
    content += "<h4>Complete Charge Counter: " + String(batt.get_complete_charge_counter()) + " </h4>";
    content += "<h4>Partial Charge Counter: " + String(batt.get_incomplete_charge_counter()) + " </h4>";
    content += "<h4>Battery Mileage: " + String(batt.get_battery_mileage_km()) + " km</h4>";
    content += "<h4>Battery Pack Age: ";
    
    uint32_t minutes = batt.get_pack_lifetime_mnt();
    uint32_t years  = minutes / 525949;
    minutes    %= 525949;
    uint32_t months = minutes / 43829;
    minutes    %= 43829;
    uint32_t days   = minutes / 1440;

    content += String(years) + " years " + String(months) + " months " + String(days) + " days</h4>";
    content += "<h4>Busbars Status: <span style=";
    content += (batt.get_busbar_fail() > 0)?("'color: #d32f2f;'>⚠ Failed (" + String(batt.get_busbar_fail(),2)) + ")":"'color: #33db12;'> OK";
    content += "</span></h4>";
    content += "<h4>Slaves Status: <span style=";
    content += (batt.get_slaves_fail() > 0)?("'color: #d32f2f;'>⚠ Failed (" + String(batt.get_slaves_fail(),2)) + ")":"'color: #33db12;'> OK";
    content += "</span></h4>";
    content += "</div>";

    // Safety Systems Section
    content += "<h3 style='color: #e53935; border-bottom: 2px solid #e53935; padding-bottom: 5px;'>🛡️ Safety Systems</h3>";
    content += "<div style='margin-left: 15px;'>";
    content += "<h4>HVIL Status: ";
    // 0:Not used;1:Opened;2:Closed;3:Unavailable
    switch (batt.get_hvil_status()) {
      case 0:
        content += "Not used</h4>";
        break;
      case 1:
        content += "<span style='color: #d32f2f;'>⚠ Error (Loop Open)</span></h4>";
        break;
      case 2:
        content += "OK (Loop Closed)</h4>";
        break;
      case 3:
        content += "Unknown</h4>";
        break;
      default:
        content += "Unknown</h4>";
    }
    content += "<h4>HVIL1 Raw State: ";
    //// 3:SC+_ret;2:SC+;1:closed;0:open;4:SC_gnd:
    switch (batt.get_hvil1_raw_state()) {
      case 0:
        content += "Open</h4>";
        break;
      case 1:
        content += "Closed</h4>";
        break;
      case 2:
        content += "<span style='color: #d32f2f;'>⚠ Short Circuit to +</span></h4>";
        break;
      case 3:
        content += "<span style='color: #d32f2f;'>⚠ Short Circuit to + Ret</span></h4>";
        break;
      case 4:
        content += "<span style='color: #d32f2f;'>⚠ Short Circuit to Gnd</span></h4>";
        break;
      default:
        content += "Unknown</h4>";
    }
    content += "<h4>HVIL2 Raw State: ";
    //// 3:SC+_ret;2:SC+;1:closed;0:open;4:SC_gnd:
    switch (batt.get_hvil2_raw_state()) {
      case 0:
        content += "Open</h4>";
        break;
      case 1:
        content += "Closed</h4>";
        break;
      case 2:
        content += "<span style='color: #d32f2f;'>⚠ Short Circuit to +</span></h4>";
        break;
      case 3:
        content += "<span style='color: #d32f2f;'>⚠ Short Circuit to + Ret</span></h4>";
        break;
      case 4:
        content += "<span style='color: #d32f2f;'>⚠ Short Circuit to Gnd</span></h4>";
        break;
      default:
        content += "Unknown</h4>";
    }
    content += "<h4>HVIL3 Raw State: ";
    //// 3:SC+_ret;2:SC+;1:closed;0:open;4:SC_gnd:
    switch (batt.get_hvil3_raw_state()) {
      case 0:
        content += "Open</h4>";
        break;
      case 1:
        content += "Closed</h4>";
        break;
      case 2:
        content += "<span style='color: #d32f2f;'>⚠ Short Circuit to +</span></h4>";
        break;
      case 3:
        content += "<span style='color: #d32f2f;'>⚠ Short Circuit to + Ret</span></h4>";
        break;
      case 4:
        content += "<span style='color: #d32f2f;'>⚠ Short Circuit to Gnd</span></h4>";
        break;
      default:
        content += "Unknown</h4>";
    }
    content += "<h4>HVIL4 Raw State: ";
    //// 3:SC+_ret;2:SC+;1:closed;0:open;4:SC_gnd:
    switch (batt.get_hvil4_raw_state()) {
      case 0:
        content += "Open(Not used?)</h4>";
        break;
      case 1:
        content += "Closed</h4>";
        break;
      case 2:
        content += "<span style='color: #d32f2f;'>⚠ Short Circuit to +</span></h4>";
        break;
      case 3:
        content += "<span style='color: #d32f2f;'>⚠ Short Circuit to + Ret</span></h4>";
        break;
      case 4:
        content += "<span style='color: #d32f2f;'>⚠ Short Circuit to Gnd</span></h4>";
        break;
      default:
        content += "Unknown</h4>";
    }

    // Diagnostic Trouble Codes Section
    content +=
        "<h3 style='color: #27b06c; border-bottom: 2px solid #27b06c; padding-bottom: 5px;'>🔧 Diagnostic Trouble "
        "Codes</h3>";
    content += "<div style='margin-left: 15px; margin-right: 15px;'>";

    ///////////
    // datalayer_extended.CMFAEV.dtc_last_read_millis = 1;
    // datalayer_extended.CMFAEV.dtc_count = 1;
    // datalayer_extended.CMFAEV.dtc_codes[0] = 0x1B0162;
    // datalayer_extended.CMFAEV.dtc_status[0] = 0x50;
    ///////////

    if (datalayer_extended.CMFAEV.dtc_last_read_millis == 0) {
      // No DTC read has been performed yet
      content +=
          "<p style='color: #ff9800;'>ℹ DTCs have not been read yet. Click 'Read DTC' to scan for fault codes.</p>";
    } else if (datalayer_extended.CMFAEV.dtc_read_failed) {
      content += "<p style='color: #d32f2f;'>⚠ Last DTC read failed or not supported</p>";
    } else if (datalayer_extended.CMFAEV.dtc_count == 0) {
      content += "<p style='color: #4CAF50;'>✓ No DTCs present</p>";
    } else {
      content += "<p><strong>DTC Count:</strong> " + String(datalayer_extended.CMFAEV.dtc_count) + "</p>";

      // Convert last read time to days:hours:minutes:seconds format
      unsigned long last_read_seconds = (millis() - datalayer_extended.CMFAEV.dtc_last_read_millis) / 1000;
      unsigned long read_days = last_read_seconds / 86400;
      unsigned long read_hours = (last_read_seconds % 86400) / 3600;
      unsigned long read_minutes = (last_read_seconds % 3600) / 60;
      unsigned long read_seconds = last_read_seconds % 60;

      content += "<p><strong>Last Read:</strong> ";
      if (read_days > 0) {
        content += String(read_days) + "d ";
      }
      if (read_hours > 0 || read_days > 0) {
        content += String(read_hours) + "h ";
      }
      content += String(read_minutes) + "m " + String(read_seconds) + "s ago</p>";

      content += "<div style='overflow-x: auto; margin-top: 10px; margin-bottom: 15px;'>";
      content +=
          "<table style='width: auto; margin: 0 auto; border-collapse: separate; border-spacing: 0; border: 1px solid "
          "#ddd; border-radius: 8px; overflow: hidden;'>";

      content += "<thead>";
      content += "<tr style='background: linear-gradient(135deg, #667eea 0%, #764ba2 100%); color: white;'>";
      content += "<th style='padding: 12px 15px; text-align: left; font-weight: 600;'>DTC Code</th>";
      content += "<th style='padding: 12px 15px; text-align: left; font-weight: 600;'>Status</th>";
      content += "<th style='padding: 12px 15px; text-align: left; font-weight: 600;'>Description</th>";
      content += "</tr>";
      content += "</thead>";

      content += "<tbody>";

      for (int i = 0; i < datalayer_extended.CMFAEV.dtc_count; i++) {
        uint32_t code = datalayer_extended.CMFAEV.dtc_codes[i];
        uint8_t status = datalayer_extended.CMFAEV.dtc_status[i];

        char dtcStr[12];
        sprintf(dtcStr, "%06lX", code);

        String statusStr = "Stored";
        String statusColor = "#757575";

        if (status & 0x08) {
          statusStr = "Confirmed";
          statusColor = "#ff6f00";
        }

        if (status & 0x01) {
          statusStr = "Active";
          statusColor = "#d32f2f";
        }

        String description = getDTCDescription(code);
        if (description.length() == 0) {
          description = "Unknown";
        }

        content += "<tr>";
        content +=
            "<td style='padding: 12px 15px; border-top: 1px solid #e0e0e0; font-family: monospace; font-size: 1.1em; "
            "font-weight: 600;'>" +
            String(dtcStr) + "</td>";
        content += "<td style='padding: 12px 15px; border-top: 1px solid #e0e0e0; color: " + statusColor +
                  "; font-weight: 500;'>" + statusStr + "</td>";
        content += "<td style='padding: 12px 15px; border-top: 1px solid #e0e0e0; font-size: 0.95em; color: #ddd;'>" +
                  description + "</td>";
        content += "</tr>";
      }

      content += "</tbody>";
      content += "</table>";
      content += "</div>";
    }

    content += "</div>";

    return content;
  }
