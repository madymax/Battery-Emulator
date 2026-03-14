#ifndef _CMFA_EV_HTML_H
#define _CMFA_EV_HTML_H

#include "../datalayer/datalayer.h"
#include "../devboard/webserver/BatteryHtmlRenderer.h"

class CmfaEvBattery;

class CmfaEvHtmlRenderer : public BatteryHtmlRenderer {
 private:
  CmfaEvBattery& batt;

 public:
  CmfaEvHtmlRenderer(CmfaEvBattery& b) : batt(b) {}
  String getDTCDescription(uint32_t code);
  String get_status_html();
};

#endif
