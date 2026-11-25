#pragma once

#include "soc/soc_caps.h"
#include "sdkconfig.h"
#if CONFIG_ZB_ENABLED

#include <math.h>
#include "Zigbee.h"
#include "ep/ZigbeeTempSensor.h"
#include "ep/ZigbeeCarbonDioxideSensor.h"

/**
 * Single-endpoint Zigbee sensor combining Temperature (0x0402),
 * Relative Humidity (0x0405), and Carbon Dioxide (0x040D) on one endpoint
 * so Zigbee2MQTT exposes clean `temperature`, `humidity`, and `co2` properties.
 */
class ZigbeeClimateCO2Sensor : public ZigbeeTempSensor {
public:
  explicit ZigbeeClimateCO2Sensor(uint8_t endpoint)
      : ZigbeeTempSensor(endpoint), _co2_sensor(false) {}

  void addCarbonDioxideSensor(float minPpm = 0.0f, float maxPpm = 65000.0f, float tolerancePpm = 0.0f) {
    if (_co2_sensor || _cluster_list == nullptr) {
      return;
    }

    esp_zb_carbon_dioxide_measurement_cluster_cfg_t co2_cfg = {
        .measured_value = NAN,
        .min_measured_value = minPpm / 1000000.0f,
        .max_measured_value = maxPpm / 1000000.0f,
    };
    esp_zb_attribute_list_t *co2_cluster = esp_zb_carbon_dioxide_measurement_cluster_create(&co2_cfg);
    if (co2_cluster == nullptr) {
      log_e("Failed to create CO2 measurement cluster");
      return;
    }

    if (tolerancePpm > 0.0f) {
      float zb_tol = tolerancePpm / 1000000.0f;
      esp_zb_carbon_dioxide_measurement_cluster_add_attr(
          co2_cluster, ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_TOLERANCE_ID, (void *)&zb_tol);
    }

    esp_err_t ret = esp_zb_cluster_list_add_carbon_dioxide_measurement_cluster(
        _cluster_list, co2_cluster, ESP_ZB_ZCL_CLUSTER_SERVER_ROLE);
    if (ret != ESP_OK) {
      log_e("Failed to add CO2 cluster: 0x%x: %s", ret, esp_err_to_name(ret));
      return;
    }
    _co2_sensor = true;
  }

  bool setCarbonDioxide(float ppm) {
    if (!_co2_sensor) {
      return false;
    }
    float zb_co2 = ppm / 1000000.0f;
    esp_zb_zcl_status_t ret = setClusterAttribute(
        ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT,
        ESP_ZB_ZCL_CLUSTER_SERVER_ROLE,
        ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID,
        &zb_co2, false);
    if (ret != ESP_ZB_ZCL_STATUS_SUCCESS) {
      log_e("Failed to set CO2: 0x%x: %s", ret, esp_zb_zcl_status_to_name(ret));
      return false;
    }
    return true;
  }

  bool reportCarbonDioxide() {
    if (!_co2_sensor) {
      return false;
    }
    esp_zb_zcl_report_attr_cmd_t report_attr_cmd = {};
    report_attr_cmd.address_mode = ESP_ZB_APS_ADDR_MODE_DST_ADDR_ENDP_NOT_PRESENT;
    report_attr_cmd.attributeID = ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID;
    report_attr_cmd.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_CLI;
    report_attr_cmd.clusterID = ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT;
    report_attr_cmd.zcl_basic_cmd.src_endpoint = _endpoint;
    report_attr_cmd.manuf_specific = 0x00U;
    report_attr_cmd.dis_default_resp = 0x00U;
    return reportClusterAttribute(&report_attr_cmd);
  }

  bool setCarbonDioxideReporting(uint16_t min_interval, uint16_t max_interval, uint16_t delta_ppm) {
    if (!_co2_sensor) {
      return false;
    }
    esp_zb_zcl_reporting_info_t reporting_info = {};
    reporting_info.direction = ESP_ZB_ZCL_CMD_DIRECTION_TO_SRV;
    reporting_info.ep = _endpoint;
    reporting_info.cluster_id = ESP_ZB_ZCL_CLUSTER_ID_CARBON_DIOXIDE_MEASUREMENT;
    reporting_info.cluster_role = ESP_ZB_ZCL_CLUSTER_SERVER_ROLE;
    reporting_info.attr_id = ESP_ZB_ZCL_ATTR_CARBON_DIOXIDE_MEASUREMENT_MEASURED_VALUE_ID;
    reporting_info.u.send_info.min_interval = min_interval;
    reporting_info.u.send_info.max_interval = max_interval;
    reporting_info.u.send_info.def_min_interval = min_interval;
    reporting_info.u.send_info.def_max_interval = max_interval;
    reporting_info.dst.profile_id = ESP_ZB_AF_HA_PROFILE_ID;
    reporting_info.manuf_code = ESP_ZB_ZCL_ATTR_NON_MANUFACTURER_SPECIFIC;
    float delta_f = delta_ppm / 1000000.0f;
    memcpy(&reporting_info.u.send_info.delta.s32, &delta_f, sizeof(float));
    return setClusterReporting(&reporting_info);
  }

private:
  bool _co2_sensor;
};

#endif  // CONFIG_ZB_ENABLED
