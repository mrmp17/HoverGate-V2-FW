#ifndef CONFIG_H
#define CONFIG_H

static constexpr char client_id[] = "hover_gate_esp";

static constexpr char gate_state_topic[] = "home/hover_gate";
static constexpr char cmd_topic[] = "home/hover_gate/cmd";
static constexpr char avail_topic[] = "home/hover_gate/available";
static constexpr char error_topic[] = "home/hover_gate/error";
static constexpr char batt_volt_topic[] = "home/hover_gate/batt_volt";

static constexpr char discovery_prefix[] = "homeassistant";
static constexpr char device_name[] = "HoverGate";
static constexpr char device_id[] = "hovergate_001";
static constexpr char device_model[] = "v2";
static constexpr char device_manufacturer[] = "PLab";

static constexpr char gate_id[] = "hovergate_001_gate";
static constexpr char error_sen_id[] = "hovergate_001_error";
static constexpr char batt_volt_sen_id[] = "hovergate_001_batt_volt";
static constexpr char reset_btn_id[] = "hovergate_001_reset";

#endif
