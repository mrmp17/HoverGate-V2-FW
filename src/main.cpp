#include <Arduino.h>

#include <WiFi.h>
#include <esp_now.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

#include "config.h"
#include "BLDC_driver.h"
#include "latch.h"
#include "gate.h"
#include "secrets.h"
#include "comms.h"
#include "remote_gate.h"

// short gate is master (has RF, implements MQTT, commands long)
#if defined(GATE_SHORT) + defined(GATE_LONG) != 1
#error "GATE_SHORT or GATE_LONG must be defined"
#endif

static constexpr uint16_t SLAVE_COMMS_INTERVAL = 250; // ms
static constexpr uint16_t PRINT_INFO_INTERVAL = 1000; // ms
static constexpr uint32_t open_long_gate_delay = 1000L;
static constexpr uint32_t close_short_gate_delay = 8000L;
static constexpr uint32_t batt_volt_pub_interval = 10000L;
static constexpr uint32_t reset_timeout = 5000;

// PINS
static constexpr uint8_t rf_pin = 1;
static constexpr uint8_t led_pin = 46;

// RF Remote
static constexpr uint32_t rf_longpress_delay = 5000;
static constexpr uint32_t rf_min_interval = 1000;
static constexpr uint32_t rf_debounce_time = 20;
static uint32_t last_rf_triggerd_time = 0;
static bool rf_short_flag = false;
static bool rf_long_flag = false; 

static uint32_t last_print_info_time = 0;
static uint32_t last_reset = 0;

#if defined(GATE_SHORT)
gate_params params {
    .loop_dt = 10, // milliseconds between loops
    .enc_ticks_per_deg = 2.725, // encoder ticks per degree of gate angle
    .angle_open = 90.0, // angle when gate open
    .angle_closed = 0.0, // angle when gate closed
    .a_max = 6.0,
    .v_max = 16.0,
    .v_min = 6.0,
    .driver_open_dir = -1, // driver pwm sign for open direction. 1 or -1.
    .max_pwm = 150, // max driver pwm
    .pid_kp = 30,
    .pid_ki = 2,
    .pid_slow_kp = 10,
    .pid_slow_ki = 0,
    .pid_imax = 300,
    .vel_update_tick_num = 5,
    .zero_vel_timeout = 2000,
    .move_uncert_before = 10.0, // degrees before target when velocity is reduced
    .move_uncert_after = 30.0, // degrees after target when velocity still set
    .max_angle_follow_error = 10.0, // max error when gate stopped is detected
    .hold_open_offset = 5.
};
const uint8_t *mac_addr = mac_gate_long;
CommsEspNow comms(mac_addr, wifi_ssid, wifi_password);
RemoteGate remote_gate(&comms);
GateState master_gate_state = GateState::closed;
GateState master_cmd_state = GateState::closed;
GateState master_prev_state = GateState::closed;
GateError master_error_code = GateError::gate_ok;
GateState master_pre_error_state = GateState::closed;
uint8_t master_sm_ctrl = 0;
WiFiClient wifi_client;
PubSubClient mqtt_client(wifi_client);
bool wifi_available = false;
bool mqtt_available = false;

#elif defined(GATE_LONG)
gate_params params {
    .loop_dt = 10, // milliseconds between loops
    .enc_ticks_per_deg = 6.1111, // encoder ticks per degree of gate angle
    .angle_open = -90.0, // angle when gate open
    .angle_closed = 0.0, // angle when gate closed
    .a_max = 4,
    .v_max = 12.0,
    .v_min = 4.0,
    .driver_open_dir = 1, // driver pwm sign for open direction. 1 or -1.
    .max_pwm = 150, // max driver pwm
    .pid_kp = 30,
    .pid_ki = 2,
    .pid_slow_kp = 10,
    .pid_slow_ki = 0,
    .pid_imax = 300,
    .vel_update_tick_num = 5,
    .zero_vel_timeout = 2000,
    .move_uncert_before = 10.0, // degrees before target when velocity is reduced
    .move_uncert_after = 30.0, // degrees after target when velocity still set
    .max_angle_follow_error = 10.0, // max error when gate stopped is detected
    .hold_open_offset = -6.
};
Latch latch;
const uint8_t *mac_addr = mac_gate_short;
CommsEspNow comms(mac_addr, wifi_ssid, wifi_password);
#endif

BLDC_driver BLDC;
Gate local_gate(params);



#if defined(GATE_SHORT)
void open_gate() {
  if (master_gate_state == GateState::open || master_gate_state == GateState::opening)
    return;
  master_cmd_state = GateState::open;
  Serial.println("CMD open");
}

void close_gate() {
  if (master_gate_state == GateState::closed || master_gate_state == GateState::closing)
    return;
  master_cmd_state = GateState::closed;
  Serial.println("CMD close");
}

void toggle_gate() {
  switch(master_gate_state) {
    case GateState::closing:
    case GateState::closed:
      open_gate();
      break;
    case GateState::opening:
    case GateState::open:
      close_gate();
      break;
  }
}

void stop_gate() {
  Serial.println("CMD stop");
  // TODO
}

void reset_gate() {
  master_gate_state = GateState::resetting;
  local_gate.reset();
  remote_gate.reset();
  master_sm_ctrl = 0;
  Serial.println("CMD reset");
}

void publish_state(GateState state) {
  if (mqtt_available) {
    mqtt_client.publish(state_topic, state_2_str(state), true);
  }
}

void publish_error(GateError master_error, GateError gate_error = GateError::gate_ok) {
  if (mqtt_available) {
    if (gate_error != GateError::gate_ok) {
      char buffer[50];
      sprintf(buffer, "Master error: %s, Gate error: %s", 
              error_2_str(master_error), error_2_str(gate_error));
      mqtt_client.publish(error_topic, buffer, true);
    } else {
      char buffer[50];
       sprintf(buffer, "Master error: %s", error_2_str(master_error));
      mqtt_client.publish(error_topic, buffer, true);
    }
  }
}

void error_handler(GateError master_error, bool emergency_stop) {
  if (master_gate_state == GateState::error) {
    return;
  }
  master_pre_error_state = master_gate_state;
  master_gate_state = GateState::error;
  if (emergency_stop) {
    local_gate.stop();
    remote_gate.stop();
  }
  publish_error(master_error);
  Serial.printf("Master error: %s\n\r", error_2_str(master_error));
}

void error_handler(GateError master_error, GateError gate_error, bool emergency_stop) {
  if (master_gate_state == GateState::error) {
    return;
  }
  master_pre_error_state = master_gate_state;
  master_gate_state = GateState::error;
  if (emergency_stop) {
    local_gate.stop();
    remote_gate.stop();
  }
  publish_error(master_error, gate_error);
  Serial.printf("Master error: %s, gate error: %s\n\r", error_2_str(master_error), error_2_str(gate_error));
}

void clear_error() {
  master_gate_state = master_pre_error_state;
  master_error_code = GateError::gate_ok;
  publish_error(master_error_code);
}

void master_gate_loop() {
  remote_gate.loop();

  // RF
  if(rf_short_flag) {
    rf_short_flag = false;
    if (millis() - last_rf_triggerd_time > rf_min_interval) {
      last_rf_triggerd_time = millis();
      Serial.println("RF triggered");
      toggle_gate();
    }
  }
  if(rf_long_flag) {
    rf_long_flag = false;
    if (millis() - last_reset > reset_timeout) {
      Serial.println("RF longpress");
      reset_gate();
      last_reset = millis();
    }
  }

  // Master state machine
  static uint32_t open_time = 0;
  static uint32_t close_time = 0;
  switch (master_gate_state) {
    default:
      // Serial comm check
      if(false) {
        error_handler(GateError::short_gate_comm_dead, false);
      }
      if(!remote_gate.is_connected()) {
        error_handler(GateError::long_gate_comm_dead, false);
      }

      // gate error check
      if(local_gate.get_error_code() != GateError::gate_ok) {
        error_handler(GateError::short_gate_error, local_gate.get_error_code(), true);
      }
      if(remote_gate.get_error_code() != GateError::gate_ok) {
        error_handler(GateError::long_gate_error, remote_gate.get_error_code(), true);
      }

      switch(master_sm_ctrl) {
        case 0: // gate closed
          if(master_cmd_state == GateState::open) {
            master_gate_state = GateState::opening;
            local_gate.open();
            open_time = millis();
            master_sm_ctrl = 1;
          }
          break;
        case 1: // short gate opening, long waiting
          if(millis() - open_time > open_long_gate_delay) {
            remote_gate.open();
            master_sm_ctrl = 2;
          }
          if (master_cmd_state == GateState::closed) {
            master_gate_state = GateState::closing;
            local_gate.close();
            close_time = millis();
            master_sm_ctrl = 5;
          }
          break;
        case 2: // both gates opening
          if (local_gate.get_state() == GateState::open
              && remote_gate.get_state() == GateState::open
            ) {
            master_gate_state = GateState::open;
            master_sm_ctrl = 3;
          }
          if (master_cmd_state == GateState::closed) {
            master_gate_state = GateState::closing;
            remote_gate.close();
            close_time = millis();
            master_sm_ctrl = 4;
          }
          break;
        case 3: // gate open
          if (master_cmd_state == GateState::closed) {
            master_gate_state = GateState::closing;
            remote_gate.close();
            close_time = millis();
            master_sm_ctrl = 4;
          }
          break;
        case 4: // long gate closing, short waiting
          if (millis() - close_time > close_short_gate_delay) {
            local_gate.close();
            master_sm_ctrl = 5;
          }
          if (master_cmd_state == GateState::open) {
            master_gate_state = GateState::opening;
            remote_gate.open();
            open_time = millis();
            master_sm_ctrl = 2;
          }
          break;
        case 5: // both gates closing
          if (local_gate.get_state() == GateState::closed
              && remote_gate.get_state() == GateState::closed
            ) {
            master_gate_state = GateState::closed;
            master_sm_ctrl = 0;
          }
          if (master_cmd_state == GateState::open) {
            master_gate_state = GateState::opening;
            local_gate.open();
            open_time = millis();
            master_sm_ctrl = 1;
          }
          break;
      break;
      }
      break;
    case GateState::error:
      // Clear communication errors
      if (master_error_code == GateError::short_gate_comm_dead) {
        if (true) {
          Serial.println("Short gate reconnected.");
          master_error_code = GateError::gate_ok;
          clear_error();
        }
      }
      if (master_error_code == GateError::long_gate_comm_dead) {
        if (remote_gate.is_connected()) {
          Serial.println("Long gate reconnected.");
          master_error_code = GateError::gate_ok;
          clear_error();
        }
      }
      break;
    case GateState::resetting:
      if (local_gate.get_error_code() == GateError::gate_ok &&
          remote_gate.get_error_code() == GateError::gate_ok) {
        master_gate_state = GateState::closed;
        master_cmd_state = master_gate_state;
        master_error_code = GateError::gate_ok;
        Serial.println("Reset successful.");
        mqtt_client.publish(error_topic, "NO_ERROR", "true");
      }
      break;
  }

  // state changed
  if (master_gate_state != master_prev_state) {
    publish_state(master_gate_state);
    master_prev_state = master_gate_state;
  }
}

#elif defined(GATE_LONG)
void slave_gate_loop() {
  // do incoming commands, periodic state report back to master
  static unsigned long last_sent_time = millis();
  if(millis() - last_sent_time > SLAVE_COMMS_INTERVAL){
    uint32_t t = millis();
    digitalWrite(led_pin, HIGH);
    delay(10);
    digitalWrite(led_pin, LOW);
    t_msg_esp_now msg;
    msg.gate_state = static_cast<uint8_t>(local_gate.get_state());
    msg.error_code = static_cast<uint8_t>(local_gate.get_error_code());
    msg.gate_angle = local_gate.get_angle();
    msg.bat_volt = 0; //todo: implement
    msg.action_cmd = 0; // not in use. set to 0
    msg.isShort = false;
    comms.send_msg(msg);
    //todo: need to do sth if this fails?
    // Serial.println("took " + String(millis()-t) + "ms to send msg.");
    //todo: what if delivery is not ok?
    last_sent_time = millis();
  }

  // do incoming commands
  if(comms.is_recv_available()){
    t_msg_esp_now msg = comms.get_recv_msg();
    switch(msg.action_cmd) {
    case 0:
      local_gate.open();
      break;
    case 1:
      local_gate.close();
      break;
    case 2:
      local_gate.reset();
      break;
    case 3:
      local_gate.stop();
      break;
    case 4:
      local_gate.toggle();
      break;
    //for testing
    case 5:
      Serial.println("Received action_cmd 5");
      break;
    default:
      // Handle invalid action_cmd values
      break;
    }
  }
}
#endif

#if defined(GATE_SHORT)
void send_mqtt_discovery_conf() {
  JsonDocument doc;
  doc["icon"] = "mdi:gate";
  doc["command_topic"] = cmd_topic;
  doc["state_topic"] = state_topic;
  doc["availability_topic"] = avail_topic;
  doc["unique_id"] = "hovergate_001";
  doc["device"]["name"] = "HoverGate";
  doc["device"]["identifiers"] = "hovergate_001";
  doc["device"]["model"] = "v2";
  doc["device"]["manufacturer"] = "PLab";
  char buffer[512];
  serializeJson(doc, buffer);
  mqtt_client.publish("homeassistant/cover/hovergate/config", buffer, true);
}

void mqtt_callback(char* topic, byte* payload, unsigned int length) {
  if (strcmp(topic, cmd_topic) == 0) {
    char msg[length + 1];
    memcpy(msg, payload, (size_t)length);
    msg[length] = '\0';  // required so it can be converted to String
    Serial.printf("Received command: %s\n\r", msg);
    if (strcmp(msg, "OPEN") == 0) {
      open_gate();
    } else if (strcmp(msg, "CLOSE") == 0) {
      close_gate();
    } else if (strcmp(msg, "STOP") == 0) {
      stop_gate();
    } else if (strcmp(msg, "RESET") == 0) {
      reset_gate();
    }
  }
}

void mqtt_handler() {
  static uint8_t ctrl = 0;
  static uint32_t reconnect_time = 20000;
  static const uint32_t connect_timeout = 20000;
  mqtt_client.loop();
  switch (ctrl) {
    case 0: // check wifi status
      if (wifi_available && !mqtt_client.connected() && millis() - reconnect_time > connect_timeout) {
        Serial.println("Reconnecting MQTT...");
        mqtt_available = false;
        mqtt_client.connect(client_id, mqtt_user, mqtt_password);
        reconnect_time = millis();
        ctrl = 1;
      }
      break;
    case 1: // wait for connection or timeout
      if (mqtt_client.connected()) {
        mqtt_available = true;
        Serial.println("MQTT connected.");
        mqtt_client.publish(avail_topic, "online", true);
        publish_state(master_gate_state);
        publish_error(master_error_code);
        send_mqtt_discovery_conf();
        mqtt_client.subscribe(cmd_topic);
        Serial.println("Sent MQTT discovery config.");
        ctrl = 0;
      }
      else if (millis() - reconnect_time > connect_timeout) {
        Serial.print("MQTT connection failed, rc=");
        Serial.println(mqtt_client.state());
        ctrl = 0;
      }
      break;
  }
}

bool get_rf_state() {
  return digitalRead(rf_pin);
}

void rf_handler() {
  static uint8_t loopCtrl = 0;
  static uint32_t timing = 0;
  static uint32_t timeNow = 0;

  timeNow = millis();

  switch(loopCtrl){
    case 0: //waiting for button press
      if(get_rf_state()){ //is button pressed?
        timing = timeNow;
        loopCtrl = 1;
      }
      break;
    case 1: //waiting for debounce period
      if(timeNow-timing >= rf_debounce_time){
        if(get_rf_state()){ //button still pressed after debounce time
          loopCtrl = 2;
        }
        else{ //button is not pressed anymore
          loopCtrl = 0; //go back to state 0
        }
      }
      break;
    case 2: //button is pressed, waiting for short and long press
      if(!get_rf_state()){ //button released
        rf_short_flag = true;
        loopCtrl = 0; //go back to state 0
      }
      else if(timeNow-timing >= rf_longpress_delay){
        rf_long_flag = true;
        loopCtrl = 3;
      }
      break;

    case 3: //waiting for button release after long press duration
      if(!get_rf_state()){ //go to state 0 when button released
        loopCtrl = 0;
      }
      break;
  }
}

#endif

void print_info() {
  #if defined(GATE_SHORT)
    Serial.printf("remote connected: %d\n\r", remote_gate.is_connected());
    Serial.printf("gate state: %s\n\r", local_gate.get_state_str());
    Serial.printf("gate angle: %f\n\r", local_gate.get_angle());
    Serial.printf("error code: %s\n\r", local_gate.get_error_str());
    Serial.printf("remote gate state: %s\n\r", remote_gate.get_state_str());
    Serial.printf("remote error code: %s\n\r", remote_gate.get_error_str());
    Serial.printf("remote gate angle: %f\n\r", remote_gate.get_angle());
#elif defined(GATE_LONG)
    Serial.printf("gate state: %s\n\r", local_gate.get_state_str());
    Serial.printf("gate angle: %f\n\r", local_gate.get_angle());
    Serial.printf("error code: %s\n\r", local_gate.get_error_str());
#endif
}

void BLDC_HandlerTask(void *pvParameters){
  while(1){
    BLDC.handler();
    vTaskDelay(1);
  }
}

void gate_HandlerTask(void *pvParameters){
  while(1){
    local_gate.loop();
    vTaskDelay(10);
  }
}

void main_task(void *pvParameters){
  while(1){
    // if (millis() - last_print_info_time > PRINT_INFO_INTERVAL) {
    //   last_print_info_time = millis();
    //   print_info();
    // }
#if defined(GATE_SHORT)
    mqtt_handler();
    rf_handler();
    master_gate_loop();
#elif defined(GATE_LONG)
    slave_gate_loop();
#endif
    vTaskDelay(10);
  }
}

void setup() {
  Serial.begin(115200);
  pinMode(led_pin, OUTPUT);
  pinMode(rf_pin, INPUT_PULLUP);
  delay(3000);
  Serial.println("Booted HoverGate V2!!!");

#if defined(GATE_SHORT)
  Serial.println("This is: SHORT GATE / MASTER");
  BLDC.set_hall_align_param(4.188790f, -1);
  bool ok = remote_gate.begin();
  if(!ok){
    Serial.println("Remote gate begin failed. Stopping here.");
    while(1);
  }
  wifi_client.setTimeout(1000);
  mqtt_client.setSocketTimeout(2);
  mqtt_client.setServer(mqtt_server, mqtt_port);
  mqtt_client.setBufferSize(512);
  mqtt_client.setCallback(mqtt_callback);

#elif defined(GATE_LONG)
  Serial.println("This is: LONG GATE / SLAVE");
  BLDC.set_hall_align_param(4.188791f, 1);
  local_gate.set_latch(&latch);
  int16_t comms_ret = comms.begin();
  if (comms_ret == -1) {
    Serial.println("Comms begin failed. Stopping here.");
    while(1);
  } else if (comms_ret == 1) {
    Serial.println("Comms begin ok, but no WiFi connection");
  } else {
    Serial.println("Comms begin success");
  }
#endif

  local_gate.set_driver(&BLDC);
  local_gate.begin();

  xTaskCreatePinnedToCore(
    BLDC_HandlerTask,  /* Task function. */
    "BLDC_HandlerTask",  /* String with name of task. */
    4095,  /* Stack size in bytes. */
    NULL,  /* Parameter passed as input of the task */
    20,  /* Priority of the task. */
    NULL,  /* Task handle. */
    1); // core 1
  Serial.println("Started BLDC_HandlerTask");

  xTaskCreate(
    gate_HandlerTask,  /* Task function. */
    "gate_HandlerTask",  /* String with name of task. */
    2048,  /* Stack size in bytes. */
    NULL,  /* Parameter passed as input of the task */
    1,  /* Priority of the task. */
    NULL);  /* Task handle. */
  Serial.println("Started gate_HandlerTask");

  xTaskCreate(
    main_task,  /* Task function. */
    "main_task",  /* String with name of task. */
    4095,  /* Stack size in bytes. */
    NULL,  /* Parameter passed as input of the task */
    1,  /* Priority of the task. */
    NULL);  /* Task handle. */
  Serial.println("Started main_task");

  Serial.println("Entering loop...");
  vTaskDelete(NULL);
}

void loop() {}
