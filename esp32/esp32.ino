#include <WiFi.h>
#include <WiFiClient.h>
#include <WebServer.h>

#include <PubSubClient.h>

#include "utils.c"
#include "auth.h"
#include "html_page.h"

#define UART_BAUDRATE 9600
#define RXD2 16
#define TXD2 17

WebServer server(80);
WiFiClient espClient;
PubSubClient client(espClient);

const char* sensor_names[] = {
  "temperature",
  "humidity",
  "ambient"
};

const char* device_names[] = {
  "led_red",
  "led_blue"
};

uint8_t tx[BUFFER_SIZE] = {0};
uint8_t rx[BUFFER_SIZE] = {0};

char mqtt_topic[128];
char mqtt_value[20];

extern sensor sensors[SENSORS_COUNT];
extern device_state devices_states[DEVICES_COUNT];
extern mode current_mode;

uint8_t rx0, rx1, idx;
sensor* psensor;
device_state* pdevice_state;

void print_buffer(uint8_t* aBuf, String aPrefix) {
  for (int i = 0; i < aBuf[1]; i++) 
    Serial.println(aPrefix + "_" + String(i) + "=" + String(aBuf[i]));
}

void DoUartTransmit() {
  if (!tx[0]) return;
  Serial2.flush();
  Serial2.write(tx, BUFFER_SIZE);
}

void getCompleteStatus() {
  memset(tx, 0, BUFFER_SIZE);
  tx[0] = CMD_GET_STATUS;
  tx[1] = 2;
  fillTxCRC(tx);
  //
  DoUartTransmit();
}

void handleRoot() {  
  Serial.println("handleRoot()");
  //
  getCompleteStatus();
  //
  server.send(200, "text/html", main_page);
}

void handleGetStatus() {
  String json = "{"; 
  //  
  json += String("\"mode\": {") + 
    "\"type\": " + String(current_mode.type) + ", " +  
    "\"period\": " + String(current_mode.period) + ", " + 
    "\"percent\": " + String(current_mode.percents) +
  "}";  
  //
  json += ", \"sensors\": [";  
  for (int i = 0; i < SENSORS_COUNT; i++) {
      json += "{\"name\": \"" + String(sensor_names[i]) +  "\", " + 
        "\"data\": {" + 
          "\"value\": " + String(sensors[i].value) + ", " +
          "\"alert_check\": " + String(sensors[i].alert_check) + ", " +
          "\"alert_compare\": " + String(sensors[i].alert_compare) +  ", " +
          "\"alert_value\": " + String(sensors[i].alert_value) +  ", " +          
          "\"alert_flag\": " + String(sensors[i].alert_flag) + 
        "}}";
      json += (i < SENSORS_COUNT - 1) ? ", " : "";         
  }
  json += "]";
  //  
  json += ", \"devices\": [";  
  for (int i = 0; i < DEVICES_COUNT; i++) {
    json += String("{") +
      "\"name\": \"" + String(device_names[i]) +  "\", " + 
      "\"value\": " + String(devices_states[i].value) + 
    "}";
    json += (i < DEVICES_COUNT - 1) ? ", " : "";
  }  
  json += "]";  
  //
  json += "}";
  //  
  //Serial.println("json = " + json);
  //
  server.send(200, "text/plane", json);
}

void handleSetMode() {
  Serial.println("handleSetMode()");
  uint8_t modeID = server.arg("mode").toInt();
  uint8_t param = 0;
  //
  if (modeID == MODE_PERIODIC) 
    param = server.arg("check_period").toInt();
  if (modeID == MODE_IFCHANGED) 
    param = server.arg("check_percents").toInt();
  //
  memset(tx, 0, BUFFER_SIZE);
  tx[0] = CMD_SET_MODE;
  tx[1] = 4;
  tx[2] = modeID;
  tx[3] = param;
  fillTxCRC(tx);
  //
  DoUartTransmit();
  //
  Serial.println("modeID=" + String(modeID) + " param=" + String(param));
  server.send(200, "text/plane", 0);
}

void handleSetAlerts() {
  Serial.println("handleSetAlerts()");    
  //  
  String s;
  memset(tx, 0, BUFFER_SIZE);
  tx[0] = CMD_SET_ALERTS;
  uint8_t i = 2;
  for (int j = 0; j < SENSORS_COUNT; j++) {    
    s = String(sensor_names[j]);
    tx[i++] = sensors[j].type;
    tx[i++] = server.arg(s + "_alert_check").toInt();
    tx[i++] = server.arg(s + "_alert_compare").toInt();
    tx[i++] = server.arg(s + "_alert_value").toInt();
  }
  tx[1] = i;
  fillTxCRC(tx);
  //
  print_buffer(tx, "TX");
  //
  DoUartTransmit();
  //
  server.send(200, "text/plane", 0);
}

void handleSetDevices() {
  Serial.println("handleSetDevices()");    
  //
  String s, a;
  memset(tx, 0, BUFFER_SIZE);
  tx[0] = CMD_SET_DEVICES;
  uint8_t i = 2;
  for (int j = 0; j < DEVICES_COUNT; j++) {
    s = String(device_names[j]) + "_switch";
    a = server.arg(s);    
    tx[i++] = j + 1;
    tx[i++] = a.toInt();    
    Serial.println(s + "=" + a);
  }
  tx[1] = i;
  fillTxCRC(tx);
  //
  DoUartTransmit();
  //
  server.send(200, "text/plane", 0);
}

void mqttconnect() {
  while (!client.connected()) {
    Serial.print("MQTT connecting... ");
    String clientId = MQTT_CLIENT_ID;
    if (client.connect(clientId.c_str())) {
      Serial.println("Connected!");
    } else {
      Serial.print("Failed, status code = ");
      Serial.print(client.state());
      Serial.println(" Try again in 5 seconds...");
      delay(5000);
    }
  }
}

void initWebServer() {  
  server.on("/", handleRoot);
  server.on("/getStatus", handleGetStatus);
  server.on("/setMode", handleSetMode);
  server.on("/setAlerts", handleSetAlerts);
  server.on("/setDevices", handleSetDevices);
}

void setup() {  
  Serial.begin(115200);
  Serial2.begin(UART_BAUDRATE, SERIAL_8N1, RXD2, TXD2);
  //
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println(ssid);
  //
  WiFi.begin(ssid, password); 
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  //
  Serial.println("");
  Serial.println("WiFi connected!");
  Serial.println("IP address: ");
  Serial.println(WiFi.localIP());
  //  
  client.setServer(mqtt_server, mqtt_port);
  mqttconnect();
  //
  initWebServer();
  server.begin();
  Serial.println("HTTP server started!");
  //
  initMode();
  initSensors();
  initDevicesStates();  
  //
  publishAll();
  //
  delay(500);
  Serial.println("Setup!");  
  //
  getCompleteStatus();    
}

void publishMode() {  
  snprintf(mqtt_topic, 128, "%s/mode/type", MQTT_ROOT);
  snprintf(mqtt_value, 20, "%d", current_mode.type);
  client.publish(mqtt_topic, mqtt_value);
  snprintf(mqtt_topic, 128, "%s/mode/period", MQTT_ROOT);
  snprintf(mqtt_value, 20, "%d", current_mode.period);
  client.publish(mqtt_topic, mqtt_value);
  snprintf(mqtt_topic, 128, "%s/mode/percents", MQTT_ROOT);
  snprintf(mqtt_value, 20, "%d", current_mode.percents);
  client.publish(mqtt_topic, mqtt_value);
  //
  Serial.println("Mode has been published!");
}

void publishSensors() {
  for (int i = 0; i < SENSORS_COUNT; i++) {
    psensor = getSensorByType(i + 1);      
    if (!psensor) continue;
    //
    snprintf(mqtt_topic, 128, "%s/sensors/%s/value", MQTT_ROOT, sensor_names[i]);
    snprintf(mqtt_value, 20, "%d", psensor->value);
    client.publish(mqtt_topic, mqtt_value);
    snprintf(mqtt_topic, 128, "%s/sensors/%s/alert_check", MQTT_ROOT, sensor_names[i]);
    snprintf(mqtt_value, 20, "%d", psensor->alert_check);
    client.publish(mqtt_topic, mqtt_value);
    snprintf(mqtt_topic, 128, "%s/sensors/%s/alert_compare", MQTT_ROOT, sensor_names[i]);
    snprintf(mqtt_value, 20, "%d", psensor->alert_compare);
    client.publish(mqtt_topic, mqtt_value);
    snprintf(mqtt_topic, 128, "%s/sensors/%s/alert_value", MQTT_ROOT, sensor_names[i]);
    snprintf(mqtt_value, 20, "%d", psensor->alert_value);
    client.publish(mqtt_topic, mqtt_value);
    snprintf(mqtt_topic, 128, "%s/sensors/%s/alert_flag", MQTT_ROOT, sensor_names[i]);
    snprintf(mqtt_value, 20, "%d", psensor->alert_flag);
    client.publish(mqtt_topic, mqtt_value);      
  }     
  Serial.println("Sensors have been published!");  
}

void publishDevices() {
  for (int i = 0; i < DEVICES_COUNT; i++) {
    pdevice_state = getDeviceStateByType(i + 1);    
    if (!pdevice_state) continue;
    //
    snprintf(mqtt_topic, 128, "%s/devices/%s", MQTT_ROOT, device_names[i]);
    snprintf(mqtt_value, 20, "%d", pdevice_state->value);
    client.publish(mqtt_topic, mqtt_value);
  }
  Serial.println("Devices have been published!");
}

void publishAll() {
  publishMode();
  publishSensors();
  publishDevices();
}

void loop() {  
  
  while (Serial2.available()) {
    Serial2.readBytes(rx, BUFFER_SIZE);
    rx0 = rx[0];
    rx1 = rx[1];
    //    
    uint8_t crc = getCRC(rx1, rx);    
    //
    if (crc != rx[rx1]) {
      Serial.println("CRC failed!");
      //rx0 = 0;
      break;      
    }
    //
    print_buffer(rx, "RX");
    //
    if (rx0 == MSG_MODE) {
      current_mode.type = rx[2];
      current_mode.period = rx[3];
      current_mode.percents = rx[4];
      //
      publishMode();
    } else
    //
    if (rx0 == MSG_SENSORS) {
      for (int i = 0; i < rx1 / 6; i++) {
        idx = 2 + i * 6;
        psensor = getSensorByType(rx[idx]);
        if (!psensor) continue;
        psensor->value = rx[idx + 1];        
        psensor->alert_check = rx[idx + 2];
        psensor->alert_compare = rx[idx + 3];
        psensor->alert_value = rx[idx + 4];
        psensor->alert_flag = rx[idx + 5];
      }     
      //
      publishSensors();
    } else
    //
    if (rx0 == MSG_DEVICES) {
       for (int i = 0; i < rx1 / 2; i++) {
        idx = 2 + i * 2;
        pdevice_state = getDeviceStateByType(rx[idx]);
        if (!pdevice_state) continue;
        pdevice_state->value = rx[idx + 1];
      }
      //
      publishDevices();
    }
        
  } 
  //  
  if (!client.connected())
    mqttconnect();     
  //
  client.loop();
  //
  server.handleClient();
  delay(1);  
  
}
