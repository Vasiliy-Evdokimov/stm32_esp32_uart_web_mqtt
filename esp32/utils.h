#ifndef UTILS_H
#define UTILS_H

#define BUFFER_SIZE 32

#define SENSORS_COUNT 3
#define DEVICES_COUNT 2 

#define MQTT_CLIENT_ID "Evdokimov_VI_ESP32"
#define MQTT_ROOT "Evdokimov_VI/bte_final"

#define MODE_PERIODIC 1
#define MODE_IFCHANGED 2
#define MODE_BYCOMMAND 3

#define SENSOR_TEMPERATURE 1
#define SENSOR_HUMIDITY 2
#define SENSOR_AMBIENT 3

#define DEVICE_LED_RED 1
#define DEVICE_LED_BLUE 2

#define CMD_GET_SENSORS 1
//
#define CMD_SET_MODE 2
#define CMD_GET_MODE 3
//
#define CMD_SET_ALERTS 4
#define CMD_GET_ALERTS 5
//
#define CMD_SET_DEVICES 6
#define CMD_GET_DEVICES 7
//
#define CMD_GET_STATUS 8

#define COMPARE_EQUAL 1
#define COMPARE_LESS 2
#define COMPARE_GREATER 3

#define MSG_MODE 1
#define MSG_SENSORS 2
#define MSG_DEVICES 3

#define ERR_CRC_FAILED 1

typedef struct {
	uint8_t id;
	uint8_t type;  
	uint8_t value;
  uint8_t previous_value;
	uint8_t alert_check;
	uint8_t alert_compare;
  uint8_t alert_value;
	uint8_t alert_flag;
} sensor;

typedef struct {  
  uint8_t type;  
  uint8_t value;
} device_state;

typedef struct {
  uint8_t type;
  uint8_t period;
  uint8_t percents;
} mode;

uint8_t getCRC(uint8_t aDataLen, uint8_t *aData);

void initSensors();

void initDevicesStates();

void initMode();

sensor* getSensorByType(uint8_t aSensorType);

uint8_t checkSensorsAlert();

uint8_t checkSensorsPercents(uint8_t aPercents);

device_state* getDeviceStateByType(uint8_t aDeviceType);

void fillTxCRC(uint8_t *aTx);

#endif
