#define TINY_GSM_MODEM_SIM800

#include <HardwareSerial.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <string.h>
#include "UgvDataTypes.h"

#define CLIENT_ID "MITECEUGV-ROVER"

// topics
#define TOPIC_INFO_GPS "c83929b2-c031-11ed-afa1-0242ac120002/info/gps"
#define TOPIC_INFO_LIDAR "c83929b2-c031-11ed-afa1-0242ac120002/info/lidar"
#define TOPIC_INFO_MOTOR "c83929b2-c031-11ed-afa1-0242ac120002/info/motor"
#define TOPIC_INFO_GYRO "c83929b2-c031-11ed-afa1-0242ac120002/info/gyro"
#define TOPIC_CONTROL "c83929b2-c031-11ed-afa1-0242ac120002/control"

class MqttClient {
  TinyGsm *modem;
  TinyGsmClient *client;
  PubSubClient *mqtt;

  public:
  static uint8_t control;

  MqttClient(HardwareSerial&);
  ~MqttClient();
  bool connect_broker();
  bool is_network_ready();
  bool connect_gprs(const char*, const char*, const char*);
  void restart_modem();
  void set_broker(const char*, uint16_t);
  void loop();
  void send_lidar_data(LidarData&);
  void send_gps_data(GpsData&);
  void send_motor_data(MotorData&);
  void send_gyro_data(GyroData&);
  static void mqtt_callback(char*, byte*, unsigned int);
  uint8_t get_control();
};