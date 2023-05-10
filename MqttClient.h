#define TINY_GSM_MODEM_SIM900

#include <SoftwareSerial.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <string.h>

#define CLIENT_ID "MITECEUGV-ROVER"

// topics
#define TOPIC_INFO_GPS "c83929b2-c031-11ed-afa1-0242ac120002/info/gps"
#define TOPIC_INFO_LIDAR "c83929b2-c031-11ed-afa1-0242ac120002/info/lidar"
#define TOPIC_INFO_MOTOR "c83929b2-c031-11ed-afa1-0242ac120002/info/motor"
#define TOPIC_INFO_GYRO "c83929b2-c031-11ed-afa1-0242ac120002/info/gyro"
#define TOPIC_INFO_ACCEL "c83929b2-c031-11ed-afa1-0242ac120002/info/accel"
#define TOPIC_CONTROL "c83929b2-c031-11ed-afa1-0242ac120002/control"
#define TOPIC_CONNECTED "c83929b2-c031-11ed-afa1-0242ac120002/connection"
#define TOPIC_INFO_NETWORK "c83929b2-c031-11ed-afa1-0242ac120002/info/network"

class MqttClient {
  TinyGsm *modem;
  TinyGsmClient *client;
  PubSubClient *mqtt;

  public:
  static uint8_t control;

  MqttClient(SoftwareSerial&);
  ~MqttClient();
  bool connect_broker();
  bool connect_gprs(const char*, const char*, const char*);
  void set_broker(const char*, uint16_t);
  void loop();
  void setup_modem();
  void send_lidar_data(uint16_t, uint16_t, uint16_t, uint16_t);
  void send_gps_data(float, float);
  void send_motor_data(byte, byte);
  void send_gyro_data(float, float, float);
  void send_accel_data(float, float, float);
  void send_network_info();
  static void mqtt_callback(char*, byte*, unsigned int);
  uint8_t get_control();
};