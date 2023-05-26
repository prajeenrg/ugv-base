#define TINY_GSM_MODEM_SIM800

#include <HardwareSerial.h>
#include <TinyGsmClient.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>
#include <string.h>
#include "FastIMU.h"
#include "UgvDataTypes.h"

#define CLIENT_ID "MITECEUGV-ROVER"

// topics
#define TOPIC_INFO_ACCEL "c83929b2-c031-11ed-afa1-0242ac120002/info/accel"
#define TOPIC_INFO_DHT "c83929b2-c031-11ed-afa1-0242ac120002/info/dht"
#define TOPIC_INFO_LIDAR "c83929b2-c031-11ed-afa1-0242ac120002/info/lidar"
#define TOPIC_INFO_GPS "c83929b2-c031-11ed-afa1-0242ac120002/info/gps"
#define TOPIC_INFO_GYRO "c83929b2-c031-11ed-afa1-0242ac120002/info/gyro"
#define TOPIC_INFO_MOTOR "c83929b2-c031-11ed-afa1-0242ac120002/info/motor"
#define TOPIC_CONTROL "c83929b2-c031-11ed-afa1-0242ac120002/control"
#define TOPIC_CONNECTED "c83929b2-c031-11ed-afa1-0242ac120002/connection"
#define TOPIC_INFO_NETWORK "c83929b2-c031-11ed-afa1-0242ac120002/info/network"

class MqttClient {
  TinyGsm *modem;
  TinyGsmClient *client;
  PubSubClient *mqtt;

public:
  static Control control;

  MqttClient(HardwareSerial &);
  ~MqttClient();
  bool connect_broker();
  bool connect_gprs(const char *, const char *, const char *);
  void set_broker(const char *, uint16_t);
  void loop();
  void setup_modem();
  void send_accel_data(AccelData &);
  void send_dht_data(DhtData &);
  void send_lidar_data(LidarData &);
  void send_gps_data(GpsData &);
  void send_gyro_data(GyroData &);
  void send_motor_data(MotorData &);
  void send_network_info();
  static void mqtt_callback(char *, byte *, unsigned int);
};