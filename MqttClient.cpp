#include "MqttClient.h"

uint8_t MqttClient::control = 5; // default enum value for stop in Dir Enum

MqttClient::MqttClient(HardwareSerial &serial) {
    modem = new TinyGsm(serial);
    client = new TinyGsmClient(*modem);
    mqtt = new PubSubClient(*client);
}

bool MqttClient::is_network_ready() {
  return !modem->waitForNetwork();
}

bool MqttClient::connect_gprs(const char *apn, const char *user, const char *pass) {
  return modem->gprsConnect(apn, user, pass);
}

void MqttClient::restart_modem() {
  modem->restart();
}

void MqttClient::set_broker(const char *broker_url, uint16_t port) {
  mqtt->setServer(broker_url, port);
  mqtt->setCallback(mqtt_callback);
}

bool MqttClient::connect_broker() {
  if (!mqtt->connect(CLIENT_ID)) {
    return false;
  }
  mqtt->subscribe(TOPIC_INFO_GPS);
  mqtt->subscribe(TOPIC_INFO_LIDAR);
  mqtt->subscribe(TOPIC_INFO_MOTOR);
  mqtt->subscribe(TOPIC_INFO_GYRO);
  mqtt->subscribe(TOPIC_CONTROL);
  return mqtt->connected();
}

void MqttClient::mqtt_callback(char* topic, byte* payload, unsigned int len) {
  if (!strcmp(topic, TOPIC_CONTROL)) {
    return;
  }
  StaticJsonDocument<100> doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    return;
  }
  MqttClient::control = doc["id"];
}

void MqttClient::send_gps_data(GpsData &data) {
  StaticJsonDocument<200> doc;
  doc["longitude"] = data.longitude;
  doc["latitude"] = data.latitude;
  char payload[200];
  serializeJson(doc, payload);
  mqtt->publish(TOPIC_INFO_GPS, payload);
}

void MqttClient::send_lidar_data(LidarData &data) {
  StaticJsonDocument<300> doc;
  doc["front"] = data.front;
  doc["back"] = data.back;
  doc["left"] = data.left;
  doc["right"] = data.right;
  char payload[300];
  serializeJson(doc, payload);
  mqtt->publish(TOPIC_INFO_LIDAR, payload);
}

void MqttClient::send_motor_data(MotorData &data) {
  StaticJsonDocument<150> doc;
  doc["leftSpeed"] = data.left;
  doc["rightSpeed"] = data.right;
  char payload[150];
  serializeJson(doc, payload);
  mqtt->publish(TOPIC_INFO_MOTOR, payload);
}

void MqttClient::send_gyro_data(GyroData &data) {
  StaticJsonDocument<400> doc;
  doc["xAxis"] = data.x;
  doc["yAxis"] = data.y;
  doc["zAxis"] = data.z;
  doc["accel"] = data.accel;
  char payload[400];
  serializeJson(doc, payload);
  mqtt->publish(TOPIC_INFO_GYRO, payload);
}

void MqttClient::loop() {
  if (mqtt->connected()) {
    mqtt->loop();
  }
}

MqttClient::~MqttClient() {
  free(mqtt);
  free(client);
  free(modem);
}