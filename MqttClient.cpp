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
  mqtt->subscribe(TOPIC_CONTROL);
  mqtt->publish(TOPIC_CONNECT_READY,"{\"connected\":true}");
  return mqtt->connected();
}

void MqttClient::mqtt_callback(char* topic, byte* payload, unsigned int len) {
  if (strcmp(topic, TOPIC_CONTROL)) {
    return;
  }
  StaticJsonDocument<24> doc;
  DeserializationError error = deserializeJson(doc, payload);
  if (error) {
    return;
  }
  MqttClient::control = doc["id"];
}

void MqttClient::send_gps_data(GpsData &data) {
  StaticJsonDocument<56> doc;
  doc["longitude"] = data.longitude;
  doc["latitude"] = data.latitude;
  char payload[56];
  serializeJson(doc, payload);
  mqtt->publish(TOPIC_INFO_GPS, payload);
}

void MqttClient::send_lidar_data(LidarData &data) {
  StaticJsonDocument<55> doc;
  doc["front"] = data.front;
  doc["back"] = data.back;
  doc["left"] = data.left;
  doc["right"] = data.right;
  char payload[55];
  serializeJson(doc, payload);
  mqtt->publish(TOPIC_INFO_LIDAR, payload);
}

void MqttClient::send_motor_data(MotorData &data) {
  StaticJsonDocument<34> doc;
  doc["leftSpeed"] = data.left;
  doc["rightSpeed"] = data.right;
  char payload[34];
  serializeJson(doc, payload);
  mqtt->publish(TOPIC_INFO_MOTOR, payload);
}

void MqttClient::send_gyro_data(GyroData &data) {
  StaticJsonDocument<97> doc;
  doc["xAxis"] = data.x;
  doc["yAxis"] = data.y;
  doc["zAxis"] = data.z;
  doc["accel"] = data.accel;
  char payload[97];
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
