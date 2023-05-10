#include "MqttClient.h"

uint8_t MqttClient::control = 5; // default enum value for stop in Dir Enum

MqttClient::MqttClient(SoftwareSerial &serial) {
    modem = new TinyGsm(serial);
    client = new TinyGsmClient(*modem);
    mqtt = new PubSubClient(*client);
}

void MqttClient::setup_modem() {
  Serial.println(F("GSM System Booting..."));
  modem->restart();
  Serial.print(F("Modem Info: "));
  Serial.println(modem->getModemInfo());
  Serial.println(F("Searching for Telco Provider..."));
  if (!modem->waitForNetwork()) {
    Serial.println(F("Telco Provider connection failed."));
    while (1);
  }
  Serial.println(F("Connected to Telco provider."));
  Serial.println(F("Signal Quality: "));
  Serial.println(modem->getSignalQuality());
}

bool MqttClient::connect_gprs(const char *apn, const char *user, const char *pass) {
  return modem->gprsConnect(apn, user, pass);
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
  mqtt->publish(TOPIC_CONNECTED,"true");
  send_network_info();
  return mqtt->connected();
}

void MqttClient::mqtt_callback(char* topic, byte* payload, unsigned int len) {
  MqttClient::control = atoi(payload);
}

void MqttClient::send_gps_data(float lat, float lon) {
  char payload[56];
  sprintf(payload, "{\"latitude\":%f,\"longitude\":%f}", lat, lon);
  mqtt->publish(TOPIC_INFO_GPS, payload);
}

void MqttClient::send_lidar_data(uint16_t front, uint16_t right, uint16_t back, uint16_t left) {
  char payload[55];
  sprintf(payload, "{\"front\":%i,\"back\":%i,\"left\":%i,\"right\":%i}", front, back, left, right);
  mqtt->publish(TOPIC_INFO_LIDAR, payload);
}

void MqttClient::send_motor_data(byte left, byte right) {
  char payload[34];
  sprintf(payload, "{\"left\":%i,\"right\":%i}", left, right);
  mqtt->publish(TOPIC_INFO_MOTOR, payload);
}

void MqttClient::send_gyro_data(float x, float y, float z) {
  char payload[97];
  sprintf(payload, "{\"gyroX\":%f,\"gyroY\":%f,\"gyroZ\":%f}", x, y, z);
  mqtt->publish(TOPIC_INFO_GYRO, payload);
}

void MqttClient::send_accel_data(float x, float y, float z) {
  char payload[97];
  sprintf(payload, "{\"accelX\":%f,\"accelY\":%f,\"accelZ\":%f}", x, y, z);
  mqtt->publish(TOPIC_INFO_ACCEL, payload);
}

void MqttClient::send_network_info() {
  char payload[200];
  sprintf(payload, "{\"strength\":%d,\"operator\":%s}", -113 + (2 * modem->getSignalQuality()), modem->getOperator());
  mqtt->publish(TOPIC_INFO_NETWORK, payload);
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
