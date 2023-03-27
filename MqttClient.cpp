#include "MqttClient.h"

uint8_t MqttClient::control = 5; // default enum value for stop in Dir Enum

MqttClient::MqttClient(HardwareSerial &serial) {
    modem = new TinyGsm(serial);
    client = new TinyGsmClient(*modem);
    mqtt = new PubSubClient(*client);
}

void MqttClient::setup_modem() {
  Serial.println("GSM System Booting...");
  modem->restart();
  Serial.println("Modem Info: " + modem->getModemInfo());
  Serial.println("Searching for Telco Provider...");
  if (!modem->waitForNetwork()) {
    Serial.println("Telco Provider connection failed.");
    while (1);
  }
  Serial.println("Connected to Telco provider.");
  Serial.println("Signal Quality: " + String(modem->getSignalQuality()));
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
  if (strcmp(topic, TOPIC_CONTROL)) {
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
  doc["gyroX"] = data.gyroX;
  doc["gyroY"] = data.gyroY;
  doc["gyroZ"] = data.gyroZ;
  char payload[97];
  serializeJson(doc, payload);
  mqtt->publish(TOPIC_INFO_GYRO, payload);
}

void MqttClient::send_accel_data(AccelData &data) {
  StaticJsonDocument<97> doc;
  doc["accelX"] = data.accelX;
  doc["accelY"] = data.accelY;
  doc["accelZ"] = data.accelZ;
  char payload[97];
  serializeJson(doc, payload);
  mqtt->publish(TOPIC_INFO_ACCEL, payload);
}

void MqttClient::send_network_info() {
  StaticJsonDocument<300> doc;
  doc["strength"] = -113 + (2 * modem->getSignalQuality());
  doc["operator"] = modem->getOperator();
  doc["ipaddr"] = modem->getLocalIP();
  doc["volt"] = modem->getBattVoltage();
  doc["imei"] = modem->getIMEI();
  doc["imsi"] = modem->getIMSI();
  doc["ccid"] = modem->getSimCCID();
  char payload[300];
  serializeJson(doc, payload);
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
