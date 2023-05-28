#include "MqttClient.h"

TinyGsm *modem = NULL;
TinyGsmClient *client = NULL;
PubSubClient *mqtt = NULL;
Control control = {0, 0, 0, false, false, false, false};

Control* get_control() {
  return &control;
}

void callback(char *topic, byte *payload, unsigned int len) {
  if (strcmp(topic, TOPIC_CONTROL) == 0) {
    StaticJsonDocument<300> doc;
    DeserializationError error = deserializeJson(doc, payload);
    if (error) {
      return;
    }

    control.left = doc["left"];
    control.right = doc["right"];
    control.rev = doc["rev"];
    control.check_left = doc["cleft"];
    control.check_front = doc["cfront"];
    control.check_back = doc["cback"];
    control.check_right = doc["cright"];
  }
  if (strcmp(topic, TOPIC_PING) == 0) {
    byte* p = (byte*)malloc(len);
    memcpy(p,payload,len);
    mqtt->publish(TOPIC_CONNECTED, p, len);
    free(p);
  }
}

void init_mqtt(HardwareSerial &serial) {
  modem = new TinyGsm(serial);
  client = new TinyGsmClient(*modem);
  mqtt = new PubSubClient(*client);
}

void setup_modem() {
  Serial.println("GSM System Booting...");
  modem->restart();
  Serial.println("Modem Info: " + modem->getModemInfo());
  Serial.println("Searching for Telco Provider...");
  if (!modem->waitForNetwork()) {
    Serial.println("Telco Provider connection failed.");
    while (1)
      ;
  }
  Serial.println("Connected to Telco provider.");
  Serial.println("Signal Quality: " + String(modem->getSignalQuality()));
}

bool connect_gprs(const char *apn, const char *user, const char *pass) {
  return modem->gprsConnect(apn, user, pass);
}

void set_broker(const char *broker_url, uint16_t port) {
  mqtt->setServer(broker_url, port);
  mqtt->setCallback(callback);
}

bool connect_broker() {
  if (!mqtt->connect(CLIENT_ID)) {
    return false;
  }
  mqtt->subscribe(TOPIC_CONTROL);
  mqtt->subscribe(TOPIC_PING);
  return mqtt->connected();
}

void send_gps_data(GpsData &data) {
  StaticJsonDocument<56> doc;
  doc["longitude"] = data.longitude;
  doc["latitude"] = data.latitude;
  char payload[56];
  serializeJson(doc, payload);
  mqtt->publish(TOPIC_INFO_GPS, payload);
}

void send_lidar_data(LidarData &data) {
  StaticJsonDocument<55> doc;
  doc["front"] = data.front;
  doc["back"] = data.back;
  doc["left"] = data.left;
  doc["right"] = data.right;
  char payload[55];
  serializeJson(doc, payload);
  mqtt->publish(TOPIC_INFO_LIDAR, payload);
}

void send_gyro_data(GyroData &data) {
  StaticJsonDocument<97> doc;
  doc["gyroX"] = data.gyroX;
  doc["gyroY"] = data.gyroY;
  doc["gyroZ"] = data.gyroZ;
  char payload[97];
  serializeJson(doc, payload);
  mqtt->publish(TOPIC_INFO_GYRO, payload);
}

void send_accel_data(AccelData &data) {
  StaticJsonDocument<97> doc;
  doc["accelX"] = data.accelX;
  doc["accelY"] = data.accelY;
  doc["accelZ"] = data.accelZ;
  char payload[97];
  serializeJson(doc, payload);
  mqtt->publish(TOPIC_INFO_ACCEL, payload);
}

void send_dht_data(DhtData &data) {
  StaticJsonDocument<100> doc;
  doc["temperature"] = data.temp;
  doc["humidity"] = data.humid;
  char payload[100];
  serializeJson(doc, payload);
  mqtt->publish(TOPIC_INFO_DHT, payload);
}

void send_network_info() {
  StaticJsonDocument<150> doc;
  doc["strength"] = -113 + (2 * modem->getSignalQuality());
  doc["operator"] = modem->getOperator();
  doc["ipaddr"] = modem->getLocalIP();
  doc["imei"] = modem->getIMEI();
  char payload[150];
  serializeJson(doc, payload);
  mqtt->publish(TOPIC_INFO_NETWORK, payload);
}

void mqtt_msg_loop() {
  if (mqtt->connected()) {
    mqtt->loop();
  }
}
