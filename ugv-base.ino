#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include "dht11.h"
#include "FastIMU.h"
#include "Gpsneo.h"
#include "MqttClient.h"
#include "UgvDataTypes.h"

#define GREATER_THAN(x, y) x > y

#define SAFE_DISTANCE_MM 50

/* aux pins */
#define DHT_IO PB5
#define IND_PIN PC13

/* motor controls pins */
#define MOTOR_AB1 PB12
#define MOTOR_AB2 PB13
#define MOTOR_CD1 PB14
#define MOTOR_CD2 PB15

/* lidar xshut pins */
#define XSHUT_1 PA5
#define XSHUT_2 PA6
#define XSHUT_3 PA4
#define XSHUT_4 PA7

/* gsm uart pins */
#define GSM_TX PA3
#define GSM_RX PA2

/* gps uart pins */
#define GPS_TX PA10
#define GPS_RX PA9

/* tof address setting */
#define TOF_ADDR_1 0x62
#define TOF_ADDR_2 0x61
#define TOF_ADDR_3 0x63
#define TOF_ADDR_4 0x60

/* mqtt broker settings */
#define BROKER_URL "test.mosquitto.org"
#define BROKER_PORT 1883

/* gsm apn settings */
#define AIRTEL_APN "airtelgprs.com"
#define AIRTEL_USER ""
#define AIRTEL_PASS ""

#define MPU_ADDR 0x68  // i2c address for mpu6500

bool status_led[] = {false, false, false, false, false, false, false, false, false, false};
uint8_t sled_i = 0;  // indicates which status to check

/* Global Data Carrying Variables */
AccelData accelData;
DhtData dhtData;
LidarData lidardata;
GpsData gpsdata;
GyroData gyroData;

Gpsneo gps(GPS_TX, GPS_RX);
Adafruit_VL53L0X lox;
HardwareSerial GSM(GSM_TX, GSM_RX);
MPU6500 mpu;
dht11 dht;

void status_led_update(bool *status_led, uint8_t *idx) {
  if (status_led[*idx]) {
    digitalWrite(IND_PIN, HIGH);
    delay(50);
    digitalWrite(IND_PIN, LOW);
  } else {
    digitalWrite(IND_PIN, LOW);
  }
  *idx = (++*idx) % 10;
}

void setup_gsm() {
  setup_modem();
  while (!connect_gprs(AIRTEL_APN, AIRTEL_USER, AIRTEL_PASS)) {
    Serial.println(F("Failed initiating GPRS network"));
    Serial.println(F("Retrying GPRS establishment after 1s..."));
    delay(1000);
  }
  status_led[8] = true;
  Serial.println("Connected to GPRS: " + String(AIRTEL_APN));
  Serial.println(F("Connecting to MQTT Broker..."));
  set_broker(BROKER_URL, BROKER_PORT);
  connect_broker();
  while (!connect_broker()) {
    Serial.println(F("Failed in connecting to MQTT Broker"));
    Serial.println(F("Retrying connection after 1s..."));
    delay(1000);
  }
  status_led[9] = true;
  Serial.println(F("Connected to MQTT Broker."));
}

void setup_mpu6500() {
  calData calib = { 0 };
  int err = mpu.init(calib, MPU_ADDR);
  if (err != 0) {
    Serial.print(F("Error initializing MPU6500, error code: "));
    Serial.println(err);
    while (1)
      ;
  }
  // perform calibration for better results
  mpu.calibrateAccelGyro(&calib);
  mpu.init(calib, MPU_ADDR);
  status_led[5] = true;
}

void get_gps_data() {
  char glat[50], glong[50];
  gps.getDataGPRMC(glat, glong);
  gpsdata.latitude = gps.convertLatitude(glat);
  gpsdata.longitude = gps.convertLongitude(glong);
  send_gps_data(gpsdata);
  Serial.printf("Latitude: %f, Longitude: %f\n", gpsdata.latitude, gpsdata.longitude);
}

void move_vehicle() {
  int mode = 0;
  int left = get_control()->left;
  int right = get_control()->right;
  if (get_control()->rev) {
    analogWrite(MOTOR_AB1, LOW);
    analogWrite(MOTOR_AB2, left);
    analogWrite(MOTOR_CD1, LOW);
    analogWrite(MOTOR_CD2, right);
  } else {
    analogWrite(MOTOR_AB1, left);
    analogWrite(MOTOR_AB2, LOW);
    analogWrite(MOTOR_CD1, right);
    analogWrite(MOTOR_CD2, LOW);
  }

  Serial.println(F("Motor Data:"));
  Serial.printf("Left Speed: %d\n", left);
  Serial.printf("Right Speed: %d\n", right);
}

void apply_brake() {
  digitalWrite(MOTOR_AB1, LOW);
  digitalWrite(MOTOR_AB2, LOW);
  digitalWrite(MOTOR_CD1, LOW);
  digitalWrite(MOTOR_CD2, LOW);
}

int get_dhtdata() {
  Serial.print("DHT Fetch Status: ");
  int chk = dht.read(DHT_IO);
  switch (chk) {
    case DHTLIB_OK:
      Serial.println("OK");
      dhtData.temp = dht.temperature;
      dhtData.humid = dht.humidity;
      send_dht_data(dhtData);
      break;
    case DHTLIB_ERROR_CHECKSUM:
      Serial.println("Checksum error");
      break;
    case DHTLIB_ERROR_TIMEOUT:
      Serial.println("Time out error");
      break;
    default:
      Serial.println("Unknown error");
      break;
  }
  return chk;
}

void get_mpudata() {
  mpu.update();
  mpu.getAccel(&accelData);
  mpu.getGyro(&gyroData);
  send_gyro_data(gyroData);
  send_accel_data(accelData);

  Serial.println(F("Gyroscope Data:"));
  Serial.printf("X axis: %f\n", gyroData.gyroX);
  Serial.printf("Y axis: %f\n", gyroData.gyroY);
  Serial.printf("Z axis: %f\n", gyroData.gyroZ);
  Serial.println(F("Accelerometer Data:"));
  Serial.printf("X axis: %f\n", accelData.accelX);
  Serial.printf("Y axis: %f\n", accelData.accelY);
  Serial.printf("Z axis: %f\n", accelData.accelZ);
}

bool setTofAddress() {
  digitalWrite(XSHUT_2, LOW);
  digitalWrite(XSHUT_3, LOW);
  digitalWrite(XSHUT_4, LOW);

  if (!lox.begin(TOF_ADDR_1, false)) {
    Serial.println(F("FTOF cannot be found"));
    return true;
  } else {
    lox.startRangeContinuous();
    status_led[1] = true;
  }

  digitalWrite(XSHUT_2, HIGH);
  if (!lox.begin(TOF_ADDR_2, false)) {
    Serial.println(F("RTOF cannot be found"));
    return true;
  } else {
    lox.startRangeContinuous();
    status_led[2] = true;
  }

  digitalWrite(XSHUT_3, HIGH);
  if (!lox.begin(TOF_ADDR_3, false)) {
    Serial.println(F("BTOF cannot be found"));
    return true;
  } else {
    lox.startRangeContinuous();
    status_led[3] = true;
  }

  digitalWrite(XSHUT_4, HIGH);
  if (!lox.begin(TOF_ADDR_4, false)) {
    Serial.println(F("LTOF cannot be found"));
    return true;
  } else {
    lox.startRangeContinuous();
    status_led[4] = true;
  }

  Serial.println(F("All TOFs ready for launch"));
  return false;
}

inline void resetTof() {
  digitalWrite(XSHUT_4, LOW);
  digitalWrite(XSHUT_2, LOW);
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_3, LOW);
  delay(10);
  digitalWrite(XSHUT_4, HIGH);
  digitalWrite(XSHUT_2, HIGH);
  digitalWrite(XSHUT_1, HIGH);
  digitalWrite(XSHUT_3, HIGH);
  delay(10);
  status_led[1] = status_led[2] = status_led[3] = status_led[4] = false;
}

void get_ranging_results() {
  lox.begin(TOF_ADDR_4, false);
  lidardata.left = lox.readRange();

  lox.begin(TOF_ADDR_2, false);
  lidardata.right = lox.readRange();

  lox.begin(TOF_ADDR_1, false);
  lidardata.front = lox.readRange();

  lox.begin(TOF_ADDR_3, false);
  lidardata.back = lox.readRange();

  send_lidar_data(lidardata);

  Serial.println(F("TOF Ranging Results:"));
  Serial.printf("TOF 1: %d\n", lidardata.front);
  Serial.printf("TOF 2: %d\n", lidardata.right);
  Serial.printf("TOF 3: %d\n", lidardata.back);
  Serial.printf("TOF 4: %d\n", lidardata.left);
}

bool check_obstacle() {
  return (get_control()->check_front && GREATER_THAN(lidardata.front, SAFE_DISTANCE_MM)) || 
    (get_control()->check_back && GREATER_THAN(lidardata.back, SAFE_DISTANCE_MM)) || 
    (get_control()->check_left && GREATER_THAN(lidardata.left, SAFE_DISTANCE_MM)) || 
    (get_control()->check_right && GREATER_THAN(lidardata.right, SAFE_DISTANCE_MM));
}

void setup() {
  Serial.begin(115200);
  Serial.println(F("Initializing the pins and peripherals..."));

  Serial.println(F("Setting pin modes..."));
  pinMode(MOTOR_AB1, OUTPUT);
  pinMode(MOTOR_AB2, OUTPUT);
  pinMode(MOTOR_CD1, OUTPUT);
  pinMode(MOTOR_CD2, OUTPUT);
  pinMode(XSHUT_4, OUTPUT);
  pinMode(XSHUT_1, OUTPUT);
  pinMode(XSHUT_3, OUTPUT);
  pinMode(XSHUT_2, OUTPUT);
  pinMode(IND_PIN, OUTPUT);
  digitalWrite(IND_PIN, LOW);
  digitalWrite(MOTOR_AB1, LOW);
  digitalWrite(MOTOR_AB2, LOW);
  digitalWrite(MOTOR_CD1, LOW);
  digitalWrite(MOTOR_CD2, LOW);
  Serial.println(F("Pins modes has been set!"));

  HardwareTimer *tim = new HardwareTimer(TIM4);
  tim->setOverflow(10, HERTZ_FORMAT);
  tim->attachInterrupt(std::bind(status_led_update, status_led, &sled_i));
  tim->resume();

  status_led[0] = true;

  Serial.println(F("Setting up ToF sensors..."));
  do {
    resetTof();
  } while (setTofAddress());
  Serial.println(F("ToF sensors setup completed!"));

  Serial.println(F("Setting up MPU6500 Sensor..."));
  setup_mpu6500();
  Serial.println(F("MPU6500 sensor setup completed!"));

  Serial.println(F("Setting up DHT11 Sensor..."));
  if (get_dhtdata() == 0) {
    status_led[6] = true;
  }
  Serial.println(F("DHT11 Ready..."));

  Serial.println(F("Setting up GPS module..."));
  if (gps.getDataGPGSA()) {
    status_led[7] = true;
  }
  Serial.println(F("GPS successfully setup"));

  Serial.println("Setting up GSM Module...");
  GSM.begin(9600);
  init_mqtt(GSM);
  setup_gsm();
  Serial.println(F("GSM Module setup completed!"));

  Serial.println(F("Initialization successfully completed!!!"));
}

void loop() {
  send_network_info();
  get_gps_data();
  get_dhtdata();
  get_mpudata();
  get_ranging_results();
  if (check_obstacle()) {
    move_vehicle();
  } else {
    apply_brake();
    Serial.println(F("There is a block"));
  }
  mqtt_msg_loop();
}
