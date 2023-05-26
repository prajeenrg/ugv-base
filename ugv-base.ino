#include <Wire.h>
#include "Adafruit_VL53L0X.h"
#include "dht11.h"
#include "FastIMU.h"
#include "Gpsneo.h"
#include "MqttClient.h"
#include "UgvDataTypes.h"

#define LESS_THAN(x, y) x < y

#define SAFE_DISTANCE_MM 40
#define BLINK_DURATION 200

/* aux pins */
#define DHT_IO PB5
#define IND_PIN PB9

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

/* Global Data Carrying Variables */
AccelData accelData;
DhtData dhtData;
LidarData lidardata;
GpsData gpsdata;
GyroData gyroData;
MotorData motordata;

/* direction settings */
enum Dir {
  NWD = 1,
  ND,
  NED,
  WD,
  STOP,
  ED,
  SWD,
  SD,
  SED,
  ROTC,
  ROTCC,
};

Gpsneo gps(GPS_TX, GPS_RX);
Adafruit_VL53L0X lox;
HardwareSerial GSM(GSM_TX, GSM_RX);
MqttClient mqtt(GSM);
MPU6500 mpu;
dht11 dht;

void setup_gsm() {
  mqtt.setup_modem();
  while (!mqtt.connect_gprs(AIRTEL_APN, AIRTEL_USER, AIRTEL_PASS)) {
    Serial.println(F("Failed initiating GPRS network"));
    Serial.println(F("Retrying GPRS establishment after 1s..."));
    delay(1000);
  }
  Serial.println("Connected to GPRS: " + String(AIRTEL_APN));
  Serial.println(F("Connecting to MQTT Broker..."));
  mqtt.set_broker(BROKER_URL, BROKER_PORT);
  mqtt.connect_broker();
  while (!mqtt.connect_broker()) {
    Serial.println(F("Failed in connecting to MQTT Broker"));
    Serial.println(F("Retrying connection after 1s..."));
    delay(1000);
  }
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
}

void getGpsData() {
  char glat[50], glong[50];
  gps.getDataGPRMC(glat, glong);
  gpsdata.latitude = gps.convertLatitude(glat);
  gpsdata.longitude = gps.convertLongitude(glong);
  mqtt.send_gps_data(gpsdata);
  Serial.printf("Latitude: %f, Longitude: %f\n", gpsdata.latitude, gpsdata.longitude);
}

void moveVehicle() {
  int mode = 0;
  motordata.left = MqttClient::control.left;
  motordata.right = MqttClient::control.right;
  if (MqttClient::control.rev) {
    analogWrite(MOTOR_AB1, LOW);
    analogWrite(MOTOR_AB2, motordata.left);
    analogWrite(MOTOR_CD1, LOW);
    analogWrite(MOTOR_CD2, motordata.right);
  } else {
    analogWrite(MOTOR_AB1, motordata.left);
    analogWrite(MOTOR_AB2, LOW);
    analogWrite(MOTOR_CD1, motordata.right);
    analogWrite(MOTOR_CD2, LOW);
  }
  mqtt.send_motor_data(motordata);

  Serial.println(F("Motor Data:"));
  Serial.printf("Left Speed: %d\n", motordata.left);
  Serial.printf("Right Speed: %d\n", motordata.right);
}

void get_dhtdata() {
  Serial.print("DHT Fetch Status: ");
  int chk = dht.read(DHT_IO);
  switch (chk) {
    case DHTLIB_OK:
      Serial.println("OK");
      dhtData.temp = dht.temperature;
      dhtData.humid = dht.humidity;
      mqtt.send_dht_data(dhtData);
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
}

void get_mpudata() {
  mpu.update();
  mpu.getAccel(&accelData);
  mpu.getGyro(&gyroData);
  mqtt.send_gyro_data(gyroData);
  mqtt.send_accel_data(accelData);

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
  digitalWrite(XSHUT_1, LOW);
  digitalWrite(XSHUT_3, LOW);
  if (!lox.begin(TOF_ADDR_4, false)) {
    Serial.println(F("LTOF cannot be found"));
    return true;
  } else {
    lox.startRangeContinuous();
  }

  digitalWrite(XSHUT_2, HIGH);
  if (!lox.begin(TOF_ADDR_2, false)) {
    Serial.println(F("RTOF cannot be found"));
    return true;
  } else {
    lox.startRangeContinuous();
  }

  digitalWrite(XSHUT_1, HIGH);
  if (!lox.begin(TOF_ADDR_1, false)) {
    Serial.println(F("FTOF cannot be found"));
    return true;
  } else {
    lox.startRangeContinuous();
  }

  digitalWrite(XSHUT_3, HIGH);
  if (!lox.begin(TOF_ADDR_3, false)) {
    Serial.println(F("BTOF cannot be found"));
    return true;
  } else {
    lox.startRangeContinuous();
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
}

void getRangingResults() {
  lox.begin(TOF_ADDR_4, false);
  lidardata.left = lox.readRange();

  lox.begin(TOF_ADDR_2, false);
  lidardata.right = lox.readRange();

  lox.begin(TOF_ADDR_1, false);
  lidardata.front = lox.readRange();

  lox.begin(TOF_ADDR_3, false);
  lidardata.back = lox.readRange();

  mqtt.send_lidar_data(lidardata);

  Serial.println(F("TOF Ranging Results:"));
  Serial.printf("TOF 1: %d\n", lidardata.front);
  Serial.printf("TOF 2: %d\n", lidardata.right);
  Serial.printf("TOF 3: %d\n", lidardata.back);
  Serial.printf("TOF 4: %d\n", lidardata.left);
}

bool checkObstacle() {
  return LESS_THAN(lidardata.front, SAFE_DISTANCE_MM) || LESS_THAN(lidardata.back, SAFE_DISTANCE_MM) || LESS_THAN(lidardata.left, SAFE_DISTANCE_MM) || LESS_THAN(lidardata.right, SAFE_DISTANCE_MM);
}

inline void blinkLed() {
  digitalWrite(IND_PIN, HIGH);
  delay(BLINK_DURATION);
  digitalWrite(IND_PIN, LOW);
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
  blinkLed();
  Serial.println(F("Pins modes has been set!"));

  Serial.println("Setting up GSM Module...");
  GSM.begin(9600);
  setup_gsm();
  blinkLed();
  Serial.println(F("GSM Module setup completed!"));

  Serial.println(F("Setting up MPU6500 Sensor..."));
  setup_mpu6500();
  blinkLed();
  Serial.println(F("MPU6500 sensor setup completed!"));

  Serial.println(F("Setting up ToF sensors..."));
  do {
    resetTof();
  } while (setTofAddress());
  blinkLed();
  Serial.println(F("ToF sensors setup completed!"));

  Serial.println(F("Initialization successfully completed!!!"));
  digitalWrite(IND_PIN, HIGH);
}

void loop() {
  getGpsData();
  get_dhtdata();
  get_mpudata();
  getRangingResults();
  if (checkObstacle()) {
    moveVehicle();
  } else {
    Serial.println(F("There is a block"));
  }
  mqtt.loop();
}
