#ifndef UGVDATATYPES_H_
#define UGVDATATYPES_H_

struct GpsData {
  float latitude;
  float longitude;
};

struct MotorData {
  uint8_t left;
  uint8_t right;
};

struct LidarData {
  uint16_t front;
  uint16_t back;
  uint16_t left;
  uint16_t right;
};

#endif