#ifndef UGVDATATYPES_H_
#define UGVDATATYPES_H_

struct GpsData {
  float latitude;
  float longitude;
};

struct LidarData {
  uint16_t front;
  uint16_t back;
  uint16_t left;
  uint16_t right;
};

struct DhtData {
  double temp;
  double humid;
};

struct Control {
  uint8_t left;
  uint8_t right;
  bool rev;
  bool check_left;
  bool check_right;
  bool check_front;
  bool check_back;
};

#endif