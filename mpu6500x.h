#ifndef MPU6500X_H
#define MPU6500X_H

#include "FastIMU.h"
#include <Wire.h>

class MPU6500X : public MPU6500 {
public:
	int init(calData cal, uint8_t address, uint32_t sda, uint32_t scl);
};

#endif
