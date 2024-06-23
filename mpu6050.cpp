#include "mpu6050.hpp"
#include <cmath>
#include <fstream>
#include <chrono>

constexpr double G = 9.80665;

Sensor::Sensor(const std::string &_path)
: path(_path),
  raw_value(0),
  value(0)
{}

bool Sensor::read_value()
{
  std::ifstream file;
  file.open(path, std::ios::in);

  if (true == file.good())
  {
    file >> this->raw_value;
    file.close();
    return true;
  }
  else
  {
    return false;
  }
}

double Sensor::get_value()
{
  if (true == read_value())
  {
    this->value = (this->raw_value);
  }
  else
  {
    printf("Unable to get data\n");
  }

  return this->value;
}

void mpu6050::normalize()
{
  accelerometer.x.normalized = accel[X].get_value() * G / 16384.0;
  accelerometer.y.normalized = accel[Y].get_value() * G / 16384.0;
  accelerometer.z.normalized = accel[Z].get_value() * G / 16384.0;

  gyroscope.x.normalized = gyro[X].get_value() / 32.8;
  gyroscope.y.normalized = gyro[Y].get_value() / 32.8;
  gyroscope.z.normalized = gyro[Z].get_value() / 32.8;
}


mpu6050::mpu6050()
: accelerometer({}),
  gyroscope({}),
  position({})
{
  accel.push_back(Sensor("/sys/bus/i2c/devices/3-0068/iio:device1/in_accel_x_raw"));
  accel.push_back(Sensor("/sys/bus/i2c/devices/3-0068/iio:device1/in_accel_y_raw"));
  accel.push_back(Sensor("/sys/bus/i2c/devices/3-0068/iio:device1/in_accel_z_raw"));

  gyro.push_back(Sensor("/sys/bus/i2c/devices/3-0068/iio:device1/in_anglvel_x_raw"));
  gyro.push_back(Sensor("/sys/bus/i2c/devices/3-0068/iio:device1/in_anglvel_y_raw"));
  gyro.push_back(Sensor("/sys/bus/i2c/devices/3-0068/iio:device1/in_anglvel_z_raw"));
}

void mpu6050::update()
{
  static std::chrono::high_resolution_clock::time_point lastTime = std::chrono::high_resolution_clock::now();
  std::chrono::high_resolution_clock::time_point currentTime = std::chrono::high_resolution_clock::now();
  std::chrono::duration<double, std::micro> micros = currentTime - lastTime;

  normalize();
  calculateAccelerometerAngles();
  calculateGyroscopeAngles(micros.count());

  position[X] = 0.98 * (position[X] + degrees(gyroscope.x.value)) + 0.02 * degrees(accelerometer.x.value);
  position[Y] = 0.98 * (position[Y] + degrees(gyroscope.y.value)) + 0.02 * degrees(accelerometer.y.value);
  position[Z] = 0.98 * (position[Z] + degrees(gyroscope.z.value)) + 0.02 * degrees(accelerometer.z.value);

  lastTime = currentTime;
}

void mpu6050::calculateAccelerometerAngles()
{
  accelerometer.x.value = atan(accelerometer.y.normalized / sqrt(sq(accelerometer.x.normalized) + sq(accelerometer.z.normalized)));
  accelerometer.y.value = atan(-1 * accelerometer.x.normalized / sqrt(sq(accelerometer.y.normalized) + sq(accelerometer.z.normalized)));
  accelerometer.z.value = atan2(accelerometer.y.value, accelerometer.x.value);
}

void mpu6050::calculateGyroscopeAngles(const double sampleMicros)
{
  gyroscope.x.value = gyroscope.x.normalized * sampleMicros / 1000000.0;
  gyroscope.y.value = gyroscope.y.normalized * sampleMicros / 1000000.0;
  gyroscope.z.value = gyroscope.z.normalized * sampleMicros / 1000000.0;
}

double mpu6050::sq(double x)
{
  return x * x;
}

double mpu6050::degrees(double x)
{
  return x * 180 / M_PI;
}

double mpu6050::get_X() const
{
  return this->position[X];
}

double mpu6050::get_Y() const
{
  return this->position[Y];
}

double mpu6050::get_Z() const
{
  return this->position[Z];
}

