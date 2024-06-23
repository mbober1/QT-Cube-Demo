#pragma once
#include <string>
#include <vector>

enum class SensorType
{
  ACCEL,
  GYRO
};

enum SensorAxis
{
  X,
  Y,
  Z,
};

class Sensor
{
private:
  std::string path;
  int raw_value;
  double value;
  bool read_value();

public:
  Sensor(const std::string &path);

  double get_value();
};

struct angle_data
{
  double normalized;
  double value;
};

struct sensor_data
{
  angle_data x;
  angle_data y;
  angle_data z;
};

class mpu6050
{
private:
  sensor_data accelerometer;
  sensor_data gyroscope;
  double position[3];

  std::vector<Sensor> accel;
  std::vector<Sensor> gyro;
  void normalize();
  double sq(double x);
  double degrees(double x);

  void calculateAccelerometerAngles();
  void calculateGyroscopeAngles(const double sampleMicros);

public:
  mpu6050();

  void update();
  double get_X() const;
  double get_Y() const;
  double get_Z() const;
};
