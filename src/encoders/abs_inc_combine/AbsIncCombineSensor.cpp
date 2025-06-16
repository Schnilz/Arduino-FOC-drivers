#include "AbsIncCombineSensor.h"
#include "common/foc_utils.h"
#include <cmath>

AbsIncCombineSensor::AbsIncCombineSensor(Sensor &_wrapped_incr,
                                         Sensor &_wrapped_abs,
                                         float transmission_ratio)
    : wrapped_incr(&_wrapped_incr), wrapped_abs(&_wrapped_abs),
      transmission_ratio(transmission_ratio), offset(0) {};

AbsIncCombineSensor::~AbsIncCombineSensor() {};

void AbsIncCombineSensor::update() {
  wrapped_incr->update();
  this->Sensor::update();
}

void AbsIncCombineSensor::init() {
  wrapped_abs->update();
  this->Sensor::init();
  float curr_abs_diff =
      fmod(wrapped_abs->getMechanicalAngle() * transmission_ratio -
               wrapped_incr->getMechanicalAngle(),
           _2PI);
  curr_abs_diff += curr_abs_diff < 0 ? _2PI : 0;
  offset = curr_abs_diff;
  update();
  full_rotations = getSensorAngle() < 0? 1:0;
  //full_rotations = wrapped_abs->getFullRotations() * transmission_ratio;
}

float AbsIncCombineSensor::getSensorAngle() {
  float ret = fmod(wrapped_incr->getMechanicalAngle() + offset, _2PI);
  return ret < 0 ? ret + _2PI : ret;
}

bool AbsIncCombineSensor::check_validity(float tolerance) {
  wrapped_abs->update();
  float difference =
      fmod(wrapped_incr->getMechanicalAngle() + offset -
               wrapped_abs->getMechanicalAngle() * transmission_ratio,
           _2PI);
  difference += (difference > _PI)    ? -_2PI
                : (difference < -_PI) ? _2PI
                                        : 0.0;
  return abs(difference) < tolerance;
}
