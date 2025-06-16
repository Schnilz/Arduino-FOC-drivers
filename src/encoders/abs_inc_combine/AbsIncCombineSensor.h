#ifndef __ABSINCCOMBINESENSOR_H__
#define __ABSINCCOMBINESENSOR_H__

#include "common/base_classes/Sensor.h"
#include "common/foc_utils.h"

class AbsIncCombineSensor : public Sensor {

public:
  /**
   * @brief Constructor of class with pointer to base class sensor and driver
   * @param wrapped_incr the wrapped incremental sensor which needs calibration
   * @param wrapped_abs the wrapped absolute sensor which needs calibration
   * @param transmission_ratio the transmission ratio of both sensors. Gets
   * applied (by devision) to the readings from wrapped_abs
   */
  AbsIncCombineSensor(Sensor &wrapped_incr, Sensor &wrapped_abs,
                      const float transmission_ratio = 1.0f);

  ~AbsIncCombineSensor();

  virtual void update() override;
  virtual void init() override;

  bool check_validity(float tolerance = _2PI / 180.0f); 

  virtual float getSensorAngle() override;
  Sensor *const wrapped_incr;
  Sensor *const wrapped_abs;
  const float transmission_ratio;
  float offset;
  protected:
};

#endif /* __ABSINCCOMBINESENSOR_H__ */
