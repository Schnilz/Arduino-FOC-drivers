# abs_inc_combine sensor

This sensor wrapper is able to combine an incremental sensor (with Quadrature / AB interface) and an absolut Sensor (e.g. a MagneticSensor).
This is useful if you have an incremental encoder for its good FOC performance and an absolute encoder after a gear box to skip initialization / calibration.

The absolute encoder is only read on init (and optional validity checks) after which the incremental sensor is used.