LSM9DS1
=======

![](https://cloud.githubusercontent.com/assets/6698410/5024295/01c82314-6abd-11e4-8a9f-748d0687ca20.jpg)

ST's new smaller, lower-power 9-axis motion sensor

The LSM9DS1 is a high-resolution (16-bit) 9-axis motion sensor (accelerometer, gyroscope, and magnetometer) in a 3 mm x 3.5 mm LGA 28 pin package. Coupled with the fine 24-bit Measurement Specialties MS5611 altimeter, the Teensy 3.1 add-on/breakout board offers 10 degrees of freedom in a compact size for many motion control applications. 

Breakout boards are for sale at [Tindie.com](https://www.tindie.com/products/onehorse/lsm9ds1-ms5611-breakout-board/).

This is a basic Arduino (Teensiduino) sketch that parametrizes the registers, initializes the sensor configurations, performs the accel and gyro self-tests, calibrates the sensors, and uses the scaled output to obtain yaw, pitch, roll, and quaternions with open-source Madgwick and Mahony sensor fusion filters.
