# Tigrillo

A library to simulate and pilot the Tigrillo quadruped robot

## Requirements

TBD!

## Installation

Clone the last repository version:
```
git clone https://github.com/Gabs48/tigrillo
```

To work with an example, fist check-out the last version and install the libraries in dev mode:
```
cd tigrillo && git checkout master && sudo python setup.py develop --record installed_files.txt
```

You can remove the library with:
```
cd tigrillo && cat installed_files.txt | sudo xargs rm -rf
```

## Examples

### Simulation

Using Bullet Simulator: TBD!
Using Gazebo: TBD!
Using Mujoco: TBD!

### Run on real platform

To run the code on the robotic platform:
 - Upload the .ino project in the OpenCM board. You can test it with a serial port running a
 - Install this package on the raspberryPI3 (see above)
 - Connect the OpenCM is connected to the RPI via a USB cable (it should then appear with */dev/ttyACM0* in
 group dialout. Make sure your user is part of this group to communicate on the bus)
 - Connect the IMU on the I2C0 device. Check it with:
 ```
 sudo i2cdetect -y 1
 ```
 - Run an example that controls the actuators in open loop and record all sensors data:
 ```
 cd tigrillo/examples && python2 run_openloop_cpg.py
 ```


### Create a custom tigrillo robot model

TBD!
