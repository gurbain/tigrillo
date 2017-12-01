# Tigrillo

A library to simulate and pilot the Tigrillo quadruped robot

## Requirements

ROS
mpi4py
Known issues:

NRP installed => need to install mpi4py manually using the mpi version from the NRP
- remove other mpi4py versions
- clone rep
- change config
- build
- install

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
Install the models
Using Mujoco: TBD!

Known issues:
Adapt the configuration files:
find . -type f -print0 | xargs -0 sed -i 's/gabs48/gurbain/g'

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


## TO ADD to installation RPI
- sudo apt-get install libfreetype6-dev libpng12-dev libcr-dev mpich mpich-doc libopenmpi-dev liblapack-dev libblas-dev
- Rmq: had to install mpi4py with pip
- Attention develop
- correct OpenCM install
- adafruit install: https://github.com/adafruit/Adafruit_Python_GPIO

