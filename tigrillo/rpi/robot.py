
"""
This file contains all methods related to the robot sensors
"""

from imu import BNO055
from uart import *
from tigrillo.core.utils import *

import csv

__author__ = "Gabriel Urbain" 
__copyright__ = "Copyright 2017, Human Brain Projet, SP10"

__license__ = "MIT" 
__version__ = "1.0" 
__maintainer__ = "Gabriel Urbain"
__email__ = "gabriel.urbain@ugent.be" 
__status__ = "Research" 
__date__ = "September 11th, 2017"


DATA_FOLDER = "data"


class Sensors:
    """
    This class can be called to read the sensors in a formatted representation
    """

    def __init__(self, uart, i2c_rst_pin):

        self.i2c_rst_pin = i2c_rst_pin
        self.uart = uart
        self.imu = None

    def start(self):
        """ Start the sensors and print information """

        self.imu = BNO055.BNO055(rst=int(self.i2c_rst_pin))

        if not self.imu.begin():
            raise RuntimeError('Failed to initialize IMU! Please, check the connection')
        else:
            print("Sensors initialized correctly!\n")
            self.printStatus()

        return

    def printStatus(self):
        """ Print sensors status """

        status, self_test, error = self.imu.get_system_status()
        sw, bl, acc, mag, gyro = self.imu.get_revision()
        cal_sys, cal_gyro, cal_acc, cal_mag = self.imu.get_calibration_status()

        print('IMU STATUS\n')

        print('\tSystem status: {0}'.format(status))
        print('\tSelf test result (0x0F is normal): 0x{0:02X}'.format(self_test))
        if status == 0x01:
            print('\tSystem error: {0}'.format(error))
            print('\tSee datasheet section 4.3.59 for the meaning.')
        print('\tSoftware version:   {0}'.format(sw))
        print('\tBootloader version: {0}'.format(bl))
        print('\tAccelerometer ID:   0x{0:02X}'.format(acc))
        print('\tMagnetometer ID:    0x{0:02X}'.format(mag))
        print('\tGyroscope ID:       0x{0:02X}\n'.format(gyro))
        print('\tCalibration System status:          {0}'.format(cal_sys))
        print('\tCalibration Accelerometers status:  0x{0:02X}'.format(cal_acc))
        print('\tCalibration Magnetometer status:    0x{0:02X}'.format(cal_mag))
        print('\tCalibration Gyroscope status:       0x{0:02X}\n'.format(cal_gyro))

    def changeUARTPeriod(self, period):
        """ Change the period of the scheduler that sends sensor values on the UART. Be carefull: the function
        getMeasure can be called asynchronously and will always return the last values with a frequency imposed
        by the user """

        line = "F"
        line += str(period)
        self.uart.write(line)
        # CHECKACK

    def resetCalib(self):
        """ Reset the sensor calibration to the current value """

        # IMU
        data = self.imu.get_calibration()
        self.imu.set_calibration(data)

        # OpenCM
        line = "R"
        self.uart.write(line)

    def getMeasureAll(self):
        """ Return a dictionnary with all the last iteration of sensors measurement """

        # IMU
        imu_timestamp = time.time()
        ori_h, ori_r, ori_p = self.imu.read_euler()
        pos_x, pos_y, pos_z, pos_w = self.imu.read_quaternion()
        temp_c = self.imu.read_temp()
        mag_x, mag_y, mag_z = self.imu.read_magnetometer()
        gyro_x, gyro_y, gyro_z = self.imu.read_gyroscope()
        acc_x, acc_y, acc_z = self.imu.read_accelerometer()
        linacc_x, linacc_y, linacc_z = self.imu.read_linear_acceleration()
        grav_x, grav_y, grav_z = self.imu.read_gravity()
        imu_read_time = time.time() - imu_timestamp

        # OpenCM
        opencm_line = self.uart.readData()
        if opencm_line:
            splitted_line = opencm_line.split(";:\n\r")
            print(splitted_line)
        else:
            print("Cannot retrieve sensor data! Please increase sensor reading period!")

        measure = {"IMU Time Stamp": imu_timestamp, "IMU IO Time": imu_read_time, "Position Heading": ori_h,
                    "Position Roll": ori_r, "Position Pitch": ori_p, "Position X": pos_x, "Position Y": pos_y,
                    "Position Z": pos_z, "Position W": pos_w, "IMU Temperature": temp_c, "Magnetometer X": mag_x,
                    "Magnetometer Y": mag_y, "Magnetometer Z": mag_z, "Gyroscope X": gyro_x, "Gyroscope Y": gyro_y,
                    "Gyroscope Z": gyro_z, "Acceleration X": acc_x, "Acceleration Y": acc_y, "Acceleration Z": acc_z,
                    "Linear Acceleration X": linacc_x, "Linear Acceleration Y": linacc_y,
                    "Linear Acceleration Z": linacc_z, "Gravity X": grav_x, "Gravity Y": grav_y, "Gravity Z": grav_z}
        return measure

    def getMeasure(self):
        """ Return a dictionnary with the essential sensors measurements """

        # IMU
        imu_timestamp = time.time()
        ori_h, ori_r, ori_p = self.imu.read_euler()
        pos_x, pos_y, pos_z, pos_w = self.imu.read_quaternion()
        gyro_x, gyro_y, gyro_z = self.imu.read_gyroscope()
        linacc_x, linacc_y, linacc_z = self.imu.read_linear_acceleration()
        grav_x, grav_y, grav_z = self.imu.read_gravity()
        imu_read_time = time.time() - imu_timestamp


        measure = {"IMU Time Stamp": imu_timestamp, "IMU IO Time": imu_read_time, "Position Heading": ori_h,
                   "Position Roll": ori_r, "Position Pitch": ori_p,"Position X": pos_x, "Position Y": pos_y,
                   "Position Z": pos_z, "Position W": pos_w, "Gyroscope X": gyro_x, "Gyroscope Y": gyro_y,
                   "Gyroscope Z": gyro_z, "Linear Acceleration X": linacc_x, "Linear Acceleration Y": linacc_y,
                   "Linear Acceleration Z": linacc_z, "Gravity Y": grav_y, "Gravity Z": grav_z}

        # OpenCM
        opencm_measure = self.uart.read_data()
        if opencm_measure:
            if ("UART Time Stamp" and "UART Loop Time" and "UART IO Time") in opencm_measure:
                measure.update({"UART Time Stamp": opencm_measure["UART Time Stamp"],
                                "UART Loop Time": opencm_measure["UART Loop Time"],
                                "UART IO Time": opencm_measure["UART IO Time"]})
            if "Sensors values" in opencm_measure:
                measure.update(opencm_measure["Sensors values"])
        else:
            print("Cannot retrieve UART sensor data! Please increase sensor reading period!")

        return measure


class Actuators:
    """
    This class can be called to read the sensors in a formatted representation
    """

    def __init__(self, uart):

        self.uart = uart

    def start(self):
        """ Start the actuators and print information """

        print("Actuators initialized correctly! Nothing to do here.\n")

    def update(self, update):
        """ Send a command to the platform trhough the UART. NB: values should be included in [0, 1000] """

        line = "A"
        line += str(int(update["FL"])) + ','
        line += str(int(update["FR"])) + ','
        line += str(int(update["BL"])) + ','
        line += str(int(update["BR"]))
        self.uart.write(line)


class Tigrillo:
    """
    This class can be called to read the sensors in a formatted representation
    """

    def __init__(self, uart_port="/dev/ttyACM0", uart_baud=9600, i2c_rst_pin="5", save_all=True, data_folder=None):

        self.uart_baud = uart_baud
        self.uart_port = uart_port
        self.i2c_rst_pin = i2c_rst_pin
        self.save_all = save_all
        self.data_folder = data_folder

        self.uart = None
        self.sensors = None
        self.sensors_index = 0
        self.actuators = None
        self.actuators_index = 0

        if save_all:
            if data_folder is None:
                data_folder = DATA_FOLDER
            mkdir(data_folder)
            self.file_s = data_folder + "/sensors_" + timestamp() + ".csv"
            self.file_a = data_folder + "/actuators_" + timestamp() + ".csv"

    def start(self):
        """ Initialize all communication channels and setup sensors and actuators config in the robot """

        # Setup and start uart daemon
        self.uart = UARTDaemon(self.uart_port, self.uart_baud)
        self.uart.start()

        self.sensors = Sensors(self.uart, self.i2c_rst_pin)
        self.actuators = Actuators(self.uart)
        self.sensors.start()
        self.actuators.start()

        return

    def reset_sensors_calib(self):
        """ Zero all sensors to their current values """

        self.sensors.resetCalib()

        return

    def get_last_sensors(self, t_run):
        """ Get a full measurement from the robot sensors """

        measure = self.sensors.getMeasure()

        if self.save_all:
            measure["Run Time"] = t_run
            with open(self.file_s, 'a') as f:
                w = csv.DictWriter(f, sorted(measure.keys()))
                if self.sensors_index == 0:
                    w.writeheader()
                w.writerow(measure)

        self.sensors_index += 1
        return measure

    def update_actuators(self, update, t_run):
        """ Send a update to the robot actuators """

        self.actuators.update(update)

        if self.save_all:
            update["Run Time"] = t_run
            with open(self.file_a, 'a') as f:
                w = csv.DictWriter(f, sorted(update.keys()))
                if self.actuators_index == 0:
                    w.writeheader()
                w.writerow(update)

        self.actuators_index += 1
        return

    def set_sensor_period(self, period=0.01):
        """ Set the period to which the OpenCM board is reading all sensors (in s) """

        print period * 1000000
        self.sensors.changeUARTPeriod(period * 1000000)
