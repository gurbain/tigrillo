/* Tigrillo_openCM

  This script implements the basic functions to interface sensors and actuators over the UART 
  so that they can be used on another board. No major computation or logi is done here.
  
  Created on September 11th 2017
  
  Gabriel Urbain <gabriel.urbain@ugent.be>
  Copyright 2017, Human Brain Projet, SP10
 */
 
#include <OLLO.h>
#define SENSOR_TIMER_PERIOD 100000 // microseconds

OLLO myOLLO;
HardwareTimer Timer(1);

// Sensors varialbes
long sens_timestamp = 0;
long sens_foot_fl_zero = 0;
long sens_foot_fr_zero = 0;
long sens_foot_bl_zero = 0;
long sens_foot_br_zero = 0;

// Actuators variables
long act_timestamp = 0;
long act_leg_fl = 0;
long act_leg_fr = 0;
long act_leg_bl = 0;
long act_leg_br = 0;


void setup() {
  
  myOLLO.begin(1);

  // Setup Timer 1 for periodic sensor reading
  Timer.pause();
  Timer.setPeriod(SENSOR_TIMER_PERIOD);
  Timer.setMode(TIMER_CH1, TIMER_OUTPUT_COMPARE);
  Timer.setCompare(TIMER_CH1, 1);
  Timer.attachInterrupt(TIMER_CH1, sensorsLoop);
  Timer.refresh();
  Timer.resume();
  
  // Setup Interuption for received UART message
  SerialUSB.begin()
  SerialUSB.attachInterrupt(readUART);
  
}


void loop() {
  
  // Nothing to do in the main loop

}


void readUART(byte* buffer, byte nCount) {
  
  SerialUSB.print("nCount =");
  SerialUSB.println(nCount);
  for(unsigned int i=0; i < nCount;i++)
    SerialUSB.print((char)buffer[i]);
  SerialUSB.println("");
  
}


void readSensors(void) {
  long start_time = 0;
  long interval_time = 0;
  int DMS_value = 0;
  int tmp =0;
  
  // Get time and check with previous timestamp
  current_timestamp = micros();
  interval_time = current_timestamp - sens_timestamp;
  sens_timestamp = current_timestamp;
  
  // Get sensors values
  DMS_value = myOLLO.read(1);
  
  // Publish everything over UART
  SerialUSB.print("[INTERVAL=");
  SerialUSB.print(SENSOR_TIMER_PERIOD); 
  SerialUSB.print("] "); 
  SerialUSB.print("DMS=");
  SerialUSB.print(DMS_value); 
  SerialUSB.print(", start_time=");
  SerialUSB.print(current_timestamp);
  SerialUSB.print(", interval_time=");
  SerialUSB.print(interval_time);
  SerialUSB.print(", exec_time=");
  SerialUSB.println(micros()- start_time);
 
}


void resetSensors(void) {

  DMS_value = myOLLO.read(1);
  
  long sens_foot_fl_zero = DMS_value[0];
  long sens_foot_fr_zero = DMS_value[1];
  long sens_foot_bl_zero = DMS_value[2];
  long sens_foot_br_zero = DMS_value[3];
  
}


void updateMotors(void) {
  
}


