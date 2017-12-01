#ifndef _TIGRILLO_OPENCM_H_
#define _TIGRILLO_OPENCM_H_

/* -------- Custom Processing Functions -------- */

void readUART(byte* buffer, byte size);

void readSensors(void);

void resetSensors(void);

void updateMotors(int act_leg[]);

int changePeriod(int period);

/* -------- Custom Utils Functions -------- */

void printTab(int* tab, int size);

void initTab(int* tab, int size);

void emptyStr(char* tab, int size);

int str2i(char* str, int* tab, int size);

void wait4Bus(void);

void freeBus(void);

#endif