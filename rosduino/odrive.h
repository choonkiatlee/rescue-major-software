#include "z_odrive.h"
#include "globals.h"


void odrive_calibrate(int motorNumber);
void odrive_init();
void odrive_run_fixed(int motorNumber, float distance_cm);
void odrive_run_continuous(int motorNumber, float distance_cm);
void odrive_set_limits(int motorNum, int vel_limit, int current_limit);
void odrive_set_limits(int motorNum);
void odrive_drive_fixed(float distance, float angular_distanc);
