#include "odrive.h"
#include "handlers.h"

int _current_state[MOTOR_NUM] = {ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION};
int _current_calibration_phase[MOTOR_NUM] = {0};
#define _CALIBRATION_STEP_NUM 3
static int _calibration_steps[_CALIBRATION_STEP_NUM] = {ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION,
                                                ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION,
                                                ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL};

void _calibrate(int motorNumber) {
  if(motorNumber > MOTOR_NUM) return;
  ODriveArduino* odr = odrives[motorNumber];
  int motornum = motors[motorNumber];
  if(!odr->run_state(motornum, _current_state[motorNumber], true)) {
//    char message[35] = "ERROR: odrive calibration timeout.";
//    rosout_msg.data = message;
//    rosout_pub.publish( &rosout_msg );
//    nh.spinOnce(); TODO
  }

  _current_state[motorNumber] = _calibration_steps[_current_calibration_phase[motorNumber]];
  delay(20);
  odr->run_state(motornum, _current_state[motorNumber], false);
  delay(20);
  _current_calibration_phase[motorNumber]++;
  if(_current_calibration_phase[motorNumber] >= _CALIBRATION_STEP_NUM) {
  _current_state[motorNumber] = _calibration_steps[0];
    _current_calibration_phase[motorNumber] = 1;
  }
}

void odrive_calibrate(int motorNumber) {
  if(motorNumber > MOTOR_NUM) return;
  ODriveArduino* odr = odrives[motorNumber];
  int motornum = motors[motorNumber];
  int requested_state;

  for(int i = 0; i < _CALIBRATION_STEP_NUM; i++) {
    _calibrate(motorNumber);
  }
  positions[motornum] = odr->read_position(motornum);
}

void odrive_set_limits(int motorNumber, int vel_limit, int current_limit) {
  if(motorNumber > MOTOR_NUM) return;
  odrives[motorNumber]->init(motors[motorNumber], vel_limit, current_limit);
}

void odrive_set_limits(int motorNumber) {
  odrives[motorNumber]->init(motors[motorNumber], ODRIVE_VEL_LIMIT, ODRIVE_CURRENT_LIMIT);
}

void odrive_init() {
  delay(500);
  for (int i = 0; i < MOTOR_NUM; i++) {
    odrive_set_limits(i);
  }
  for (int axis = 0; axis < 2; ++axis) {
    odrive_serial << "w axis" << axis << ".controller.config.vel_limit " << 22000.0f << '\n';
    odrive_serial << "w axis" << axis << ".motor.config.current_lim " << 11.0f << '\n';
    // This ends up writing something like "w axis0.motor.config.current_lim 10.0\n"
  }
//  for(int i = 0; i < _CALIBRATION_STEP_NUM - 1; i++) {
//    for(int j = 0; j < MOTOR_NUM; j++) {
//      _calibrate(j);
//    }
//  }
      int motornum = 0;
      int requested_state;

      requested_state = ODriveArduino::AXIS_STATE_MOTOR_CALIBRATION;
//      Serial << "Axis"/ << c << ": Requesting state " << requested_state << '\n';
      odrives[0]->run_state(motornum, requested_state, true);

      requested_state = ODriveArduino::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
//      Serial << "Axis" <</ c << ": Requesting state " << requested_state << '\n';
      odrives[0]->run_state(motornum, requested_state, true);

      requested_state = ODriveArduino::AXIS_STATE_CLOSED_LOOP_CONTROL;
//      Serial << "Axis" << c /<< ": Requesting state " << requested_state << '\n';
      odrives[0]->run_state(motornum, requested_state, false); // don't wait
}

void odrive_run_fixed(int motorNumber, float distance_cm) {
  if(motorNumber > MOTOR_NUM) return;
  ODriveArduino* odr = odrives[motorNumber];
  int motornum = motors[motorNumber];
  positions[motornum] += distance_cm / RAD_TO_CM;
//  odr->SetTrajectory(motornum, positions[motornum]);
  odr->SetPosition(motornum, positions[motornum]);
}

void odrive_run_continuous(int motorNumber, float distance_cm) {
  if(motorNumber > MOTOR_NUM) return;
  ODriveArduino* odr = odrives[motorNumber];
  int motornum = motors[motorNumber];
  positions[motornum] += distance_cm / RAD_TO_CM;
  odr->SetPosition(motornum, positions[motornum]);
}

void odrive_drive_fixed(float distance, float angular_distance) {
  float radius = distance / angular_distance;
  float radiusL = radius + TRACKS_WIDTH / 2;
  float radiusR = radius - TRACKS_WIDTH / 2;
  float distanceL = radiusL * angular_distance;
  float distanceR = radiusR * angular_distance;
  odrive_run_fixed(MOTOR_LEFT, distanceL * MOTOR_LEFT_DIR);
  odrive_run_fixed(MOTOR_RIGHT, distanceR * MOTOR_RIGHT_DIR);
}
