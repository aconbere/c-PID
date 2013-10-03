#ifndef PID_h
#define PID_h

#include "dbg.h"
#include <stdlib.h>
#include <stdio.h>

typedef int bool;
enum { false, true };

typedef struct PID {
  unsigned long last_computed;
  double setpoint;

  double out_min;
  double out_max;

  double current_integral_value;
  double last_process_value;
  double sample_time;

  double Kp;
  double Ki;
  double Kd;
} PID;

// PV - process value (sensed value)
// SP - setpoint (desired value)
// Kp - Proportianal tuning parameter
// Ki - Integral tuning parameter
// Kd - Derivative tuning parameter

PID *PID_create(
    double current_output,
    double current_input,
    double setpoint,
    double out_min,
    double out_max,
    double sample_time,
    double Kp,
    double Ki,
    double Kd
);

void PID_tune(PID *pid, double Kp, double Ki, double Kd);
bool PID_should_compute(PID *pid);
double PID_clamp_output(PID *pid, double output);
double PID_next(PID *pid, double process_value);

#endif
