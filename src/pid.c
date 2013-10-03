#include "pid.h"
#include "pid_time.h"

// PV - process value (sensed value)
// SP - setpoint (desired value)
// Kp - Proportianal tuning parameter
// Ki - Integral tuning parameter
// Kd - Derivative tuning parameter

PID *PID_create(
    double current_input,
    double setpoint,
    double sample_time,
    double out_min,
    double out_max,
    double Kp,
    double Ki,
    double Kd
) {
  PID *pid = calloc(1, sizeof(PID));

  check_mem(pid);

  pid->setpoint     = setpoint;
  pid->sample_time  = sample_time;
  pid->out_max      = out_max;
  pid->out_min      = out_min;
  pid->Kp           = Kp;
  pid->Ki           = Ki;
  pid->Kd           = Kd;

  pid->current_integral_value  = PID_clamp_output(pid, 0);
  pid->last_process_value      = PID_clamp_output(pid, current_input);
  pid->sample_time             = 300;

  return pid;
error:
  return NULL;
}

bool PID_should_compute(PID *pid) {
  // constrain compute to a fixed time interval
  if (getCurrentMilliseconds() - pid->last_computed < pid->sample_time) {
    return false;
  }
  return true;
}

double PID_clamp_output(PID *pid, double output) {
  if (output > pid->out_max) {
    return pid->out_max;
  } else if (output < pid->out_min) {
    return pid->out_min;
  } else {
    return output;
  }
}

void PID_tune(PID *pid, double Kp, double Ki, double Kd) {
  double sample_time = (double)(pid->sample_time / 1000);

  pid->Kp = Kp;
  pid->Ki = Ki * sample_time;
  pid->Kd = Kd / sample_time;
}

double PID_next(PID *pid, double process_value) {
  // How far off from our desired value are we?
  double error = pid->setpoint - process_value;

  // Integral values are clamped here since this is an
  // accumulator, without clamping the integer might
  // always grow beyound our otput bounds.
  pid->current_integral_value = PID_clamp_output( pid,
    pid->current_integral_value + (pid->Ki * error)
  );
  
  // Calculate (d/dt)e(t) (derivative of error over time)
  // by noting that the derivative of the error is equal to 
  // the negative of the derivative of the process_value.
  //
  // This is done to prevent "Derivative Kick", wherein the output
  // value spikes due to changes in the setpoint.
  double input_derivative = pid->last_process_value - process_value;

  // This is the final pid equation
  // This method is clamped to prevent exceeding our output bounds
  // Kp * e(t)  +  Ki * ∫ e(t) dt  +  Kd * (d/dt) e(t)
  double output = PID_clamp_output( pid,
    (pid->Kp * error) + pid->current_integral_value - (pid->Kd * input_derivative)
  );

  pid->last_process_value = process_value;
  pid->last_computed = getCurrentMilliseconds();

  return output;
}
