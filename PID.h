#ifndef __PID__
#define __PID__

#include <float.h>

class PID
{
  public:
    PID() {
      _Kp = 1.f;
      _Ki = 0.f;
      _Kd = 0.f;
      _dt = 0.1f;
      _integral = _previous_output = _previous_error = 0;
      _lowerLimit = FLT_MIN;
      _upperLimit = FLT_MAX;
    }

    void setGain(float kp, float ki, float kd) {
      _Kp = kp;
      _Ki = ki;
      _Kd = kd;
    }

    void setInterval(float sec) {
      _dt = sec;
    }

    void setOutputLimits(float lowerLimit, float upperLimit)
    {
      _lowerLimit = lowerLimit;
      _upperLimit = upperLimit;
    }

    // PID controller - Wikipedia
    // https://en.wikipedia.org/wiki/PID_controller#Pseudocode

    float process(float error)
    {
      float output, derivative;
      if (_lowerLimit < _previous_output && _previous_output < _upperLimit) {
        _integral = _integral + error * _dt;
      }
      derivative = (error - _previous_error) / _dt;
      output = (_Kp * error) + (_Ki * _integral) + (_Kd * derivative);
      _previous_error = error;
      _previous_output = output;
      return constrain(output, _lowerLimit, _upperLimit);
    }

    void reset()
    {
      _integral = _previous_output = _previous_error = 0.f;
    }

  public:
    float _previous_output;
    float _integral;

  private:
    float _Kp, _Ki, _Kd, _dt;
    float _previous_error;
    float _lowerLimit, _upperLimit;
};

#endif

