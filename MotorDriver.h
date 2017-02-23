#ifndef __MOTORDRIVER__
#define __MOTORDRIVER__

#include "PWM.h"

class MotorDriver {
  public:
    MotorDriver(int forward_pin, int reverse_pin, int pwm_pin)
    {
      _forwardPin = forward_pin;
      _reversePin = reverse_pin;
      _pwmPin = pwm_pin;
    }

    void begin()
    {
      pwmMode(_pwmPin, 25000, 10);
      pinMode(_forwardPin, OUTPUT);
      pinMode(_reversePin, OUTPUT);
    }

    void go(int32_t duty) {
      if (duty < 0) {
        digitalWrite(_forwardPin, LOW);
        digitalWrite(_reversePin, HIGH);
        duty = -duty;
      } else {
        digitalWrite(_forwardPin, HIGH);
        digitalWrite(_reversePin, LOW);
      }
      pwmWrite(_pwmPin, duty);
    }

    void forward(uint32_t duty) {
      digitalWrite(_forwardPin, HIGH);
      digitalWrite(_reversePin, LOW);
      pwmWrite(_pwmPin, duty);
    }

    void reverse(uint32_t duty) {
      digitalWrite(_forwardPin, LOW);
      digitalWrite(_reversePin, HIGH);
      pwmWrite(_pwmPin, duty);
    }

    void stop(void) {
      digitalWrite(_forwardPin, LOW);
      digitalWrite(_reversePin, LOW);
      pwmWrite(_pwmPin, 0);
    }

    void brake(void) {
      digitalWrite(_forwardPin, HIGH);
      digitalWrite(_reversePin, HIGH);
      pwmWrite(_pwmPin, 0);
    }

  private:
    int _forwardPin, _reversePin, _pwmPin;
};

#endif

