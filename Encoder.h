#ifndef __ENCODER__
#define __ENCODER__

class Encoder {
  public:
    Encoder(int pina, int pinb)
    {
      _pinA = pina;
      _pinB = pinb;
      _lastCount = _count = 0;
    }

    void begin()
    {
      pinMode(_pinA, INPUT);
      pinMode(_pinB, INPUT);
      _lastCode = (digitalRead(_pinB) << 1) + digitalRead(_pinA);
    }

    void poll()
    {
      static const int dir_tbl[] = { +0, +1, -1, +0,
                                     -1, +0, +0, +1,
                                     +1, +0, +0, -1,
                                     +0, -1, +1, +0,
                                   };
      int code = (digitalRead(_pinB) << 1) + digitalRead(_pinA);
      int num = dir_tbl[(_lastCode << 2) + code];
      if (num) {
        _count += num;
      }
      _lastCode = code;
    }

    int32_t count()
    {
      return _count;
    }

    int32_t delta()
    {
      int32_t d = _count - _lastCount;
      _lastCount = _count;
      return d;
    }

  private:
    int32_t _count;
    int32_t _lastCount;
    int _lastCode;
    int _pinA;
    int _pinB;
};

#endif

