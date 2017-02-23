#ifndef __RING__
#define __RING__

template<typename T, int LENGTH> class Ring {
  public:
    Ring() {
      _wp = _rp = 0;
    }

    void put(const T &dat)
    {
      int n = (_wp + 1) & (LENGTH - 1);
      if (n != _rp) {
        _buffer[_wp] = dat;
        _wp = n;
      }
    }

    T& get()
    {
      T &dat = _buffer[_rp];
      if (_wp != _rp) {
        _rp = (_rp + 1) & (LENGTH - 1);
      }
      return dat;
    }

    int count()
    {
      return (_wp + LENGTH - _rp) & (LENGTH - 1);
    }

  private:
    int _wp, _rp;
    T _buffer[LENGTH];
};

#endif
