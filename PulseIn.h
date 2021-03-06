#ifndef __PULSEIN__
#define __PULSEIN__

#include "wiring_private.h"

#define CAPTURE_TCC ((Tcc*)TCC1)
#define CAPTURE_CLKCTRL_ID (GCLK_CLKCTRL_ID_TCC0_TCC1)
#define CAPTURE_EVSYS_ID_USER (EVSYS_ID_USER_TCC1_MC_0)
#define CAPTURE_IRQn TCC1_IRQn
#define CAPTURE_IRQ_Handler TCC1_Handler

static void input_capture_init(void)
{
  static bool _input_capture_initialized = false;
  if (!_input_capture_initialized) {

    // Clock EIC for I/O interrupts
    PM->APBAMASK.reg |= PM_APBAMASK_EIC ;

    // Enable GCLK for IEC (External Interrupt Controller)
    GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_ID(GCM_EIC));
    while (GCLK->STATUS.bit.SYNCBUSY == 1);

    // Enable EIC
    EIC->CTRL.bit.ENABLE = 1;
    while (EIC->STATUS.bit.SYNCBUSY == 1);

    // Clock EVSYS for I/O interrupts
    PM->APBCMASK.reg |= PM_APBCMASK_EVSYS;

    // Clock TCCx
    GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | CAPTURE_CLKCTRL_ID);
    while (GCLK->STATUS.bit.SYNCBUSY == 1);

    // Disable CAPTURE_TCC
    CAPTURE_TCC->CTRLA.bit.ENABLE = 0;
    while (CAPTURE_TCC->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);

    // Enable CPTEN0/CPTEN1, Set prescaler to 1/64
    CAPTURE_TCC->CTRLA.reg = TCC_CTRLA_CPTEN0 | TCC_CTRLA_CPTEN1 | TCC_CTRLA_PRESCALER_DIV64;
    while (CAPTURE_TCC->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);

    // Enable Capture Channel Event Input
    CAPTURE_TCC->EVCTRL.bit.MCEI0 = 1;
    CAPTURE_TCC->EVCTRL.bit.MCEI1 = 1;

    CAPTURE_TCC->INTENSET.bit.MC0 = 1;
    CAPTURE_TCC->INTENSET.bit.MC1 = 1;

    NVIC_EnableIRQ(CAPTURE_IRQn);

    // Enable CAPTURE_TCC
    CAPTURE_TCC->CTRLA.bit.ENABLE = 1;
    while (CAPTURE_TCC->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);

    _input_capture_initialized = true;
  }
}

static void input_capture_attach(int channel, int pin)
{
  EExt_Interrupts in = g_APinDescription[pin].ulExtInt;

  if (in != NOT_AN_INTERRUPT)
  {
    // Enable Event Output
    EIC->EVCTRL.reg |= EIC_EVCTRL_EXTINTEO(1 << in);

    // Assign pin to EIC
    pinPeripheral(pin, PIO_EXTINT);

    // Configure the interrupt mode
    int cfg = in >> 3;
    int pos = (in & 0x7) << 2;
    EIC->CONFIG[cfg].reg |= (EIC_CONFIG_SENSE0_RISE_Val | 0x8) << pos;
    //    Serial.println(cfg);
    //    Serial.println(pos);
    //    Serial.println(EIC->CONFIG[cfg].reg, HEX);

    // Attach the event user to channel
    EVSYS->USER.reg = EVSYS_USER_CHANNEL(channel + 1) |
                      EVSYS_USER_USER(CAPTURE_EVSYS_ID_USER + channel);

    EVSYS->CHANNEL.reg = EVSYS_CHANNEL_EDGSEL_NO_EVT_OUTPUT |
                         EVSYS_CHANNEL_PATH_ASYNCHRONOUS |
                         EVSYS_CHANNEL_EVGEN(EVSYS_ID_GEN_EIC_EXTINT_0 + in) |
                         EVSYS_CHANNEL_CHANNEL(channel);
  }
}

static uint32_t input_capture_read(int channel)
{
  while (CAPTURE_TCC->SYNCBUSY.reg & (TCC_SYNCBUSY_CC0 | TCC_SYNCBUSY_CC1));
  return CAPTURE_TCC->CC[channel].reg;
}

//static int input_capture_valid(int channel)
//{
//  return (CAPTURE_TCC->STATUS.reg & TCC_STATUS_CCBV(1 << channel));
//}

class PulseIn
{
  public:
    PulseIn(int pin_a, int pin_b)
    {
      _pinA = pin_a;
      _pinB = pin_b;
    }

    void begin()
    {
      _lastTime = 0;
      _pulseWidth = _err = 0;
      if (!_instance[0]) {
        _channel = 0;
        _instance[0] = this;
      } else if (!_instance[1]) {
        _channel = 1;
        _instance[1] = this;
      }
      input_capture_init();
      input_capture_attach(_channel, _pinA);
    }

    void end()
    {
    }

    void poll()
    {
      uint32_t t = input_capture_read(_channel);
      _pulseWidth = (t - _lastTime) & 0xffffff;
      _lastTime = t;
    }

    uint32_t pulseWidth()
    {
      return (_pulseWidth * 100) / 75;
    }

    uint32_t pulsePerSecond()
    {
      if (_pulseWidth) {
        return 750000 / _pulseWidth;
      }
      return 0;
    }

    int error()
    {
      return _err;
    }

    void reset()
    {
      _lastTime = 0;
      _pulseWidth = _err = 0;
    }

    static PulseIn *_instance[2];
  private:
    int _pinA, _pinB;
    int _channel;
    int _err;
    uint32_t _lastTime;
    uint32_t _pulseWidth;
};

PulseIn *PulseIn::_instance[2] = { 0, 0 };

void CAPTURE_IRQ_Handler(void)
{
  if (CAPTURE_TCC->INTFLAG.bit.MC0) {
    if (PulseIn::_instance[0])
      PulseIn::_instance[0]->poll();
  }
  if (CAPTURE_TCC->INTFLAG.bit.MC1) {
    if (PulseIn::_instance[1])
      PulseIn::_instance[1]->poll();
  }
}

#endif

