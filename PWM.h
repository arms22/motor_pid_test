#ifndef __PWM__
#define __PWM__

#include "wiring_private.h"

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTC_8(Tc* TCx) __attribute__((always_inline, unused));
static void syncTC_8(Tc* TCx) {
  while (TCx->COUNT8.STATUS.bit.SYNCBUSY);
}

// Wait for synchronization of registers between the clock domains
static __inline__ void syncTCC(Tcc* TCCx) __attribute__((always_inline, unused));
static void syncTCC(Tcc* TCCx) {
  while (TCCx->SYNCBUSY.reg & TCC_SYNCBUSY_MASK);
}

#define GCLK0_FREQ (48000000)

typedef struct _TcSetting {
  bool enabled;
  int frequency;
  int resolution;
  uint32_t divider;
  uint32_t period;
} TcSetting;

static TcSetting tcSettings[TCC_INST_NUM + TC_INST_NUM];

static void dividerAndPeriod(TcSetting &tcs, int max_reso)
{
  static const uint32_t div_N[8] = {1, 2, 4, 8, 16, 64, 256, 1024};
  uint32_t reso = 1 << max_reso;
  int i;
  for (i = 0; i < 8; i++) {
    uint32_t per = GCLK0_FREQ / div_N[i] / tcs.frequency;
    tcs.divider = i;
    tcs.period = per - 1;
    if (per <= reso) {
      break;
    }
  }
}

static inline uint32_t mapResolution(uint32_t value, uint32_t reso, uint32_t period)
{
  uint32_t maxval = (1 << reso) - 1;
  if (value == maxval) return period;
  return (value * period) / maxval;
}

void pwmMode(int pin, int freq, int res = 8)
{
  PinDescription pinDesc = g_APinDescription[pin];
  uint32_t attr = pinDesc.ulPinAttribute;
  if ((attr & PIN_ATTR_PWM) == PIN_ATTR_PWM)
  {
    uint32_t tcNum = GetTCNumber(pinDesc.ulPWMChannel);
    TcSetting &tcs = tcSettings[tcNum];
    tcs.frequency = freq;
    tcs.resolution = res;
  }
}

void pwmWrite(int pin, uint32_t value)
{
  const PinDescription &pinDesc = g_APinDescription[pin];
  uint32_t attr = pinDesc.ulPinAttribute;

  if ((attr & PIN_ATTR_PWM) == PIN_ATTR_PWM)
  {
    uint32_t tcNum = GetTCNumber(pinDesc.ulPWMChannel);
    uint8_t tcChannel = GetTCChannelNumber(pinDesc.ulPWMChannel);
    TcSetting &tcs = tcSettings[tcNum];

    if (attr & PIN_ATTR_TIMER) {
#if !(ARDUINO_SAMD_VARIANT_COMPLIANCE >= 10603)
      // Compatibility for cores based on SAMD core <=1.6.2
      if (pinDesc.ulPinType == PIO_TIMER_ALT) {
        pinPeripheral(pin, PIO_TIMER_ALT);
      } else
#endif
      {
        pinPeripheral(pin, PIO_TIMER);
      }
    } else {
      // We suppose that attr has PIN_ATTR_TIMER_ALT bit set...
      pinPeripheral(pin, PIO_TIMER_ALT);
    }

    if (!tcs.enabled) {
      tcs.enabled = true;

      static const uint16_t GCLK_CLKCTRL_IDs[] = {
        GCLK_CLKCTRL_ID(GCM_TCC0_TCC1), // TCC0
        GCLK_CLKCTRL_ID(GCM_TCC0_TCC1), // TCC1
        GCLK_CLKCTRL_ID(GCM_TCC2_TC3),  // TCC2
        GCLK_CLKCTRL_ID(GCM_TCC2_TC3),  // TC3
        GCLK_CLKCTRL_ID(GCM_TC4_TC5),   // TC4
        GCLK_CLKCTRL_ID(GCM_TC4_TC5),   // TC5
        GCLK_CLKCTRL_ID(GCM_TC6_TC7),   // TC6
        GCLK_CLKCTRL_ID(GCM_TC6_TC7),   // TC7
      };
      GCLK->CLKCTRL.reg = (uint16_t) (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | GCLK_CLKCTRL_IDs[tcNum]);
      while (GCLK->STATUS.bit.SYNCBUSY == 1);

      // Set PORT
      if (tcNum >= TCC_INST_NUM) {
        dividerAndPeriod(tcs, 8);
        value = mapResolution(value, tcs.resolution, tcs.period);

        // -- Configure TC
        Tc* TCx = (Tc*) GetTC(pinDesc.ulPWMChannel);
        // Disable TCx
        TCx->COUNT8.CTRLA.bit.ENABLE = 0;
        syncTC_8(TCx);
        // Set Timer counter Mode to 8 bits, normal PWM, prescaler 1/tcs.divider
        TCx->COUNT8.CTRLA.reg &= ~(TC_CTRLA_MODE_Msk | TC_CTRLA_WAVEGEN_Msk | TC_CTRLA_PRESCALER_Msk);
        TCx->COUNT8.CTRLA.reg |= TC_CTRLA_MODE_COUNT8 | TC_CTRLA_WAVEGEN_NPWM | (tcs.divider << TC_CTRLA_PRESCALER_Pos);
        syncTC_8(TCx);
        // Set the initial value
        TCx->COUNT8.CC[tcChannel].reg = (uint8_t) value;
        syncTC_8(TCx);
        // Set PER to maximum counter value (resolution : 0xFF)
        TCx->COUNT8.PER.reg = tcs.period;
        syncTC_8(TCx);
        // Enable TCx
        TCx->COUNT8.CTRLA.bit.ENABLE = 1;
        syncTC_8(TCx);
      } else {
        dividerAndPeriod(tcs, 24);
        value = mapResolution(value, tcs.resolution, tcs.period);

        // -- Configure TCC
        Tcc* TCCx = (Tcc*) GetTC(pinDesc.ulPWMChannel);
        // Disable TCCx
        TCCx->CTRLA.bit.ENABLE = 0;
        syncTCC(TCCx);
        // Set prescaler to 1/DIV
        TCCx->CTRLA.reg &= ~TCC_CTRLA_PRESCALER_Msk;
        TCCx->CTRLA.reg |= (tcs.divider << TCC_CTRLA_PRESCALER_Pos);
        syncTCC(TCCx);
        // Set TCx as normal PWM
        TCCx->WAVE.reg &= ~TCC_WAVE_WAVEGEN_Msk;
        TCCx->WAVE.reg |= TCC_WAVE_WAVEGEN_NPWM;
        syncTCC(TCCx);
        // Set the initial value
        TCCx->CC[tcChannel].reg = (uint32_t) value;
        syncTCC(TCCx);
        // Set PER to maximum counter value (resolution : 0xFF)
        TCCx->PER.reg = tcs.period;
        syncTCC(TCCx);
        // Enable TCCx
        TCCx->CTRLA.bit.ENABLE = 1;
        syncTCC(TCCx);
      }
    } else {
      value = mapResolution(value, tcs.resolution, tcs.period);

      if (tcNum >= TCC_INST_NUM) {
        Tc* TCx = (Tc*) GetTC(pinDesc.ulPWMChannel);
        TCx->COUNT8.CC[tcChannel].reg = (uint8_t) value;
        syncTC_8(TCx);
      } else {
        Tcc* TCCx = (Tcc*) GetTC(pinDesc.ulPWMChannel);
        TCCx->CTRLBSET.bit.LUPD = 1;
        syncTCC(TCCx);
        TCCx->CCB[tcChannel].reg = (uint32_t) value;
        syncTCC(TCCx);
        TCCx->CTRLBCLR.bit.LUPD = 1;
        syncTCC(TCCx);
      }
    }
  }

  // Nothing to do.
}

#endif

