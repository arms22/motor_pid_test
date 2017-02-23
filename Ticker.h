#ifndef __TICKER__
#define __TICKER__

//#define TICKER_TC ((TcCount16*)TC4)
//#define TICKER_CLKCTRL_ID (GCLK_CLKCTRL_ID_TC4_TC5)
//#define TICKER_IRQn TC4_IRQn
//#define TICKER_IRQ_Handler TC4_Handler
//
//#define TICKER2_TC ((TcCount16*)TC3)
//#define TICKER2_CLKCTRL_ID (GCLK_CLKCTRL_ID_TCC2_TC3)
//#define TICKER2_IRQn TC5_IRQn
//#define TICKER2_IRQ_Handler TC3_Handler

typedef void (*TickerFuncPtr)(void);

typedef struct _TickerSetting {
  TcCount16 *TC;
  int tc_channel;
  int clkctrl_id;
  IRQn_Type irq;
  int priority;
  int compare;
  TickerFuncPtr handler;
} TickerSetting;

static TickerSetting _tickerSetting[4] = {
  { (TcCount16*)TC4, 0, GCLK_CLKCTRL_ID_TC4_TC5, TC4_IRQn,  0, 0, NULL, },
  { (TcCount16*)TC4, 1, GCLK_CLKCTRL_ID_TC4_TC5, TC4_IRQn,  0, 0, NULL, },
  { (TcCount16*)TC3, 0, GCLK_CLKCTRL_ID_TCC2_TC3, TC3_IRQn, 3, 0, NULL, },
  { (TcCount16*)TC3, 1, GCLK_CLKCTRL_ID_TCC2_TC3, TC3_IRQn, 3, 0, NULL, },
};

static void ticker_init(int channel, TickerFuncPtr handler, int frequency)
{
  TickerSetting &ts = _tickerSetting[channel];

  ts.handler = handler;

  GCLK->CLKCTRL.reg = (GCLK_CLKCTRL_CLKEN | GCLK_CLKCTRL_GEN_GCLK0 | ts.clkctrl_id);
  while (GCLK->STATUS.bit.SYNCBUSY == 1);

  // Disable TC
  ts.TC->CTRLA.bit.ENABLE = 0;
  while (ts.TC->STATUS.bit.SYNCBUSY);

  // Set Timer counter Mode to 16bits, Normal Frequency, prescaler 1/64
  ts.TC->CTRLA.reg = TC_CTRLA_MODE_COUNT16 | TC_CTRLA_WAVEGEN_NFRQ | TC_CTRLA_PRESCALER_DIV64;
  while (ts.TC->STATUS.bit.SYNCBUSY);

  // Set the initial value
  ts.compare = 750000 / frequency;
  ts.TC->CC[ts.tc_channel].reg = ts.compare - 1;
  while (ts.TC->STATUS.bit.SYNCBUSY);

  // Set the Match Interrupt Enable
  ts.TC->INTENSET.reg |= TC_INTENSET_MC(1 << ts.tc_channel);

  // Clear the Match Interrupt Status
  ts.TC->INTFLAG.reg = TC_INTFLAG_MC(1 << ts.tc_channel);

  // Set the Match Interrupt Enable
  //    TICKER_TC->INTENSET.bit.MC0 = 1;
  //    TICKER_TC->INTENSET.bit.MC1 = 1;

  // Clear the Match Interrupt Status
  //    TICKER_TC->INTFLAG.bit.MC0 = 1;
  //    TICKER_TC->INTFLAG.bit.MC1 = 1;

  // Enable TC Interrupt
  NVIC_SetPriority(ts.irq, ts.priority);
  NVIC_EnableIRQ(ts.irq);

  // Enable TC
  ts.TC->CTRLA.bit.ENABLE = 1;
  while (ts.TC->STATUS.bit.SYNCBUSY);
}

static uint32_t ticker_count(int channel)
{
  TickerSetting &ts = _tickerSetting[channel];
  ts.TC->READREQ.reg = TC_READREQ_RREQ | TC_READREQ_ADDR(0x10);
  while (ts.TC->STATUS.bit.SYNCBUSY);
  return ts.TC->COUNT.reg;
}

void attachTickerInterrupt(int channel, TickerFuncPtr callback, int frequency)
{
  ticker_init(channel, callback, frequency);
}

void TC4_Handler()
{
  // channel 0-1
  for (int i = 0; i < 2; i++)
  {
    TickerSetting &ts = _tickerSetting[i];
    if (ts.TC->INTFLAG.reg | TC_INTFLAG_MC(1 << ts.tc_channel)) {

      // Update CCx value
      ts.TC->CC[ts.tc_channel].reg += ts.compare;
      while (ts.TC->STATUS.bit.SYNCBUSY);

      // Call the callback function if assigned
      if (ts.handler)
        ts.handler();

      ts.TC->INTFLAG.reg = TC_INTFLAG_MC(1 << ts.tc_channel);
    }
  }
}

void TC3_Handler()
{
  // channel 2-3
  for (int i = 2; i < 4; i++)
  {
    TickerSetting &ts = _tickerSetting[i];
    if (ts.TC->INTFLAG.reg | TC_INTFLAG_MC(1 << ts.tc_channel)) {

      // Update CCx value
      ts.TC->CC[ts.tc_channel].reg += ts.compare;
      while (ts.TC->STATUS.bit.SYNCBUSY);

      // Call the callback function if assigned
      if (ts.handler)
        ts.handler();

      ts.TC->INTFLAG.reg = TC_INTFLAG_MC(1 << ts.tc_channel);
    }
  }
}

#endif

