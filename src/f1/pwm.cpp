#include "pwm.h"
#include "gpio.h"
#include "board.h"



PWM_Out::PWM_Out()
{}

void PWM_Out::init(uint8_t pin, uint16_t frequency, uint32_t max_us, uint32_t min_us)
{
  // Resolve the appropriate pin and timer
  GPIO_TypeDef* GPIOPtr;
  uint16_t GPIO_Pin;
  TIM_TypeDef* TIMPtr;
  uint16_t channel;

  if(pin < 7)
  {
    GPIOPtr = pwm_hardware[pin - 1].GPIO;
    GPIO_Pin = pwm_hardware[pin - 1].pin;
    TIMPtr = pwm_hardware[pin - 1].TIM;
    channel = pwm_hardware[pin - 1].channel;
  }

  else
  {
    GPIOPtr = rc_hardware[pin - 1].GPIO;
    GPIO_Pin = rc_hardware[pin - 1].pin;
    TIMPtr = rc_hardware[pin - 1].TIM;
    channel = rc_hardware[pin - 1].channel;
  }

  uint32_t timer_prescaler = 8;
  uint32_t timer_freq_hz = SystemCoreClock / timer_prescaler;

  cycles_per_us_ = timer_freq_hz / 1000000;
  max_cyc_ = max_us * cycles_per_us_;
  min_cyc_ = min_us * cycles_per_us_;

  uint16_t period_cyc = timer_freq_hz / frequency;

  // Initialize the pin
  pin_.init(GPIOPtr, GPIO_Pin, GPIO_Mode_Out_PP);

  TIM_ = TIMPtr;

  // ConfigTimeBase
  TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
  TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
  TIM_TimeBaseStructure.TIM_Period = period_cyc - 1; // (0 - index)
  TIM_TimeBaseStructure.TIM_Prescaler = timer_prescaler - 1;  // ( 0 - indexed)
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIMPtr, &TIM_TimeBaseStructure);

  pin_.set_mode(GPIO_Mode_AF_PP);

  // pwmOCConfig
  TIM_OCInitTypeDef TIM_OCInitStructure;

  TIM_OCStructInit(&TIM_OCInitStructure);
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM2;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_OutputNState = TIM_OutputNState_Disable;
  TIM_OCInitStructure.TIM_Pulse = min_cyc_ - 1;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_Low;
  TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;

  if(TIMPtr == TIM1 || TIMPtr == TIM8)
  {
    TIM_CtrlPWMOutputs(TIMPtr, ENABLE);
  }
  switch (channel) {
      case TIM_Channel_1:
          TIM_OC1Init(TIMPtr, &TIM_OCInitStructure);
          TIM_OC1PreloadConfig(TIMPtr, TIM_OCPreload_Enable);
          CCR_ = &TIMPtr->CCR1;
          break;
      case TIM_Channel_2:
          TIM_OC2Init(TIMPtr, &TIM_OCInitStructure);
          TIM_OC2PreloadConfig(TIMPtr, TIM_OCPreload_Enable);
          CCR_ = &TIMPtr->CCR2;
          break;
      case TIM_Channel_3:
          TIM_OC3Init(TIMPtr, &TIM_OCInitStructure);
          TIM_OC3PreloadConfig(TIMPtr, TIM_OCPreload_Enable);
          CCR_ = &TIMPtr->CCR3;
          break;
      case TIM_Channel_4:
          TIM_OC4Init(TIMPtr, &TIM_OCInitStructure);
          TIM_OC4PreloadConfig(TIMPtr, TIM_OCPreload_Enable);
          CCR_ = &TIMPtr->CCR4;
          break;
  }

  TIM_Cmd(TIMPtr, ENABLE);
}

void PWM_Out::enable()
{
  pin_.set_mode(GPIO_Mode_AF_PP);
}

void PWM_Out::disable()
{
  pin_.set_mode(GPIO_Mode_Out_PP);
  pin_.write(LOW);
}

void PWM_Out::write(float value)
{
  *CCR_ = min_cyc_ + (uint16_t)((max_cyc_ - min_cyc_) * value);
}

void PWM_Out::writeus(uint32_t us)
{
  *CCR_ = us*cycles_per_us_;
}
