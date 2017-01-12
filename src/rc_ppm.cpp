#include "rc_ppm.h"

RC_PPM* RC_PPMPtr = NULL;

RC_PPM::RC_PPM(){}


void RC_PPM::init()
{
    // Copy data from the hardware configuration array into the class
    GPIO_ = rc_hardware[0].GPIO;
    TIM_ = rc_hardware[0].TIM;
    pin_number_ = rc_hardware[0].pin;
    channel_ = rc_hardware[0].channel;
    IRQ_Channel_ = rc_hardware[0].IQR_Channel;

    // Link up external C Pointer
    RC_PPMPtr = this;

    // zero out all measurement variables
    current_capture_ = 0;
    last_capture_ = 0;
    chan_ = 0;

    pin_.init(GPIO_, pin_number_, GPIO_Mode_IN_FLOATING);

    // Configure the Capture Timer
    TIM_ICInitTypeDef TIM_ICInitStruct;
    TIM_ICStructInit(&TIM_ICInitStruct);
    TIM_ICInitStruct.TIM_Channel = channel_;
    TIM_ICInitStruct.TIM_ICPolarity = TIM_ICPolarity_Rising;
    TIM_ICInitStruct.TIM_ICSelection = TIM_ICSelection_DirectTI;
    TIM_ICInitStruct.TIM_ICPrescaler = TIM_ICPSC_DIV1;
    TIM_ICInitStruct.TIM_ICFilter = false; // Could enable if we want to use the pwm filter
    TIM_ICInit(TIM_, &TIM_ICInitStruct);

    // Configure the Timer itself
    TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
    TIM_TimeBaseStructInit(&TIM_TimeBaseStructure);
    TIM_TimeBaseStructure.TIM_Period = 0xFFFE;
    TIM_TimeBaseStructure.TIM_Prescaler = (SystemCoreClock / (1000000)) - 1;
    TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM_, &TIM_TimeBaseStructure);

    // Start counting!
    TIM_Cmd(TIM_, ENABLE);

    // Set up the Interrupt for the PPM
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_InitStructure.NVIC_IRQChannel = TIM2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    // Tell the Timer to interrupt
    switch(channel_)
    {
    case TIM_Channel_1:
        TIM_ITConfig(TIM_, TIM_IT_CC1, ENABLE);
        break;
    case TIM_Channel_2:
        TIM_ITConfig(TIM_, TIM_IT_CC2, ENABLE);
        break;
    case TIM_Channel_3:
        TIM_ITConfig(TIM_, TIM_IT_CC3, ENABLE);
        break;
    case TIM_Channel_4:
        TIM_ITConfig(TIM_, TIM_IT_CC4, ENABLE);
        break;
    }
}

float RC_PPM::read(uint8_t channel)
{
    return (float)(RC_raw_[channel] - MIN_US) / (float)(MAX_US - MIN_US);
}

uint32_t RC_PPM::readus(uint8_t channel)
{
    return RC_raw_[channel];
}


void RC_PPM::pulse_callback()
{
    if(TIM_GetITStatus(TIM_, TIM_IT_CC1))
    {
        TIM_ClearITPendingBit(TIM_, TIM_IT_CC1);

        current_capture_ = TIM_GetCapture1(TIM_);
        uint16_t diff = current_capture_ - last_capture_;
        last_capture_ = current_capture_;

        // We're on a new frame
        if(diff > 2500)
        {
            chan_ = 0;
        }
        else
        {
            // If it's a valid reading, then save it!
            if(diff > 750 && diff < 2250 && chan_ < NUM_RC_INPUTS)
            {
                RC_raw_[chan_] = diff;
            }
            chan_++;
        }
    }
}

extern "C"
{

void TIM1_CC_IRQHandler(void)
{
    if(RC_PPMPtr != NULL)
    {
        RC_PPMPtr->pulse_callback();
    }
}

void TIM2_IRQHandler(void)
{
    if(RC_PPMPtr != NULL)
    {
        RC_PPMPtr->pulse_callback();
    }
}

void TIM3_IRQHandler(void)
{
    volatile int test = 0;
}

void TIM4_IRQHandler(void)
{
    volatile int test = 0;
}
}
