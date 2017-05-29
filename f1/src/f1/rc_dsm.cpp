#include "rc_dsm.h"

RC_DSM* RC_DSMPtr;

void byte_received_callback(uint8_t c);

RC_DSM::RC_DSM()
{
    RC_DSMPtr = this;
}

void RC_DSM::init()
{
    init(DSM_2048);
}

void RC_DSM::init(dsm_type_t dsm_type)
{
    // Configure the class object
    dsm_type_ = dsm_type;

    switch(dsm_type)
    {
    case DSM_2048:
        shift_ = 3;
        mask_ = 0x07;
        num_channels_ = DSM_2048_MAX_CHANNELS;
        break;
    case DSM_1024:
        shift_ = 2;
        mask_ = 0x03;
        num_channels_ = DSM_1024_MAX_CHANNELS;
        break;
    }

    pos_ = 0;
    last_data_receive_us_ = 0;
    incoming_ = false;
    new_frame_ = false;

    // Initialize the UART
    uart_.init(DSM_UART_PORT, 115200, UART::MODE_DMA_RX);

    // Register the UART callback
    uart_.register_receive_CB(&byte_received_callback);
}


void RC_DSM::handle_byte(uint8_t c)
{
    incoming_ = true;
    uint64_t now = micros();
    uint64_t diff = now - last_data_receive_us_;
    last_data_receive_us_ = now;
    if (diff > 5000)
    {
        pos_ = 0;
    }

    frame_[pos_] = c;
    pos_++;

    if(pos_ == DSM_FRAME_SIZE)
    {
        new_frame_ = true;
        pos_ = 0;
    }
}

uint32_t RC_DSM::readus(uint8_t channel)
{
    // Convert the raw byte stream into the RC struct before reading
    if (new_frame_)
    {
        for (int i = 3; i < DSM_FRAME_SIZE; i += 2)
        {
            uint8_t chan = 0x0F & (frame_[i -1] >> shift_);
            if(chan < num_channels_)
            {
                uint16_t raw = (uint16_t) (((frame_[i -1] & mask_ ) << 8) | frame_[i]);
                if(DSM_2048)
                {
                    RC_raw_[chan] = (raw >> 1) + 988;
                }
                else
                {
                    RC_raw_[chan] = raw + 988;
                }
            }
        }
        new_frame_ = false;
    }

    if (channel >= NUM_RC_INPUTS)
    {
        return 1500;
    }
    else
    {
        return RC_raw_[channel];
    }
}

float RC_DSM::read(uint8_t channel)
{
    return (float)(readus(channel) - MIN_US) / (float)(MAX_US - MIN_US);
}


void byte_received_callback(uint8_t c)
{
    RC_DSMPtr->handle_byte(c);
}
