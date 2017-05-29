/*
 * led.cpp - Implementation of LED class which abstracts LED functionality with
 * the naze32
 *
 * Copyright (c) 2016 James Jackson
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "led.h"
#include "board.h"

LED::LED(){}

LED::LED(GPIO_TypeDef* port, uint16_t pin) :
  gpio_(port, pin, GPIO_Mode_Out_PP)
{
  gpio_.write(HIGH);
}

void LED::toggle()
{
  gpio_.toggle();
}

void LED::turn_on()
{
  gpio_.write(LOW);
}

void LED::turn_off()
{
  gpio_.write(HIGH);
}

void LED::blink(uint32_t ms_delay)
{
  static uint32_t last_time = 0;
  uint32_t now = millis();

  if(now - last_time > ms_delay)
  {
    toggle();
    last_time = now;
  }
}
