/*
 * Hello World Example.
 * This is used to test timing, and the UART send and receive.
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
#include <airbourne.h>

UART Serial1;

static void _putc(void *p, char c)
{
    (void)p; // avoid compiler warning about unused variable
    Serial1.put_byte((uint8_t*)&c, 1);
}

void recieve_byte_CB(uint8_t c)
{
    printf("%c", c);
}

int main(void)
{
    airbourne_init();

    Serial1.init(2, 115200, UART::MODE_DMA_TX_RX);
    init_printf(NULL, _putc);
    LED0.toggle();

    Serial1.register_receive_CB(&recieve_byte_CB);

    while (1)
    {
        delay_ms(1000);
    }
}
