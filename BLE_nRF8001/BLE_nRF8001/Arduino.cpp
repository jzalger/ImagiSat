/*
Copyright (c) 2012-2014 RedBearLab

Permission is hereby granted, free of charge, to any person obtaining a copy of this software 
and associated documentation files (the "Software"), to deal in the Software without restriction, 
including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, 
and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, 
subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, 
INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR 
PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE 
FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, 
ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/

/*
* This file is used to map the pins of Arduino to mbed 
*/

#include "mbed.h"
#include "Arduino.h"

void pinMode(DigitalInOut *pin, int mode) 
{
    if (mode == INPUT)
    {
        pin->input();
    }
    else if (mode == INPUT_PULLUP)
    {
        pin->input();
        pin->mode(PullUp);
    }
    else if (mode == OUTPUT)
    {
        pin->output();
    }    
}

void digitalWrite(DigitalInOut *pin, int value) 
{
    pin->write(value);
}

int digitalRead(DigitalInOut *pin)
{
    return pin->read();
}