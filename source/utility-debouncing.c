/*
 * This file is part of the libohiboard project.
 *
 * Copyright (C) 2019 A. C. Open Hardware Ideas Lab
 *
 * Authors:
 *  Marco Giammarini <m.giammarini@warcomeb.it>
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
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

#include "utility-debouncing.h"

System_Errors UtilityDebouncing_debounce (Gpio_Pins pin, UtilityDeboucing_Config* config)
{
    System_Errors ret = ERRORS_UTILITYDEBOUNCING_NO_HOLD;
    if (config->newEvent)
    {
        config->count = (config->holdTime / config->checkTime);
        // Save timeout value
        config->countTimeout = ((config->holdTime + config->debounceTime) / config->checkTime);
        // Setup timeout count...
        config->newEvent = FALSE;
    }
    // read current state
    Gpio_Level current = Gpio_get(pin);

    // Decrease timeout counter
    if (--(config->countTimeout) == 0)
    {
        return ERRORS_UTILITYDEBOUNCING_TIMEOUT;
    }

    // The button has the desiderata level, so wait to become stable
    if (current == config->holdLevel)
    {
        // Check if time expired
        if (--(config->count) == 0)
        {
            ret = ERRORS_UTILITYDEBOUNCING_HOLD;
            // Reset counter
            config->count = config->holdTime / config->checkTime;
        }
    }
    else
    {
        // Reset timer
        config->count = config->holdTime / config->checkTime;
    }
    return ret;
}

System_Errors UtilityDebouncing_countinuousDebounce (Gpio_Pins pin, UtilityDeboucing_Config* config)
{
    System_Errors ret = ERRORS_UTILITYDEBOUNCING_NO_HOLD;
    if (config->newEvent)
    {
        config->count = (config->holdTime / config->checkTime);
        // Setup timeout count...
        config->newEvent = FALSE;
    }
    // read current state
    Gpio_Level current = Gpio_get(pin);

    // The button has the desiderata level, so wait to become stable
    if (current == config->holdLevel)
    {
        // Check if time expired
        if (--(config->count) == 0)
        {
            ret = ERRORS_UTILITYDEBOUNCING_HOLD;
            // Reset counter
            config->count = config->holdTime / config->checkTime;
        }
    }
    else
    {
        // Reset timer
        config->count = config->holdTime / config->checkTime;
    }
    return ret;
}
