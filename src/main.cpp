/************************************************************************
 * Copyright 2018 © Kévin Lesénéchal <kevin.lesenechal@gmail.com>       *
 * This file is part of MotoLogger.                                     *
 *                                                                      *
 * MotoLogger is free software: you can redistribute it and/or modify   *
 * it under the terms of the GNU General Public License as published by *
 * the Free Software Foundation, either version 3 of the License, or    *
 * (at your option) any later version.                                  *
 *                                                                      *
 * MotoLogger is distributed in the hope that it will be useful,        *
 * but WITHOUT ANY WARRANTY; without even the implied warranty of       *
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the        *
 * GNU General Public License for more details.                         *
 *                                                                      *
 * You should have received a copy of the GNU General Public License    *
 * along with MotoLogger.  If not, see <http://www.gnu.org/licenses/>.  *
 ************************************************************************/

#include <hw.hpp>
#include <fs.hpp>
#include <main.hpp>

#include <motologger/application.hpp>

#include <stdarg.h>

int main()
{
    for (;;) {
        hw::init();
        hw::set_rtc_timestamp(1540493949);
        fs::init();

        {
            motologger::application app;
            app.run();
        }

        fs::deinit();
        hw::stop();
    }
}

void panic(const char* errf, ...)
{
    va_list ap;
    bool toggle = true;

    va_start(ap, errf);
    printf("*** PANIC! *** ");
    vprintf(errf, ap);
    printf("\n");
    va_end(ap);

    for (;;) {
        hw::set_led(hw::led::error, toggle);
        toggle = !toggle;
        HAL_Delay(200);
    }
}
