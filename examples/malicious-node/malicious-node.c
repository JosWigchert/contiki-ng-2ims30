/*
 * Copyright (c) 2006, Swedish Institute of Computer Science.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the Institute nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE INSTITUTE AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE INSTITUTE OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * This file is part of the Contiki operating system.
 *
 */

/**
 * \file
 *         A very simple Contiki application showing how Contiki programs look
 * \author
 *         Adam Dunkels <adam@sics.se>
 */

#include "contiki.h"
#include "gpio-hal.h"

#include <stdio.h> /* For printf() */

#define SINKHOLE
#define BLACKHOLE

#define BUTTON_0_PIN 0
#define BUTTON_1_PIN 4

#define LED_0_PIN 10
#define LED_1_PIN 15

/*---------------------------------------------------------------------------*/

#ifdef SINKHOLE
bool sinkhole_activated = false;
#endif
#ifdef BLACKHOLE
bool blackhole_activated = false;
#endif
/*---------------------------------------------------------------------------*/

PROCESS(gpio_process, "GPIO handler");
PROCESS(watchdog_process, "Watchdog handler");

AUTOSTART_PROCESSES(&gpio_process, &watchdog_process);

/*---------------------------------------------------------------------------*/

PROCESS_THREAD(gpio_process, ev, data)
{
  static struct etimer timer;

  PROCESS_BEGIN();

  /* Setup a periodic timer that expires after 10 seconds. */
  etimer_set(&timer, CLOCK_SECOND * .0167);

  static uint8_t prev_button0 = 1;
  static uint8_t prev_button1 = 1;

  while(1) 
  {
    uint8_t button0value = gpio_hal_arch_read_pin(0, BUTTON_0_PIN);
    uint8_t button1value = gpio_hal_arch_read_pin(0, BUTTON_1_PIN);

    if (button0value != prev_button0 && button0value == 0)
    {
      #ifdef SINKHOLE
      sinkhole_activated = !sinkhole_activated;

      if (sinkhole_activated)
      {
        printf("Sinkhole Activated\n");
        gpio_hal_arch_write_pin(0, LED_0_PIN, true);
      }
      else
      {
        printf("Sinkhole Deactivated\n");
        gpio_hal_arch_write_pin(0, LED_0_PIN, false);

      }
      #endif
    }
    if (button1value != prev_button1 && button1value == 0)
    {
      #ifdef BLACKHOLE
      blackhole_activated = !blackhole_activated;

      if (blackhole_activated)
      {
        printf("Blackhole Activated\n");
        gpio_hal_arch_write_pin(0, LED_1_PIN, true);
      }
      else
      {
        printf("Blackhole Deactivated\n");
        gpio_hal_arch_write_pin(0, LED_1_PIN, false);
      }
      #endif
    }

    prev_button0 = button0value;
    prev_button1 = button1value;

    /* Wait for the periodic timer to expire and then restart the timer. */
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
    etimer_reset(&timer);
  }

  PROCESS_END();
}

PROCESS_THREAD(watchdog_process, ev, data)
{
  static struct etimer timer;

  PROCESS_BEGIN();

  /* Setup a periodic timer that expires after 10 seconds. */
  etimer_set(&timer, CLOCK_SECOND * 10);

  while(1) {
    printf("Hello, Malicious node\n");
    
    /* Wait for the periodic timer to expire and then restart the timer. */
    PROCESS_WAIT_EVENT_UNTIL(etimer_expired(&timer));
    etimer_reset(&timer);
  }

  PROCESS_END();
}
/*---------------------------------------------------------------------------*/
