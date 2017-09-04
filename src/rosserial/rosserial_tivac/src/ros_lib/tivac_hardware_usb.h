/*
 * Copyright (c) 2015, Robosavvy Ltd.
 * All rights reserved.
 * Author: Vitor Matos
 *
 * Redistribution and use in source and binary forms, with or without modification, are permitted provided that the
 * following conditions are met:
 *
 *   1. Redistributions of source code must retain the above copyright notice, this list of conditions and the
 * following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice, this list of conditions and the
 * following disclaimer in the documentation and/or other materials provided with the distribution.
 *   3. Neither the name of the copyright holder nor the names of its contributors may be used to endorse or promote
 * products derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED
 * WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE
 * GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

//*****************************************************************************
//
// Bare minimum hardware resources allocated for rosserials communication.
// * 2 LEDs if desired
// * One USB port
// * Two USBBuffers of UART_BUFFER_SIZE
// * Systick Interrupt handler
//
//*****************************************************************************

#ifndef ROS_LIB_TIVAC_HARDWARE_USB_H
#define ROS_LIB_TIVAC_HARDWARE_USB_H

#include <stdbool.h>
#include <stdint.h>
extern "C"
{
  #include <inc/hw_types.h>
  #include <inc/hw_memmap.h>
  #include <inc/hw_ints.h>
  #include <driverlib/sysctl.h>
  #include <driverlib/gpio.h>
  #include <driverlib/rom.h>
  #include <driverlib/rom_map.h>
  #include <driverlib/systick.h>
  #include <driverlib/pin_map.h>
  #include <driverlib/usb.h>
  #include <usblib/usblib.h>
  #include <usblib/usbcdc.h>
  #include <usblib/usb-ids.h>
  #include <usblib/device/usbdevice.h>
  #include <usblib/device/usbdcdc.h>
  #include "usb_serial_structs.h"
}

#define SYSTICKHZ  1000UL

#ifdef TM4C123GXL
#define LED1        GPIO_PIN_3  // Green LED
#define LED2        GPIO_PIN_2  // Blue LED
#define LED_PORT    GPIO_PORTF_BASE
#define LED_PERIPH  SYSCTL_PERIPH_GPIOF
#endif

#ifdef TM4C1294XL
#define LED1        GPIO_PIN_1  // D1 LED
#define LED2        GPIO_PIN_0  // D2 LED
#define LED_PORT    GPIO_PORTN_BASE
#define LED_PERIPH  SYSCTL_PERIPH_GPION
#ifndef TM4C129FREQ
#error "Must define system clock frequency on: TM4C129FREQ"
#endif
#endif

extern volatile uint32_t g_ui32milliseconds;
extern volatile uint32_t g_ui32heartbeat;

class TivaCHardware
{
  public:
    TivaCHardware() {}

    void init()
    {
#ifdef TM4C123GXL
      this->ui32SysClkFreq = MAP_SysCtlClockGet();
#endif
#ifdef TM4C1294XL
      this->ui32SysClkFreq = TM4C129FREQ;
#endif

      // Setup LEDs
#if defined(LED_HEARTBEAT) || defined(LED_COMM)
      MAP_SysCtlPeripheralEnable(LED_PERIPH);
#endif
#ifdef LED_HEARTBEAT
      MAP_GPIOPinTypeGPIOOutput(LED_PORT, LED1);
#endif
#ifdef LED_COMM
      MAP_GPIOPinTypeGPIOOutput(LED_PORT, LED2);
#endif

      // Enable time keeping
      g_ui32milliseconds = 0;
      // Set up timer such that it produces one tick for each millisecond
      SysTickIntRegister(TivaCHardware::SystickIntHandler);
      MAP_SysTickPeriodSet(this->ui32SysClkFreq/SYSTICKHZ);
      MAP_SysTickEnable();
      MAP_SysTickIntEnable();

      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_USB0);
#ifdef TM4C123GXL
      // Configure the required pins for USB operation.
      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
      MAP_GPIOPinTypeUSBAnalog(GPIO_PORTD_BASE, GPIO_PIN_5 | GPIO_PIN_4);
#endif
#ifdef TM4C1294XL
      MAP_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOL);
      MAP_GPIOPinTypeUSBAnalog(GPIO_PORTL_BASE, GPIO_PIN_6 | GPIO_PIN_7);
#endif

      // Initialize the transmit and receive buffers.
      USBBufferInit(&g_sTxBuffer);
      USBBufferInit(&g_sRxBuffer);

      // Set the USB stack mode to Device mode with VBUS monitoring.
      USBStackModeSet(0, eUSBModeForceDevice, 0);

      // Pass our device information to the USB library and place the device
      // on the bus.
      USBDCDCInit(0, &g_sCDCDevice);

      // Register USB interrupt handler
      USBIntRegister(USB0_BASE, USB0DeviceIntHandler);

      // Enable processor interrupts.
      MAP_IntMasterEnable();
    }

    // read a byte from the serial port. -1 = failure
    int read()
    {
      uint8_t ui8ReadData;
      if (USBBufferRead(&g_sRxBuffer, &ui8ReadData, 1) == 1)
      {
#ifdef LED_COMM
        // Blink the LED to show a character transfer is occuring.
        MAP_GPIOPinWrite(LED_PORT, LED2, MAP_GPIOPinRead(LED_PORT, LED2)^LED2);
#endif
        return ui8ReadData;
      }
      else
        return -1;
    }

    // write data to the connection to ROS
    void write(uint8_t* data, int length)
    {
#ifdef LED_COMM
      // Blink the LED to show a character transfer is occuring.
      MAP_GPIOPinWrite(LED_PORT, LED2, MAP_GPIOPinRead(LED_PORT, LED2)^LED2);
#endif
      // Let's assume for now all length is available for storage in the buffer.
      // Otherwise must use: USBBufferSpaceAvailable
      USBBufferWrite(&g_sTxBuffer, data, length);
    }

    // returns milliseconds since start of program
    uint32_t time()
    {
      return g_ui32milliseconds;
    }

    // Timing variables and System Tick interrupt handler.
    static void SystickIntHandler()
    {
      ++g_ui32milliseconds;
#ifdef LED_HEARTBEAT
      if (++g_ui32heartbeat >= SYSTICKHZ)
      {
        MAP_GPIOPinWrite(LED_PORT, LED1, MAP_GPIOPinRead(LED_PORT, LED1)^LED1);
        g_ui32heartbeat = 0;
      }
#endif
    }

    // System frequency
    uint32_t ui32SysClkFreq;
    uint32_t getSysClkFreq(void)
    {
      return this->ui32SysClkFreq;
    }

    // Not really accurate ms delay. But good enough for our purposes.
    // For a more elaborate delay check out ``Energia/hardware/lm4f/cores/lm4f/wiring.c``
    void delay(uint32_t ms)
    {
      while (ms > 500)
      {
        MAP_SysCtlDelay(this->ui32SysClkFreq/3/SYSTICKHZ * 500);
        ms -= 500;
      }
      MAP_SysCtlDelay(this->ui32SysClkFreq/3/SYSTICKHZ * ms);
    }
};
#endif  // ROS_LIB_TIVAC_HARDWARE_USB_H
