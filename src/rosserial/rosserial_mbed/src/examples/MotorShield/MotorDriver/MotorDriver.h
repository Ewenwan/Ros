/*
  MotorDriver.h
  2014 Copyright (c) Seeed Technology Inc.  All right reserved.

  Author:lawliet.zou@gmail.com
  2014-02-11

  This library is free software; you can redistribute it and/or
  modify it under the terms of the GNU Lesser General Public
  License as published by the Free Software Foundation; either
  version 2.1 of the License, or (at your option) any later version.

  This library is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  Lesser General Public License for more details.

  You should have received a copy of the GNU Lesser General Public
  License along with this library; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA  02110-1301  USA
*/
#ifndef __MOTORDRIVER_H__
#define __MOTORDRIVER_H__

#include "mbed.h"
#include "SoftwarePWM.h"

/**************Motor ID**********************/
#define MOTORA                  0
#define MOTORB                  1

#define LOW                     0
#define HIGH                    1

#define MOTOR_POSITION_LEFT     0
#define MOTOR_POSITION_RIGHT    1
#define MOTOR_CLOCKWISE         0
#define MOTOR_ANTICLOCKWISE     1

#define USE_DC_MOTOR            0

#define MOTOR_PERIOD            10000 //10ms

struct MotorStruct
{
  uint8_t speed;
  uint8_t direction;
  uint8_t position;
};

/** Motor Driver class.
 *  offer API to control the movement of motor
 */
class MotorDriver
{

public:

  /** Create Motor Driver instance
   *  @param int1 pin 1 of motor movement control
   *  @param int2 pin 2 of motor movement control
   *  @param int3 pin 3 of motor movement control
   *  @param int4 pin 4 of motor movement control
      @param speedA speed control of motorA
      @param speedB speed control of motorB
   */
  MotorDriver(PinName int1, PinName int2, PinName int3, PinName int4, PinName speedA, PinName speedB): _int1(int1), _int2(int2), _int3(int3), _int4(int4), _speedA(speedA), _speedB(speedB)
  {
    _int1 = 0;
    _int2 = 0;
    _int3 = 0;
    _int4 = 0;
  };

  /** set motor to initialized state
   */
  void init();

  /** config position of motor
   *  @param position the position set to motor,MOTOR_POSITION_LEFT or MOTOR_POSITION_RIGHT will be allowed
   *  @param motorID the ID define which motor will be set, you can choose MOTORA or MOTORB
   */
  void configure(uint8_t position, uint8_t motorID);

  /** set speed of motor
   *  @param speed speed value set to motor, [0,100] will be allowed
   *  @param motorID the ID define which motor will be set, you can choose MOTORA or MOTORB
   */
  void setSpeed(uint8_t speed, uint8_t motorID);

  /** set direction of motor,
   *  @param direction the direction of motor, MOTOR_CLOCKWISE or MOTOR_ANTICLOCKWISE will be allowed
   *  @param motorID the ID define which motor will be set, you can choose MOTORA or MOTORB
   */
  void setDirection(uint8_t direction, uint8_t motorID);

  /** rotate motor
   *  @param direction the direction set to motor, MOTOR_CLOCKWISE or MOTOR_ANTICLOCKWISE will be allowed
   *  @param motor_position the position set to motor,MOTOR_POSITION_LEFT or MOTOR_POSITION_RIGHT will be allowed
   */
  void rotate(uint8_t direction, uint8_t motor_position);

  /** rotate motorA or motorB
   *  @param direction the direction set to motor, MOTOR_CLOCKWISE or MOTOR_ANTICLOCKWISE will be allowed
   *  @param motorID the ID define which motor will be set, you can choose MOTORA or MOTORB
   */
  void rotateWithID(uint8_t direction, uint8_t motorID);

  /** make motor go forward
   */
  void goForward();

  /** make motor go backward
   */
  void goBackward();

  /** make motor go left
   */
  void goLeft();

  /** make motor go right
   */
  void goRight();

  /** make motor stop
   */
  void stop();

  /** make motorA or motorB stop
   *  @param motorID the ID define which motor will be set, you can choose MOTORA or MOTORB
   */
  void stop(uint8_t motorID);

  /** motor A
   */
  MotorStruct motorA;

  /** motor B
   */
  MotorStruct motorB;

private:

  DigitalOut  _int1;
  DigitalOut  _int2;
  DigitalOut  _int3;
  DigitalOut  _int4;
  SoftwarePWM _speedA;
  SoftwarePWM _speedB;
};
#endif
