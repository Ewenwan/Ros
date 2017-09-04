/*
  MotorDriver.cpp
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

#include "MotorDriver.h"

void MotorDriver::init()
{
  stop();
  /*Configure the motor A to control the wheel at the left side.*/
  configure(MOTOR_POSITION_LEFT, MOTORA);
  /*Configure the motor B to control the wheel at the right side.*/
  configure(MOTOR_POSITION_RIGHT, MOTORB);
  setSpeed(0, MOTORA);
  setSpeed(0, MOTORB);
  setDirection(MOTOR_ANTICLOCKWISE, MOTORA);
  setDirection(MOTOR_CLOCKWISE, MOTORB);
}
void MotorDriver::configure(uint8_t position, uint8_t motorID)
{
  if (motorID == MOTORA)motorA.position = position;
  else motorB.position = position;
}

void MotorDriver::setSpeed(uint8_t speed, uint8_t motorID)
{
  if (motorID == MOTORA) motorA.speed = speed;
  else if (motorID == MOTORB) motorB.speed = speed;
}
void MotorDriver::setDirection(uint8_t direction, uint8_t motorID)
{
  if (motorID == MOTORA)motorA.direction = direction;
  else if (motorID == MOTORB)motorB.direction = direction;
}

void MotorDriver::rotate(uint8_t direction, uint8_t motor_position)
{
  if (motor_position == motorA.position)
  {
    rotateWithID(direction, MOTORA);
  }
  if (motor_position == motorB.position)
  {
    rotateWithID(direction, MOTORB);
  }
}

void MotorDriver::rotateWithID(uint8_t direction, uint8_t motorID)
{
  if (motorID == MOTORA)
  {
    _speedA.Enable(motorA.speed * 100, MOTOR_PERIOD);
    _int1 = (MOTOR_CLOCKWISE == direction) ? LOW : HIGH;
    _int2 = !_int1;
  }
  else if (motorID == MOTORB)
  {
    _speedB.Enable(motorB.speed * 100, MOTOR_PERIOD);
    _int3 = (MOTOR_CLOCKWISE == direction) ? LOW : HIGH;
    _int4 = !_int3;
  }
}

void MotorDriver::goForward()
{
  rotate(MOTOR_ANTICLOCKWISE, MOTOR_POSITION_LEFT);
  rotate(MOTOR_CLOCKWISE, MOTOR_POSITION_RIGHT);
}
void MotorDriver::goBackward()
{
  rotate(MOTOR_ANTICLOCKWISE, MOTOR_POSITION_RIGHT);
  rotate(MOTOR_CLOCKWISE, MOTOR_POSITION_LEFT);
}
void MotorDriver::goLeft()
{
  rotate(MOTOR_CLOCKWISE, MOTOR_POSITION_RIGHT);
  rotate(MOTOR_CLOCKWISE, MOTOR_POSITION_LEFT);
}
void MotorDriver::goRight()
{
  rotate(MOTOR_ANTICLOCKWISE, MOTOR_POSITION_RIGHT);
  rotate(MOTOR_ANTICLOCKWISE, MOTOR_POSITION_LEFT);
}

void MotorDriver::stop()
{
  _speedA.Disable();
  _speedB.Disable();
}

void MotorDriver::stop(uint8_t motorID)
{
  if (motorID == MOTORA)_speedA.Disable();
  else if (motorID == MOTORB)_speedB.Disable();
}

