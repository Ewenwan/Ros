#include "SoftwarePWM.h"
#include "mbed.h"

SoftwarePWM::SoftwarePWM(PinName Pin) : SoftwarePWMPin(Pin) {}

void SoftwarePWM::SetPosition(int Pos)
{
  Position = Pos;
}

void SoftwarePWM::StartPulse()
{
  if (Position <= 0)
  {
    SoftwarePWMPin = 0 ;
  }
  else
  {
    SoftwarePWMPin = 1;
    PulseStop.attach_us(this, &SoftwarePWM::EndPulse, Position);
  }
}

void SoftwarePWM::EndPulse()
{
  SoftwarePWMPin = 0;
}

void SoftwarePWM::Enable(int StartPos, int Period)
{
  Position = StartPos;
  Pulse.attach_us(this, &SoftwarePWM::StartPulse, Period);
}

void SoftwarePWM::Disable()
{
  Pulse.detach();
}