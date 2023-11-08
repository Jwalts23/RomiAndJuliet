#include "serial_comm.h"
#include "PIDcontroller.h"

String serialInput;

bool CheckSerialInput(void)
{
  if(Serial.available())
  {
    char c = Serial.read();
    serialInput += c;
    if(c == '\n') return true;
  }

  return false;
}

void ParseSerialInput(void)
{
  float k = 0;
  switch(serialInput[0])
  {
    case 'P':
      k = serialInput.substring(1).toFloat();
      leftMotorController.SetKp(k);
      rightMotorController.SetKp(k);
      break;
    case 'I':
      k = serialInput.substring(1).toFloat();
      leftMotorController.SetKi(k);
       rightMotorController.SetKi(k);
      break;
    case 'D':
      k = serialInput.substring(1).toFloat();
      leftMotorController.SetKd(k);
      rightMotorController.SetKi(k);
      break;
  }

  serialInput = "";
}
