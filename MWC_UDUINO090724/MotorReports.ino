
float MotorReportL()
{ //Counts the speed of each quadrature encoder using the interrupt function to provide highest priority for a short burst of time
  countL = 0;
  attachInterrupt(c_LeftEncoderInterrupt, HandleLeftMotorInterruptA, RISING);
  delayMicroseconds(counttime);
  detachInterrupt(c_LeftEncoderInterrupt);
  encL = countL;
  return encL;
}


float MotorReportR()
{
  countR = 0;
  attachInterrupt(c_RightEncoderInterrupt, HandleRightMotorInterruptA, RISING);
  delayMicroseconds(counttime);
  detachInterrupt(c_RightEncoderInterrupt);
  encR = countR;
  return encR;
}
