
// Script to convert encoder speed to motor speed
float ConvertEncoder2Speed(float encVal, int motor, float motspeedIn)
{
  float zerothresh = 3.5; // amplitude around zero that is used to ensure stability around zero
  int minmotspeed = -127;
  int maxmotspeed = 127;

  if (motor == 0)
  { //Left motor
    bRev = riseB[1];  //Calibration equation values in the form y=mx+b.  Separate values are used for forward and reverse because of discontinuity near 0
    mRev = risingslope[1];
    bFor = fallB[0];
    mFor = fallingslope[0];

  }
  else if (motor == 1)
  { //Right motor
    bRev = riseBR[1];
    mRev = risingslopeR[1];
    bFor = fallBR[0];
    mFor = fallingslopeR[0];
  }

  float b = (bRev + bFor) / 2;
  float m = (mRev + mFor) / 2;

  if ((abs(encVal) < zerothresh) )
  { // If the encoder value is near stopped, make the motors stop.  This is to prevent small jittery adjustments when wheels are stopped
    motspeed = 0;
  }

  else
  {
    if (Calibrate == 0) {
      motspeed = encVal / 285 * 127;

    }
    else {
      ////Reverse
      if (encVal >= zerothresh)   // if positive speed
      {
        //    motspeed = ((encVal - bRev) / mRev);  //Implement the calibration equation
        motspeed = encVal * mRev  ; //Implement the calibration equation
      }


      ////Forward, same mechanics as forward, but values reversed
      else if (encVal <= -zerothresh) //if negative speed
      {
        //    motspeed = ((encVal - bFor) / mFor);
        motspeed = encVal * mFor  ; //Implement the calibration equation
      }
    }
  }
  // Limit the output to not go over 160 or under 20 because the encoder reads too fast there
  if (motspeed > maxmotspeed)   {
    motspeed = maxmotspeed;
  }
  if (motspeed < minmotspeed)    {
    motspeed = minmotspeed;
  }

  return motspeed;
}
