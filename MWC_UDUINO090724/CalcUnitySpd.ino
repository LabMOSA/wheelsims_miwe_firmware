float ConvertEncoder2UnityVals()

//float ConvertEncoder2UnityVals(float encVal, int motor, int speedin)
{ // These values are calculated similary to motor values, but not adjusted for dynamics.  These are used as real-time updates to Unity, so there should be minimal lag,
  //(meaning they're not stored in the buffer) and are just a straight conversion of the encoders to current speed in a form that Unity will accept.
  
  float UnityspeedPower = 1.125;
  float unityvaloffset = 0.092;
  float UnityScalar = 0.75;
  float zerospeedthresh = 0.0125;
  
  
  if (lspeed  > zerospeedthresh)
  { LspeedUnity = -pow(abs(lspeed), UnityspeedPower) / 127 * UnityScalar - unityvaloffset;
  }
  else if (lspeed  < -zerospeedthresh)
  { LspeedUnity = pow(abs(lspeed), UnityspeedPower) / 127 * UnityScalar + unityvaloffset;
  }
  else
  { LspeedUnity = 0;
  }

  if (rspeed  > zerospeedthresh)
  { RspeedUnity = -pow(abs(rspeed), UnityspeedPower) / 127 * UnityScalar - unityvaloffset;
  }
  else if (rspeed  < -zerospeedthresh)
  { RspeedUnity = pow(abs(rspeed), UnityspeedPower) / 127 * UnityScalar + unityvaloffset;
  }
  else
  { RspeedUnity = 0;
  }

  
  
//float rawspeed=0;
//float unityvaloffset=10;
//
//  if ((abs(encVal) < zerothresh) )
//  {
//    rawspeed = 0;
//  }
//
//  
//  else if (encVal> zerothresh)
//  {
//    rawspeed== -encVal * 70 / 220 + unityvaloffset;
//  }
//  else if (encVal< -zerothresh)
//  {
//    rawspeed== -encVal * 70 / 220 - unityvaloffset;
//  }
//  
//  return rawspeed;




  
//  if (motor == 0)
//  { //Left motor
//    bRev = riseB[1];  //Calibration equation values in the form y=mx+b.  Separate values are used for forward and reverse because of discontinuity near 0
//    mRev = risingslope[1];
//    bFor = fallB[0];
//    mFor = fallingslope[0];
//  }
//  else if (motor == 1)
//  { //Right motor
//    bRev = riseBR[1];
//    mRev = risingslopeR[1];
//    bFor = fallBR[0];
//    mFor = fallingslopeR[0];
//  }
//
//float rawspeed=0;
//
//  if ((abs(encVal) < zerothresh) )
//  {
//    rawspeed = 0;
//  }
//
//  ////Forward
//  else if (encVal >= zerothresh)   // if positive speed
//  {
//    rawspeed = ((encVal - bFor) / mFor);  //Implement the calibration equation
//    if (rawspeed > 160)   {
//      rawspeed = 160;
//    }
//    rawspeed = -(rawspeed - 90) / 70;
//  }
//  ////Backward
//  else if (encVal <= zerothresh) //if negative speed
//  {
//    rawspeed = ((encVal - bRev) / mRev);
//    if (rawspeed < 20)    {
//      rawspeed = 20;
//    }
//    rawspeed = -(rawspeed - 90) / 70;
//  }
//
//  return rawspeed;

}