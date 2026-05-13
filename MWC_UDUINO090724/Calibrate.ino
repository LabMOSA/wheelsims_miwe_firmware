
float calibrateMotors()
{ // This is the calibration routine that will run any time the Arduino is powered on.
  // Ramp up to full speed - Only go to 160 (not 180) as the encoders become unreliable over 160 - If we use dedicated encoder reader, this can be adjusted

  int delayramptime = 50;   //ms to pause on each calibration loop to allow vel to stabilize
  int minmotspeed = -125;
  int maxmotspeed = 125;

  for (int powerval = 0; powerval <= maxmotspeed; powerval += 5)
  {
     ST.motor(1, powerval);
    ST.motor(2, powerval);
    delay(delayramptime);
    MotorReportL();
    MotorReportR();
     Serial.print(encL);
    Serial.print('\t');
    Serial.print(encR);
    Serial.print('\t');
    Serial.println(powerval);

    for (int angle = 0; angle <= 3; angle += 1)
    {
      if (powerval == RiFaPower[angle])
      { rising[angle] = encL;
        risingR[angle] = encR;
      }
    }
  }

  // Go through full range of speeds in one direction, recording values for calibration
  for (int powerval = maxmotspeed; powerval >= minmotspeed; powerval -= 5)
  {
    ST.motor(1, powerval);
    ST.motor(2, powerval);
    delay(delayramptime);
    MotorReportL();
    MotorReportR();
     Serial.print(encL);
    Serial.print('\t');
    Serial.print(encR);
    Serial.print('\t');
    Serial.println(powerval);


    for (int angle = 0; angle <= 3; angle += 1)
    {
      if (powerval == RiFaPower[angle])
      { falling[angle] = encL;
        fallingR[angle] = encR;
      }
    }

  }
  // Go through full range of speeds in other direction recording values for calibration
  for (int powerval = minmotspeed; powerval <= 0; powerval += 5)
  {
    ST.motor(1, powerval);
    ST.motor(2, powerval);
    delay(delayramptime);
    MotorReportL();
    MotorReportR();
    Serial.print(encL);
    Serial.print('\t');
    Serial.print(encR);
    Serial.print('\t');
    Serial.println(powerval);

    for (int angle = 0; angle <= 3; angle += 1)
    {
      if (powerval == RiFaPower[angle])
      { rising[angle] = encL;
        risingR[angle] = encR;
      }
    }

  }


  for (int segment = 0; segment < 2; segment ++)
  {
    index = segment * 2;  //0 and then 2
    index2 = index + 1;   //1 and then 3
    // Simple manipulations of y=mx+b to get linear relationships between input speed and encoder values
    risingslope[segment] =  (RiFaPower[index] - RiFaPower[index2])/(rising[index] - rising[index2])  ;
    fallingslope[segment] = (RiFaPower[index] - RiFaPower[index2])/(falling[index] - falling[index2])  ;
//    grandslope = (risingslope[0] + risingslope[1] + fallingslope[0] + fallingslope[1]) / 4;
//    riseB[segment] = rising[index] - (risingslope[segment] * RiFaPower[index]);
//    fallB[segment] = falling[index] - (fallingslope[segment] * RiFaPower[index]);


    // Repeated calculations for the Right motor
    risingslopeR[segment] =  (RiFaPower[index] - RiFaPower[index2])/(risingR[index] - risingR[index2]) ;
    fallingslopeR[segment] = (RiFaPower[index] - RiFaPower[index2])/(fallingR[index] - fallingR[index2])  ;
//    grandslopeR = (risingslopeR[0] + risingslopeR[1] + fallingslopeR[0] + fallingslopeR[1]) / 4;
//    riseBR[segment] = risingR[index] - (risingslopeR[segment] * RiFaPower[index]);
//    fallBR[segment] = fallingR[index] - (fallingslopeR[segment] * RiFaPower[index]);
  }

//Serial.println("Calibration Complete, Values:");
//Serial.print(riseB[1]);
//Serial.print('\t');
//Serial.print(risingslope[1]);
//Serial.print('\t');
//Serial.print(fallB[0]);
//Serial.print('\t');
//Serial.println(fallingslope[0]);
//
//Serial.print(riseBR[1]);
//Serial.print('\t');
//Serial.print(risingslopeR[1]);
//Serial.print('\t');
//Serial.print(fallBR[0]);
//Serial.print('\t');
//Serial.println(fallingslopeR[0]);


}
