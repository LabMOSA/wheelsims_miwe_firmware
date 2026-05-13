void CalcFeedback()
{
  float accthresholdF = 4; // Threshold of acceleration to detect a push
  float zerothreshF = 1;    // Threshold of velocity to detect a push (neglect small jumps when wheel is stopped)

// y=slope*x+offset constants for the Left wheel
  float pushVelLslope = 0.0567;
  float pushVelLoffset = 0.1294;  
  float pushAngLslope = 3.1091;
  float pushAngLoffset = 18.887;  
  float pushMaxForceLslope = 1;
  float pushMaxForceLoffset = 0;  
  float pushPowerLslope = 1;
  float pushPowerLoffset = 0;
// y=slope*x+offset constants for the Right wheel
  float pushVelRslope = 0.0567;
  float pushVelRoffset = 0.1294;  
  float pushAngRslope = 3.1091;
  float pushAngRoffset = 18.887;  
  float pushMaxForceRslope = 1;
  float pushMaxForceRoffset = 0;  
  float pushPowerRslope = 1;
  float pushPowerRoffset = 0;
// y=slope*x+offset constants for the combined wheel calculations
  float pushVelBslope = 0.0567;
  float pushVelBoffset = 0.1294;  
  float pushAngBslope = 3.1091;
  float pushAngBoffset = 18.887;  
  float pushMaxForceBslope = 1;
  float pushMaxForceBoffset = 0;  
  float pushPowerBslope = 1;
  float pushPowerBoffset = 0;

//// Standardized values commented here for debugging purposes, All slopes =1, all offsets=0
//  // y=slope*x+offset constants for the Left wheel
//  float pushVelLslope = 1;
//  float pushVelLoffset = 0;  
//  float pushAngLslope = 1;
//  float pushAngLoffset = 0;  
//  float pushMaxForceLslope = 1;
//  float pushMaxForceLoffset = 0;  
//  float pushPowerLslope = 1;
//  float pushPowerLoffset = 0;
//// y=slope*x+offset constants for the Right wheel
//  float pushVelRslope = 1;
//  float pushVelRoffset = 0;  
//  float pushAngRslope = 1;
//  float pushAngRoffset = 0;  
//  float pushMaxForceRslope = 1;
//  float pushMaxForceRoffset = 0;  
//  float pushPowerRslope = 1;
//  float pushPowerRoffset = 0;
//// y=slope*x+offset constants for the combined wheel calculations
//  float pushVelBslope = 1;
//  float pushVelBoffset = 0;  
//  float pushAngBslope = 1;
//  float pushAngBoffset = 0;  
//  float pushMaxForceBslope = 1;
//  float pushMaxForceBoffset = 0;  
//  float pushPowerBslope = 1;
//  float pushPowerBoffset = 0;
  



  /////////Things to calculate: push time, push velocity, push angle, push frequency, power output, peak force during push
  //// Store speed values in buffers
  for (int i = 1; i < feedbackcycles; i++) {
    feedbackBufferL[int(feedbackcycles - i)] = feedbackBufferL[int(feedbackcycles - i - 1)];
    feedbackBufferR[int(feedbackcycles - i)] = feedbackBufferR[int(feedbackcycles - i - 1)];
    feedbackBufferB[int(feedbackcycles - i)] = feedbackBufferB[int(feedbackcycles - i - 1)];
    forceBufferL[int(feedbackcycles - i)] = forceBufferL[int(feedbackcycles - i - 1)];
    forceBufferR[int(feedbackcycles - i)] = forceBufferR[int(feedbackcycles - i - 1)];
    forceBufferB[int(feedbackcycles - i)] = forceBufferB[int(feedbackcycles - i - 1)];
  }

  ////Detect Pushes
  /////////////////////////////////
  /// We are filling a buffer of 0's and 1's to represent whether a push is being detected at the time as well as storing the values while the push is being detecte
  if (abs(lspeed) > zerothreshF && LAccFiltFeedback.Current() < -accthresholdF ) {
    feedbackBufferL[0] = 1;
    forceBufferL[0] = LmotorInput - lspeed;
  }
  else {
    feedbackBufferL[0] = 0;
    forceBufferL[0] = 0;
  }

  if (abs(rspeed) > zerothreshF && RAccFiltFeedback.Current() < -accthresholdF ) {
    feedbackBufferR[0] = 1;
    forceBufferR[0] = RmotorInput - rspeed;
  }
  else {
    feedbackBufferR[0] = 0;
    forceBufferR[0] = 0;
  }

  if (feedbackBufferL[0] == 1 && feedbackBufferR[0] == 1 ) {
    feedbackBufferB[0] = 1;
    forceBufferB[0] = (forceBufferL[0] + forceBufferR[0]) / 2;
  }
  else {
    feedbackBufferB[0] = 0;
    forceBufferB[0] = 0;
  }


  //////Check Values at initiation of a push
  /////////////////////////////////////////
  if (feedbackBufferL[0] - feedbackBufferL[1] > 0) { // value goes from 0 to 1 (not accelerating to acclerating)
    t0L = millis();
    v0L = lspeed ;
    pushDetectedL = 1;
    long tlastL = t0L;
  }
  if (feedbackBufferR[0] - feedbackBufferR[1] > 0) { // value goes from 0 to 1 (not accelerating to acclerating)
    t0R = millis();
    v0R = rspeed ;
    pushDetectedR = 1;
    long tlastR = t0R;
  }
  if (feedbackBufferB[0] - feedbackBufferB[1] > 0) { // value goes from 0 to 1 (not accelerating to acclerating)
    t0B = millis();
    v0B = (lspeed + rspeed) / 2;
    pushDetectedB = 1;
    long tlastB = t0B;
  }

  ///////If continuing to push, count up the values, and get the max force of the push
  ////////////////////////////////////////////////////////////////////////////////////
  if (pushDetectedL == 1) {
    if (forceBufferL[0] > maxForceL) {
      maxForceL = forceBufferL[0];
    }
    sumWorkL += forceBufferL[0]*1000 * float(lspeed / (micros() - tlastL));
    sumAngL += 1000 * float(lspeed / (micros() - tlastL));
    sumVL += lspeed;
    sumcyclesL += 1;
    tlastL = micros();
  }
  if (pushDetectedR == 1) {
    if (forceBufferR[0] > maxForceR) {
      maxForceR = forceBufferR[0];
    }
    sumWorkR += forceBufferR[0]*1000 * float(rspeed / (micros() - tlastR));
    sumAngR += 1000 * float(rspeed / (micros() - tlastR));
    sumVR += rspeed;
    sumcyclesR += 1;
    tlastR = micros();
  }
  if (pushDetectedB == 1) {
    if (forceBufferB[0] > maxForceB) {
      maxForceB = forceBufferB[0];
    }
    sumWorkB += forceBufferB[0]*1000 * float(((lspeed + rspeed) / 2) / (micros() - tlastB));
    sumAngB += 1000 * float(((lspeed + rspeed) / 2) / (micros() - tlastB));
    sumVB += (lspeed + rspeed) / 2;
    sumcyclesB += 1;
    tlastB = micros();
  }

  //////Check values at the completion of a push
  ////////Also, calculate difference between start and end
  //////////////////////////////////////////////
  if (feedbackBufferL[0] - feedbackBufferL[1] < 0) { // value goes from 1 to 0 (no longer acclerating)
    t1L = millis();
    v1L = lspeed ;
    float tBetweenPushesL = float(t0L - t0Lold);
    pushDetectedL = 0;

    if (t1L - t0L > 200) {
      pushtimeL = t1L - t0L;
      pushVelL = pushVelLslope*(-sumVL / sumcyclesL)+pushVelLoffset;
      pushAngL = pushAngLslope*(-sumAngL)+pushAngLoffset;
      pushFreqL = 1000 / tBetweenPushesL;
      pushCountL += 1;
      pushMaxForceL = pushMaxForceLslope*maxForceL+pushMaxForceLoffset;
      pushPowerL = pushPowerLslope*sumWorkL/pushtimeL+pushPowerLoffset;

      t0Lold = t0L;
      maxForceL = 0;
      sumWorkL = 0;
      sumVL = 0;
      sumAngL = 0;
      sumcyclesL = 0;
    }
  }

  if (feedbackBufferR[0] - feedbackBufferR[1] < 0) { // value goes from 1 to 0 (no longer acclerating)
    t1R = millis();
    v1R = rspeed ;
    float tBetweenPushesR = float(t0R - t0Rold);
    pushDetectedR = 0;

    if (t1R - t0R > 200) {
      pushtimeR = t1R - t0R;
      pushVelR = pushVelRslope*(-sumVR / sumcyclesR)+pushVelRoffset;
      pushAngR = pushAngRslope*(-sumAngR)+pushAngRoffset;
      pushFreqR = 1000 / tBetweenPushesR;
      pushCountR += 1;
      pushMaxForceR = pushMaxForceRslope*maxForceR+pushMaxForceRoffset;
      pushPowerR = pushPowerRslope*sumWorkR/pushtimeR+pushPowerRoffset;

      t0Rold = t0R;
      maxForceR = 0;
      sumWorkR = 0;
      sumVR = 0;
      sumAngR = 0;
      sumcyclesR = 0;
    }
  }

  if (feedbackBufferB[0] - feedbackBufferB[1] < 0) { // value goes from 1 to 0 (no longer acclerating)
    t1B = millis();
    v1B = (lspeed + rspeed) / 2;
    float tBetweenPushesB = float(t0B - t0Bold);
    pushDetectedB = 0;

    if (t1B - t0B > 200) {
      pushtimeB = t1B - t0B;
      pushVelB = pushVelBslope*(-sumVB / sumcyclesB)+pushVelBoffset;
      pushAngB = pushAngBslope*(-sumAngB)+pushAngBoffset;
      pushFreqB = 1000 / tBetweenPushesB;
      pushCountB += 1;
      pushMaxForceB = pushMaxForceBslope*maxForceB+pushMaxForceBoffset;
      pushPowerB = pushPowerBslope*sumWorkB/pushtimeB+pushPowerBoffset;

      t0Bold = t0B;
      maxForceB = 0;
      sumWorkB = 0;
      sumVB = 0;
      sumAngB = 0;
      sumcyclesB = 0;
    }
  }


}
