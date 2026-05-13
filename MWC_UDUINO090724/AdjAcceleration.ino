void AdjAcceleration()

{

  ///////////////////////////////////////////
  /////Acceleration adjustments to resistance
  ///////////////////////////////////////////
    float accthreshold = 0.125;
    float jerkthreshold=0.0125;
  float accResistScalar = 1000;  //default600
    const int NumAccCycles = 1;


  /////////////////////////////////////////////////////////////

  float ladjAcc = 0;
  float radjAcc = 0;
  float LAccBuffer[NumAccCycles];
  float RAccBuffer[NumAccCycles];

  // Set buffer of Acc values to be filled with 0s (no motion) at the start
  for (int i = 0; i < NumAccCycles; i++)
  {
    LAccBuffer[i] = 0;
    RAccBuffer[i] = 0;
  }

  //// Calculate Acceleration and smooth with averaging
  for (int i = 1; i < NumAccCycles; i++)
  { LAccBuffer[int(NumAccCycles - i)] = LAccBuffer[int(NumAccCycles - i - 1)];
    RAccBuffer[int(NumAccCycles - i)] = RAccBuffer[int(NumAccCycles - i - 1)];
  }

  LAccFilt.Filter(encBufferLfilt[0] - encBufferLfilt[1]);
  RAccFilt.Filter(encBufferRfilt[0] - encBufferRfilt[1]);
  LAccFiltFeedback.Filter(lspeed- LmotorInput);
  RAccFiltFeedback.Filter(rspeed- RmotorInput);

  LAccBuffer[0] = LAccFilt.Current();
  RAccBuffer[0] = RAccFilt.Current();


  float LAccOld = LAcc;
  float RAccOld = RAcc;
  float Lnum = 0;
  float Rnum = 0;
  for (int i = 0; i < NumAccCycles; i++)
  { Lnum = Lnum + LAccBuffer[i];
    Rnum = Rnum + RAccBuffer[i];
  }
  LAcc = Lnum / NumAccCycles;
  RAcc = Rnum / NumAccCycles;

  Ljerk = LAcc - LAccOld;
  Rjerk = RAcc - RAccOld;


if ( (LAcc < -accthreshold && Ljerk < -jerkthreshold) || ( LAcc > accthreshold && Ljerk > jerkthreshold)  ) {
      ladjAcc = -accResistScalar * Ljerk;
    }

if ( (RAcc < -accthreshold && Rjerk < -jerkthreshold) || ( RAcc > accthreshold && Rjerk > jerkthreshold)  ) {
      radjAcc = -accResistScalar * Rjerk;
    }

//  if (lspeed < 0) {
//    if (LAcc < -accthreshold && Ljerk < -jerkthreshold) {
//      ladjAcc = -accResistScalar * Ljerk;
//    }
//    else if ( LAcc > accthreshold) {
//      ladjAcc = -accResistScalar * Ljerk ;
//    }
//  }
//  else if (lspeed > 0) {
//    if ( LAcc < -accthreshold) {
//      ladjAcc = -accResistScalar * LAcc;
//    }
//    else 
//    if ( LAcc > accthreshold && Ljerk > jerkthreshold) {
//      ladjAcc = -accResistScalar * Ljerk;
//    }
//  }
//
//  if (rspeed < 0) {
//    if (RAcc < -accthreshold && Rjerk < -jerkthreshold) {
//      radjAcc = -accResistScalar * Rjerk;
//    }
//    else if ( RAcc > accthreshold) {
//      radjAcc = -accResistScalar * Rjerk ;
//    }
//  }
//  else if (rspeed > 0) {
//    if ( RAcc < -accthreshold ) {
//      radjAcc = -accResistScalar * RAcc;
//    }
//    else 
//    if ( RAcc > accthreshold && Rjerk > jerkthreshold) {
//      radjAcc = -accResistScalar * Rjerk;
//    }
//  }

  ladjAccFilt.Filter(ladjAcc);
  radjAccFilt.Filter(radjAcc);


  adjAcc[0] = ladjAccFilt.Current();
  adjAcc[1] = radjAccFilt.Current();

}
