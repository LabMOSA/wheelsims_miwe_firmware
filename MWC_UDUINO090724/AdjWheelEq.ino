void AdjWheelEq()
{
  ////////////////////////////////////////////
  //  // Equate wheelspeeds over time

  float speedadjdenom = 1.1;
  float rotadjdenom = 1.1;

  float ladj = 0;
  float radj = 0;

  rotdet = (((lspeed ) * (rspeed )) >= 0); //if both values positive or one is zero, rotdet should equal 1, otherwise (rotating in diff directions) rotdet=0
  speedadj = float(abs(lspeed - rspeed)) / float(speedadjdenom);
  rotadj = float(abs(lspeed - rspeed)) / float(rotadjdenom);
  ///Rolling Forward or Backward
  if (rotdet == 1) {  //if rotation in the same direction
    if (lspeed > rspeed) {
      adjEqWhl[0] = - speedadj;
      adjEqWhl[1] = + speedadj;
    }
    else if (lspeed < rspeed) {
      adjEqWhl[0] = + speedadj;
      adjEqWhl[1] = - speedadj;
    }
    else
    {
      adjEqWhl[0] = 0;
      adjEqWhl[1] = 0;
    }
  }

  else {
    // Rotating left or Right
    if (lspeed > rspeed) {
      adjEqWhl[0] =  - rotadj;
      adjEqWhl[1] =  + rotadj;
    }
    else if (lspeed < rspeed) {
      adjEqWhl[0] =  + rotadj;
      adjEqWhl[1] =  - rotadj;
    }
    else
    {
      adjEqWhl[0] = 0;
      adjEqWhl[1] = 0;
    }
  }
}
