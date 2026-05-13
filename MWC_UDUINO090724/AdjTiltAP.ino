   void AdjTiltAP()
 {
  float APadjust = 0.5;
int APconstUphill = 10;
int APconstDownhill = 5;

  float LadjAP=0;
  float RadjAP=0;
  ////AP tilt////
  // Uphill
  if (APtilt > 1 && APtilt < 90) {
    int tiltreffactor = APtilt ; //How large of tilt is being experienced
    adjAPtilt[0]  = APconstUphill + int (APadjust * tiltreffactor); //Increase motor speed
    adjAPtilt[1]  = APconstUphill + int (APadjust * tiltreffactor);
  }

  // Downhill
  else if (APtilt < 359 && APtilt > 270) {
    int tiltreffactor = (360 - APtilt) ; //How large of tilt is being experienced
    adjAPtilt[0] = - APconstDownhill - int (APadjust * tiltreffactor); //Decrease motor speed
    adjAPtilt[1] = - APconstDownhill - int (APadjust * tiltreffactor);
  }
  else{
    adjAPtilt[0] = 0;
    adjAPtilt[1] = 0;
  }

 }
