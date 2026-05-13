void AdjTiltLR()
{
int UpsideWheelAdjust = 5;
int DownsideWheelAdjust = 2;

  float LadjLR = 0;
  float RadjLR = 0;

  //// Tilted Right
  if (LRtilt > 1 && LRtilt < 45) {
    if (lspeed > 0 && rspeed > 0) {
      adjLRtilt[0] = + UpsideWheelAdjust;
      adjLRtilt[1] = - DownsideWheelAdjust;
    }
    if (lspeed < 0 && rspeed < 0) {
      adjLRtilt[0] = - UpsideWheelAdjust;
      adjLRtilt[1] = + DownsideWheelAdjust;
    }
  }

  //// Tilted Left
  else if (LRtilt < 359 && LRtilt > 315) {
    if (lspeed > 0 && rspeed > 0) {
      adjLRtilt[0] = - DownsideWheelAdjust;
      adjLRtilt[1] = + UpsideWheelAdjust;
    }
    if (lspeed < 0 && rspeed < 0) {
      adjLRtilt[0] = + DownsideWheelAdjust;
      adjLRtilt[1] = - UpsideWheelAdjust;
    }

  }
  else{
    adjLRtilt[0] = 0;
      adjLRtilt[1] = 0;
  }

}
