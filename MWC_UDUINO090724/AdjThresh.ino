float AdjThreshold(float encVal, float motValCurrent, float motValPrevious)
{

float adjThresh=0;
float threshval = 10.5;    //amplitude beyond the current speed that must be passed in order to be judged as accelerating or decelerating
float extraResistance = 10;//15;
float zerothresh = 1.5;

  ////Forward
  if (encVal >= zerothresh)   // if positive speed
  {
    if (motValCurrent > (motValPrevious + threshval)) //increasing speed
    {
//      adjThresh = -(motValCurrent-motValPrevious) - threshval - extraResistance;
      adjThresh =  - threshval - extraResistance;
    }
    else if (motValCurrent < (motValPrevious - threshval)) //decreasing speed
    {
//      adjThresh = (motValCurrent-motValPrevious)  + threshval + extraResistance;
      adjThresh =   + threshval + extraResistance;
    }
    
  }
  ////Backward, same mechanics as forward, but values reversed
  else if (encVal <= -zerothresh) //if negative speed
  {
    if (motValCurrent < (motValPrevious - threshval)) //increasing negative speed
    {
//      adjThresh = -(motValCurrent-motValPrevious) + threshval + extraResistance;
      adjThresh =  + threshval + extraResistance;
    }
    else if (motValCurrent > (motValPrevious + threshval)) // decreasing negtive speed
    {
//      adjThresh = (motValCurrent-motValPrevious)  - threshval - extraResistance;
      adjThresh =   - threshval - extraResistance;
    }
  }

  return adjThresh;

}
