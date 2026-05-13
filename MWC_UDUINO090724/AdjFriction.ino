float AdjFriction(float encVal, float motValCurrent, float motValPrevious)
{

float zerothresh = 1.75;
float adjFric=0;
float threshval = 2.5; 
volatile float decline = 0.25;    //amount to reduce the speed by on each cycle if the speed is constant
volatile float nonlineardeclinefactor = 0.05;
    
  ////Forward
  if (encVal >= zerothresh)   // if positive speed
  {
    if (motValCurrent-motValPrevious >  threshval) //increasing speed
    {adjFric=0;
      }
      else
    { //If not accelerating or stopped, keep motor speed, but reduce with every cycle.  Linear and speed dependent terms used to replicate linear and non-linear energy losses of system
      adjFric =  - decline - motValCurrent  * nonlineardeclinefactor;
    }
  }
  ////Backward, same mechanics as forward, but values reversed
  else if (encVal <= -zerothresh) //if negative speed
  {
    if ((motValCurrent-motValPrevious) <  -threshval) //increasing negative speed
    {adjFric=0;
      }
      else{
      adjFric =  + decline - motValCurrent * nonlineardeclinefactor;
    }
  }
return adjFric;

}
