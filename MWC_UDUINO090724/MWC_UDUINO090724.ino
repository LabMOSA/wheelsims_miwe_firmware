
/*This is code written to replicate the dynamics of a manual wheelchair when used in conjunction with the device developed at McGill University/JRH under the direction of Philippe Archambault
   The code is also meant to communicate with the MiWe software (Wheelchair Simulator or InfiniteRunner) where additional functionality is added to reproduce tilt and contact scenarios from the game.
   The code was written by Sam Leitkam (samuel.leitkam@mcgill.ca or leitkams@gmail.com) and is the intellectual property of Philippe Archambault.
*/

//////////////////////////////

//Load all necessary Libraries

#include <digitalWriteFast.h>
#include <SerialCommand.h>
#include <Filter.h>

#include <SoftwareSerial.h>
#include <Sabertooth.h>
#include<Uduino.h>
Uduino uduino("TreaHunt");

SoftwareSerial SWSerial(NOT_A_PIN, 9); // RX on no pin (unused), TX on pin 11 (to S1).
Sabertooth ST(130, SWSerial); // Address 128, and use SWSerial as the serial port.



//////////////////////////////

// Establish pin connections for Quadrature encoders
// Left encoder
#define c_LeftEncoderInterrupt 1
#define c_LeftEncoderPinA 3
#define c_LeftEncoderPinB 5
#define LeftEncoderIsReversed
volatile bool _LeftEncoderBSet;
volatile long countL = 0;

// Right encoder
#define c_RightEncoderInterrupt 0
#define c_RightEncoderPinA 2
#define c_RightEncoderPinB 4
volatile bool _RightEncoderBSet;
volatile long countR = 0;
//////////////////////////////////

//Initialize all variables and variable types

///Variables to adjust the kinetics of the interaction
//////////////
const int NumCycleDelays = 40;
int MotorLagCycles = 15; // Number of cycles to delay before updating the motor speeds - gives impression of inertia
int counttime = 6000;       //ns that each wheel is calculating speed (25)
//volatile float zerothresh = 1.75; // amplitude around zero that is used to ensure stability around zero

const int NumEncCycles = 3;
float zerospeedthresh = 0.0125;
const bool Calibrate=0;  //// 0 if using hard-coded values for wheel speed calculatio, 1 if running the "Calibrate" subroutine


ExponentialFilter<float> FilteredL(75, 0);// default 25
ExponentialFilter<float> FilteredR(75, 0);

ExponentialFilter<float> LAccFilt(2.5, 0);
ExponentialFilter<float> RAccFilt(2.5, 0);//default2

ExponentialFilter<float> LAccFiltFeedback(20, 0);
ExponentialFilter<float> RAccFiltFeedback(20, 0);

ExponentialFilter<float> ladjAccFilt(25, 0);
ExponentialFilter<float> radjAccFilt(25, 0);

/////////////


volatile float Acc = 0;
volatile float LAcc = 0;
volatile float RAcc = 0;
volatile float Ljerk = 0;
volatile float Rjerk = 0;
volatile float rawoffsetforward = 0;
volatile float rawoffsetreverse = 0;
float lspeed = 0;
float rspeed = 0;
float lspeedAccref = 0;
float rspeedAccref = 0;
float lspeedold[NumCycleDelays];
float rspeedold[NumCycleDelays];
float encBufferL[NumEncCycles];
float encBufferR[NumEncCycles];
float encBufferLfilt[NumEncCycles];
float encBufferRfilt[NumEncCycles];
float encL = 0;
float encR = 0;;
float encLin = 0;
float encRin = 0;
float adjAcc[] = {0, 0};
float adjAPtilt[] = {0, 0};
float adjLRtilt[] = {0, 0};
float adjEqWhl[] = {0, 0};
float adjCont[] = {0, 0};
float adjFric[] = {0, 0};
float adjThresh[] = {0, 0};
float Wtime;

//////Calibration variables
//////////////////////////

volatile float rising[4], falling[4], risingtmp[4], fallingtmp[4];
volatile float risingR[4], fallingR[4], risingtmpR[4], fallingtmpR[4];
volatile float riseB[2], fallB[2], risingslope[2], fallingslope[2];
volatile float riseBR[2], fallBR[2], risingslopeR[2], fallingslopeR[2];
int RiFaPower[] = { -25, -110, 110, 25}; // values to use as the points of estimation of the linear regression for the motor/encoder calibration
float grandslope, grandslopeR;

int index, index2;
int rotdet, rotadj, speedadj;

float bFor, mFor, bRev, mRev;
int motor, encVal, encValcur;
float motspeed, motspeedold;
volatile float rawspeed, LspeedUnity, RspeedUnity;
int LmotorInput = 0;
int RmotorInput = 0;
volatile float APtilt, LRtilt, ContactX, ContactY;
volatile float Mass = 15;
volatile int Contact = 0;



//const int pushcount = 5;
//volatile unsigned int pushtBuffer[pushcount];
//volatile float pushVBuffer[pushcount];
//volatile unsigned long t0, t1, pushtime;
//volatile float v0, v1, pushVel, pushAng;
//volatile float feedbackbuffersum;
//long lasttime;

volatile long  t0L = 0;
volatile long  t1L = 0;
volatile long  tBetweenPushesL = 0;
volatile long  t0Lold = 0;
volatile float v0L = 0;
volatile float v1L = 0;
volatile int   pushDetectedL = 0;
volatile float maxForceL = 0;
volatile float sumWorkL = 0;
volatile float sumVL=0;
volatile float sumAngL=0;
volatile int   sumcyclesL=0;
volatile int   pushtimeL = 0;
volatile float pushVelL = 0;
volatile float pushAngL = 0;
volatile float pushFreqL = 0;
volatile int   pushCountL = 0;
volatile float pushMaxForceL = 0;
volatile float pushPowerL = 0;
volatile long  tlastL=0;

volatile long  t0R = 0;
volatile long  t1R = 0;
volatile long  tBetweenPushesR = 0;
volatile long  t0Rold = 0;
volatile float v0R = 0;
volatile float v1R = 0;
volatile int   pushDetectedR = 0;
volatile float maxForceR = 0;
volatile float sumWorkR = 0;
volatile float sumVR=0;
volatile float sumAngR=0;
volatile int   sumcyclesR=0;
volatile int   pushtimeR = 0;
volatile float pushVelR = 0;
volatile float pushAngR = 0;
volatile float pushFreqR = 0;
volatile int   pushCountR = 0;
volatile float pushMaxForceR = 0;
volatile float pushPowerR = 0;
volatile long  tlastR=0;

volatile long  t0B = 0;
volatile long  t1B = 0;
volatile long  tBetweenPushesB = 0;
volatile long  t0Bold = 0;
volatile float v0B = 0;
volatile float v1B = 0;
volatile int   pushDetectedB = 0;
volatile float maxForceB = 0;
volatile float sumWorkB = 0;
volatile float sumVB=0;
volatile float sumAngB=0;
volatile int   sumcyclesB=0;
volatile int   pushtimeB = 0;
volatile float pushVelB = 0;
volatile float pushAngB = 0;
volatile float pushFreqB = 0;
volatile int   pushCountB = 0;
volatile float pushMaxForceB = 0;
volatile float pushPowerB = 0;
volatile long  tlastB=0;

const int feedbackcycles = 2;
bool feedbackBufferL[feedbackcycles];
bool feedbackBufferR[feedbackcycles];
bool feedbackBufferB[feedbackcycles];
int forceBufferL[feedbackcycles];
int forceBufferR[feedbackcycles];
int forceBufferB[feedbackcycles];
float cont_temp;

// Initialization code to establish communication with motors, encoders, and Unity
void setup()
{
  SWSerial.begin(19200);

  // Quadrature encoders
  // Left encoder
  pinMode(c_LeftEncoderPinA, INPUT_PULLUP);      // sets pin A as input
  pinMode(c_LeftEncoderPinB, INPUT_PULLUP);      // sets pin B as input

  // Right encoder
  pinMode(c_RightEncoderPinA, INPUT_PULLUP);      // sets pin A as input
  pinMode(c_RightEncoderPinB, INPUT_PULLUP);      // sets pin B as input

  Serial.begin(250000); //Communication rate with Unity
  
  // Set buffer of speed values to be filled with 90s (no motion) at the start
  for (int i = 0; i < NumCycleDelays; i++)
  {
    lspeedold[i] = 0;
    rspeedold[i] = 0;
  }

  // Set buffer of Enc values to be filled with 0s (no motion) at the start
  for (int i = 0; i < NumEncCycles; i++)
  {
    encBufferL[i] = 0;
    encBufferR[i] = 0;
    encBufferLfilt[i] = 0;
    encBufferRfilt[i] = 0;
  }

  // Set buffer of feedback values to be filled with 0s (no push detected) at the start
  for (int i = 0; i < feedbackcycles; i++)
  {
    feedbackBufferL[i] = 0;
    feedbackBufferR[i] = 0;
    feedbackBufferB[i] = 0;
    forceBufferL[i] = 0;
    forceBufferR[i] = 0;
    forceBufferB[i] = 0;
  }


  //Run Calibration routine at the outset of program
  if (Calibrate==1){
 //calibrateMotors();
  }

  // Establish communication scripts with Unity
  uduino.addCommand("UPDATEUA", updateUnity);
  //uduino.addCommand("TreasureRecievedData", TreasureData);
  uduino.addCommand("TrRcvDta", TrRcvDta);

\

  //uduino.addCommand("UPDATEUA", updateArduino);

  delay(500);
}




// Data received from Unity into Arduino (serial form, 5 pieces of data)
void TrRcvDta() {
  char *arg;

  arg = uduino.next();
  LRtilt = atof(arg);

  arg = uduino.next();
  APtilt = atof(arg);

  arg = uduino.next();
 // cont_temp = atoi(arg);
  //Contact= int(trunc(cont_temp));
Contact=  atoi(arg);

  arg = uduino.next();
  ContactX = atof(arg);

  arg = uduino.next();
  ContactY = atof(arg);

  arg = uduino.next();
  Mass = atof(arg);
}
/////////////////////////////////////


// Data sent from Arduino to Unity
void updateUnity() {
  
  
  Serial.print(LspeedUnity);  // 0
  
  Serial.print('\t');
  Serial.print(RspeedUnity);// 1

 
  
  Serial.print('\t');
  Serial.print(pushCountB);  //positive integer, counting up from 0 //2
  Serial.print('\t');
  Serial.print(pushAngB);  //positive float //3 <==
  Serial.print('\t');
  Serial.print(pushFreqB);   //positive //4 <==
  Serial.print('\t');
  Serial.print(pushVelB);    //positive float//5
  Serial.print('\t');
  Serial.print(pushtimeB);   //positive integer, in ms//6
  Serial.print('\t');
  Serial.print(pushPowerB);//positive float//7
  
  Serial.print('\t');
  Serial.print(pushCountL);  //positive integer, counting up from 0//8
  Serial.print('\t');
  Serial.print(pushAngL);  //positive float//9 <==
  Serial.print('\t');
  Serial.print(pushFreqL);   //positive float//10
  Serial.print('\t');
  Serial.print(pushVelL);    //positive float//11
  Serial.print('\t');
  Serial.print(pushtimeL);   //positive integer, in ms//12
  Serial.print('\t');
  Serial.print(pushPowerL);//positive float//13
  
  Serial.print('\t');
  Serial.print(pushCountR);  //positive integer, counting up from 0
  Serial.print('\t');
  Serial.print(pushAngR);  //positive float  <== 15
  Serial.print('\t');
  Serial.print(pushFreqR);   //positive float
  Serial.print('\t');
  Serial.print(pushVelR);    //positive float
  Serial.print('\t');
  Serial.print(pushtimeR);   //positive integer, in ms
  Serial.print('\t');
  Serial.println(pushPowerR);//positive float

  //  Serial.print(PushIndex);  //positive integer, counting up from 0
  //  Serial.print('\t');
  //  Serial.print(PushAngle);  //positive float
  //  Serial.print('\t');
  //  Serial.print(PushFreq);   //positive float
  //  Serial.print('\t');
  //  Serial.print(PushVel);    //positive float
  //  Serial.print('\t');
  //  Serial.print(PushTime);   //positive integer, in ms
  //  Serial.print('\t');
  //  Serial.println(PushForce);//positive float

}
///////////////////////////////////
///////////////////////////////////


//Main program loop
void loop()
{

  //  Serial.println(micros()-lasttime);
  //  lasttime=micros();
  //uduino.update();
  updateUnity();
  //delay(20);

  //Get Encoder values
  encL = MotorReportL();
  encR = MotorReportR();

/*
  if (abs(encL) < 3) {
    encL = 0;
  }

  if (abs(encR) < 3) {
       encR= 0;
  }
*/
  FilteredL.Filter(encL);
  FilteredR.Filter(encR);

  //Store encoder values in a buffer by shifting each one one place, and calculate the running total of those values
  for (int i = 1; i < NumEncCycles; i++)
  { encBufferLfilt[int(NumEncCycles - i)] = encBufferLfilt[int(NumEncCycles - i - 1)];
    encBufferRfilt[int(NumEncCycles - i)] = encBufferRfilt[int(NumEncCycles - i - 1)];
  }
  //Store the newest value in the buffer
  encBufferLfilt[0] = FilteredL.Current();
  encBufferRfilt[0] = FilteredR.Current();

  //Convert Encoder values to motor speed values
  lspeed = ConvertEncoder2Speed(FilteredL.Current(), 0, lspeedold[0]);
  rspeed = ConvertEncoder2Speed(FilteredR.Current(), 1, rspeedold[0]);
  //// Store speed values in buffers
  for (int i = 1; i < NumCycleDelays; i++)
  { lspeedold[int(NumCycleDelays - i)] = lspeedold[int(NumCycleDelays - i - 1)];
    rspeedold[int(NumCycleDelays - i)] = rspeedold[int(NumCycleDelays - i - 1)];
  }

  //  lspeedold[0] = lspeed ;
  //  rspeedold[0] = rspeed ;

  ConvertEncoder2UnityVals();

  /////////////////////////////////
  adjFric[0] = AdjFriction(encBufferLfilt[0], encBufferLfilt[0], encBufferLfilt[1]);
  adjFric[1] = AdjFriction(encBufferRfilt[0], encBufferRfilt[0], encBufferRfilt[1]);
  adjThresh[0] = AdjThreshold(encBufferLfilt[0], encBufferLfilt[0], encBufferLfilt[1]);
  adjThresh[1] = AdjThreshold(encBufferRfilt[0], encBufferRfilt[0], encBufferRfilt[1]);
  AdjWheelEq();
  AdjTiltAP();
  AdjTiltLR();
  CalcFeedback();

  lspeedold[0] = lspeed + adjFric[0] + adjEqWhl[0] + adjAPtilt[0] + adjLRtilt[0] ;
  rspeedold[0] = rspeed + adjFric[1] + adjEqWhl[1] + adjAPtilt[1] + adjLRtilt[1] ;

  AdjAcceleration();

  LmotorInput = (lspeedold[MotorLagCycles] + lspeedold[MotorLagCycles - 1]) / 2  + adjThresh[0]+ adjAcc[0];
  RmotorInput = (rspeedold[MotorLagCycles] + rspeedold[MotorLagCycles - 1]) / 2  + adjThresh[1]+ adjAcc[1]  ;

  //LmotorInput =  lspeedold[MotorLagCycles]  ;
  //RmotorInput =  rspeedold[MotorLagCycles] ;

//        Serial.print(LAccFilt.Current()*100);
//        Serial.print('\t');
//        Serial.print(LAccFiltFeedback.Current());
//        Serial.print('\t');
//        Serial.print(RAcc*100);
  //      Serial.print('\t');
  //      Serial.print(adjAcc[0]);
  //      Serial.print('\t');
  //      Serial.print(adjAcc[1]);
  //      Serial.print('\t');
  //      Serial.print(FilteredL.Current());
  //      Serial.print('\t');
  //      Serial.println(FilteredR.Current());

//    Serial.print(LspeedUnity*10);
//    Serial.print('\t');
//    Serial.println(RspeedUnity*10);
//    Serial.print('\t');
//  Serial.print(pushCountB);  //positive integer, counting up from 0
//  Serial.print('\t');
//  Serial.println(pushAngB);  //positive float
//  Serial.print('\t');
//  Serial.print(pushFreqB);   //positive float
//  Serial.print('\t');
//  Serial.print(pushVelB);    //positive float
//  Serial.print('\t');
//  Serial.print(pushtimeB);   //positive integer, in ms
//  Serial.print('\t');
//  Serial.print( pushMaxForceB);   //positive integer, in ms
//  Serial.print('\t');
//  Serial.println( pushPowerB);   //positive integer, in ms


  if (Contact == 1) {
    //If contact is detected, replace the speed buffers with new information
    //The values will make the wheels reverse from the direction of the contact, then go to 0 velocity
    AdjContact();
    LmotorInput = adjCont[0];
    RmotorInput = adjCont[1];
    for (int i = 0; i < MotorLagCycles; i++)
    {
      
      lspeedold[i] = 0;
      rspeedold[i] = 0;
    }
  }
  ///////////////////////////////////////////////////////

  ////////////////////////////////////////////////////////
  ////  ////Write values to the Motors
  //   MotLeft.write(LmotorInput);
  //  MotRight.write(RmotorInput);

  ST.motor(1, RmotorInput);
  ST.motor(2, LmotorInput);
  //
  // int motorInput=0;
  //////  MotLeft.write(motorInput);
  //////  MotRight.write(motorInput);
  //  ST.motor(1, motorInput);
  ////delayMicroseconds(1000);
  //    ST.motor(2, motorInput);

  //int motorInput=1500;
  //  MotLeft.writeMicroseconds(motorInput);
  //  MotRight.writeMicroseconds(motorInput);

  //  //Save this cycles values for comparison on next cycle
  //  int LmotorOld = LmotorInput;
  //  int RmotorOld = RmotorInput;

  //Check to see if Unity has sent any new data before next cycle
//  while (Serial.available() > 0) {
//    sCmd.readSerial();
//  }
}
////////////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////
