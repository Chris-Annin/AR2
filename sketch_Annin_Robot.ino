/*  AR2 - Stepper motor robot control software Ver 1.3
    Copyright (c) 2017, Chris Annin
    All rights reserved.

    You are free to share, copy and redistribute in any medium
    or format.  You are free to remix, transform and build upon
    this material.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

          Redistributions of source code must retain the above copyright
          notice, this list of conditions and the following disclaimer.
          Redistribution of this software in source or binary forms shall be free
          of all charges or fees to the recipient of this software.
          Redistributions in binary form must reproduce the above copyright
          notice, this list of conditions and the following disclaimer in the
          documentation and/or other materials provided with the distribution.
          Neither the name of Chris Annin nor the
          names of its contributors may be used to endorse or promote products
          derived from this software without specific prior written permission.
          you must give appropriate credit and indicate if changes were made. You may do
          so in any reasonable manner, but not in any way that suggests the
          licensor endorses you or your use.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
    ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
    WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
    DISCLAIMED. IN NO EVENT SHALL CHRIS ANNIN BE LIABLE FOR ANY
    DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
    (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
    ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
    (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
    SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

    chris.annin@gmail.com

    Log:
    Ver 1.1 - new timing algorithm
    Ver 1.2 - fix calibration timing
    Ver 1.3 - base speed to 200 increase step delay to 10
*/

// SPEED // millisecond multiplier // raise value to slow robot speeds
const int SpeedMult = 200;

// MOTOR DIRECTION // if using DM542T driver(CW) set to 0 // if using ST6600 or DM320T driver(CCW) set to 1
const int J1rotdir = 0;
const int J2rotdir = 0;
const int J3rotdir = 0;
const int J4rotdir = 1;
const int J5rotdir = 0;
const int J6rotdir = 0;

// DIRECTION TO RUN MOTORS FOR CALIBRATION // default for AR2 robot = 0,0,1,0,0,1
////// if building custom robot and your limit switch is flipped or located on different side of arm / direction this allows
////// you to change direction for auto cal - but you must change your joint variable in the AR2.py program to match.
const int J1caldir = 0;
const int J2caldir = 0;
const int J3caldir = 1;
const int J4caldir = 0;
const int J5caldir = 0;
const int J6caldir = 1;

#include <Servo.h>

Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;
Servo servo7;

String inData;
String function;

const int J1stepPin = 2;
const int J1dirPin = 3;
const int J2stepPin = 4;
const int J2dirPin = 5;
const int J3stepPin = 6;
const int J3dirPin = 7;
const int J4stepPin = 8;
const int J4dirPin = 9;
const int J5stepPin = 10;
const int J5dirPin = 11;
const int J6stepPin = 12;
const int J6dirPin = 13;

const int J1calPin = 14;
const int J2calPin = 15;
const int J3calPin = 16;
const int J4calPin = 17;
const int J5calPin = 18;
const int J6calPin = 19;

const int Input22 = 22;
const int Input23 = 23;
const int Input24 = 24;
const int Input25 = 25;
const int Input26 = 26;
const int Input27 = 27;
const int Input28 = 28;
const int Input29 = 29;
const int Input30 = 30;
const int Input31 = 31;
const int Input32 = 32;
const int Input33 = 33;
const int Input34 = 34;
const int Input35 = 35;
const int Input36 = 36;
const int Input37 = 37;

const int Output38 = 38;
const int Output39 = 39;
const int Output40 = 40;
const int Output41 = 41;
const int Output42 = 42;
const int Output43 = 43;
const int Output44 = 44;
const int Output45 = 45;
const int Output46 = 46;
const int Output47 = 47;
const int Output48 = 48;
const int Output49 = 49;
const int Output50 = 50;
const int Output51 = 51;
const int Output52 = 52;
const int Output53 = 53;



void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);

  pinMode(A0, OUTPUT);
  pinMode(A1, OUTPUT);
  pinMode(A2, OUTPUT);
  pinMode(A3, OUTPUT);
  pinMode(A4, OUTPUT);
  pinMode(A5, OUTPUT);
  pinMode(A6, OUTPUT);
  pinMode(A7, OUTPUT);

  pinMode(J1stepPin, OUTPUT);
  pinMode(J1dirPin, OUTPUT);
  pinMode(J2stepPin, OUTPUT);
  pinMode(J2dirPin, OUTPUT);
  pinMode(J3stepPin, OUTPUT);
  pinMode(J3dirPin, OUTPUT);
  pinMode(J4stepPin, OUTPUT);
  pinMode(J4dirPin, OUTPUT);
  pinMode(J5stepPin, OUTPUT);
  pinMode(J5dirPin, OUTPUT);
  pinMode(J6stepPin, OUTPUT);
  pinMode(J6dirPin, OUTPUT);

  pinMode(J1calPin, INPUT);
  pinMode(J2calPin, INPUT);
  pinMode(J3calPin, INPUT);
  pinMode(J4calPin, INPUT);
  pinMode(J5calPin, INPUT);
  pinMode(J6calPin, INPUT);

  pinMode(Input22, INPUT);
  pinMode(Input23, INPUT);
  pinMode(Input24, INPUT);
  pinMode(Input25, INPUT);
  pinMode(Input26, INPUT);
  pinMode(Input27, INPUT);
  pinMode(Input28, INPUT);
  pinMode(Input29, INPUT);
  pinMode(Input30, INPUT);
  pinMode(Input31, INPUT);
  pinMode(Input32, INPUT);
  pinMode(Input33, INPUT);
  pinMode(Input34, INPUT);
  pinMode(Input35, INPUT);
  pinMode(Input36, INPUT);
  pinMode(Input37, INPUT);

  pinMode(Output38, OUTPUT);
  pinMode(Output39, OUTPUT);
  pinMode(Output40, OUTPUT);
  pinMode(Output41, OUTPUT);
  pinMode(Output42, OUTPUT);
  pinMode(Output43, OUTPUT);
  pinMode(Output44, OUTPUT);
  pinMode(Output45, OUTPUT);
  pinMode(Output46, OUTPUT);
  pinMode(Output47, OUTPUT);
  pinMode(Output48, OUTPUT);
  pinMode(Output49, OUTPUT);
  pinMode(Output50, OUTPUT);
  pinMode(Output51, OUTPUT);
  pinMode(Output52, OUTPUT);
  pinMode(Output53, OUTPUT);

  servo0.attach(A0);
  servo1.attach(A1);
  servo2.attach(A2);
  servo3.attach(A3);
  servo4.attach(A4);
  servo5.attach(A5);
  servo6.attach(A6);
  servo7.attach(A7);

  digitalWrite(Output38, HIGH);
  digitalWrite(Output39, HIGH);
  digitalWrite(Output40, HIGH);
  digitalWrite(Output41, HIGH);
  digitalWrite(Output42, HIGH);
  digitalWrite(Output43, HIGH);
  digitalWrite(Output44, HIGH);
  digitalWrite(Output45, HIGH);

  digitalWrite(J1stepPin, HIGH);
  digitalWrite(J2stepPin, HIGH);
  digitalWrite(J3stepPin, HIGH);
  digitalWrite(J4stepPin, HIGH);
  digitalWrite(J5stepPin, HIGH);
  digitalWrite(J6stepPin, HIGH);

}


void loop() {

  //test led
  if (digitalRead(J1calPin) == HIGH || digitalRead(J2calPin) == HIGH || digitalRead(J3calPin) == HIGH || digitalRead(J4calPin) == HIGH || digitalRead(J5calPin) == HIGH || digitalRead(J6calPin) == HIGH)
  {
    digitalWrite(J6dirPin, HIGH);
  }
  else
  {
    digitalWrite(J6dirPin, LOW);
  }




  while (Serial.available() > 0)
  {
    char recieved = Serial.read();
    inData += recieved;
    // Process message when new line character is recieved
    if (recieved == '\n')
    {
      String function = inData.substring(0, 2);


      //-----COMMAND TO MOVE SERVO---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "SV")
      {
        int SVstart = inData.indexOf('V');
        int POSstart = inData.indexOf('P');
        int servoNum = inData.substring(SVstart + 1, POSstart).toInt();
        int servoPOS = inData.substring(POSstart + 1).toInt();
        if (servoNum == 0)
        {
          servo0.write(servoPOS);
        }
        if (servoNum == 1)
        {
          servo1.write(servoPOS);
        }
        if (servoNum == 2)
        {
          servo2.write(servoPOS);
        }
        if (servoNum == 3)
        {
          servo3.write(servoPOS);
        }
        if (servoNum == 4)
        {
          servo4.write(servoPOS);
        }
        if (servoNum == 5)
        {
          servo5.write(servoPOS);
        }
        if (servoNum == 6)
        {
          servo6.write(servoPOS);
        }
        if (servoNum == 7)
        {
          servo7.write(servoPOS);
        }
        Serial.print("Servo Done");
      }




      //-----COMMAND TO WAIT TIME---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "WT")
      {
        int WTstart = inData.indexOf('S');
        float WaitTime = inData.substring(WTstart + 1).toFloat();
        int WaitTimeMS = WaitTime * 1000;
        delay(WaitTimeMS);
        Serial.print("Done");
      }

      //-----COMMAND IF INPUT THEN JUMP---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "JF")
      {
        int IJstart = inData.indexOf('X');
        int IJTabstart = inData.indexOf('T');
        int IJInputNum = inData.substring(IJstart + 1, IJTabstart).toInt();
        if (digitalRead(IJInputNum) == HIGH)
        {
          Serial.println("True\n");
        }
        if (digitalRead(IJInputNum) == LOW)
        {
          Serial.println("False\n");
        }
      }
      //-----COMMAND SET OUTPUT ON---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "ON")
      {
        int ONstart = inData.indexOf('X');
        int outputNum = inData.substring(ONstart + 1).toInt();
        digitalWrite(outputNum, HIGH);
        Serial.print("Done");
      }
      //-----COMMAND SET OUTPUT OFF---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "OF")
      {
        int ONstart = inData.indexOf('X');
        int outputNum = inData.substring(ONstart + 1).toInt();
        digitalWrite(outputNum, LOW);
        Serial.print("Done");
      }
      //-----COMMAND TO WAIT INPUT ON---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "WI")
      {
        int WIstart = inData.indexOf('N');
        int InputNum = inData.substring(WIstart + 1).toInt();
        while (digitalRead(InputNum) == LOW) {
          delay(100);
        }
        Serial.print("Done");
      }
      //-----COMMAND TO WAIT INPUT OFF---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "WO")
      {
        int WIstart = inData.indexOf('N');
        int InputNum = inData.substring(WIstart + 1).toInt();

        //String InputStr =  String("Input" + InputNum);
        //uint8_t Input = atoi(InputStr.c_str ());
        while (digitalRead(InputNum) == HIGH) {
          delay(100);
        }
        Serial.print("Done");
      }

      //-----COMMAND TO CALIBRATE---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "LL")
      {
        int J1start = inData.indexOf('A');
        int J2start = inData.indexOf('B');
        int J3start = inData.indexOf('C');
        int J4start = inData.indexOf('D');
        int J5start = inData.indexOf('E');
        int J6start = inData.indexOf('F');
        int J1step = (inData.substring(J1start + 1, J2start).toInt()) + 200;
        int J2step = (inData.substring(J2start + 1, J3start).toInt()) + 200;
        int J3step = (inData.substring(J3start + 1, J4start).toInt()) + 200;
        int J4step = (inData.substring(J4start + 1, J5start).toInt()) + 200;
        int J5step = (inData.substring(J5start + 1, J6start).toInt()) + 200;
        int J6step = (inData.substring(J6start + 1).toInt()) + 200;

        //RESET COUNTERS
        int J1done = 0;
        int J2done = 0;
        int J3done = 0;
        int J4done = 0;
        int J5done = 0;
        int J6done = 0;

        String J1calStat = "0";

        //SET DIRECTIONS
        // J1 //
        if (J1rotdir == 1 && J1caldir == 1) {
          digitalWrite(J1dirPin, HIGH);
        }
        else if (J1rotdir == 0 && J1caldir == 1) {
          digitalWrite(J1dirPin, LOW);
        }
        else if (J1rotdir == 1 && J1caldir == 0) {
          digitalWrite(J1dirPin, LOW);
        }
        else if (J1rotdir == 0 && J1caldir == 0) {
          digitalWrite(J1dirPin, HIGH);
        }

        // J2 //
        if (J2rotdir == 1 && J2caldir == 1) {
          digitalWrite(J2dirPin, HIGH);
        }
        else if (J2rotdir == 0 && J2caldir == 1) {
          digitalWrite(J2dirPin, LOW);
        }
        else if (J2rotdir == 1 && J2caldir == 0) {
          digitalWrite(J2dirPin, LOW);
        }
        else if (J2rotdir == 0 && J2caldir == 0) {
          digitalWrite(J2dirPin, HIGH);
        }

        // J3 //
        if (J3rotdir == 1 && J3caldir == 1) {
          digitalWrite(J3dirPin, HIGH);
        }
        else if (J3rotdir == 0 && J3caldir == 1) {
          digitalWrite(J3dirPin, LOW);
        }
        else if (J3rotdir == 1 && J3caldir == 0) {
          digitalWrite(J3dirPin, LOW);
        }
        else if (J3rotdir == 0 && J3caldir == 0) {
          digitalWrite(J3dirPin, HIGH);
        }

        // J4 //
        if (J4rotdir == 1 && J4caldir == 1) {
          digitalWrite(J4dirPin, HIGH);
        }
        else if (J4rotdir == 0 && J4caldir == 1) {
          digitalWrite(J4dirPin, LOW);
        }
        else if (J4rotdir == 1 && J4caldir == 0) {
          digitalWrite(J4dirPin, LOW);
        }
        else if (J4rotdir == 0 && J4caldir == 0) {
          digitalWrite(J4dirPin, HIGH);
        }

        // J5 //
        if (J5rotdir == 1 && J5caldir == 1) {
          digitalWrite(J5dirPin, HIGH);
        }
        else if (J5rotdir == 0 && J5caldir == 1) {
          digitalWrite(J5dirPin, LOW);
        }
        else if (J5rotdir == 1 && J5caldir == 0) {
          digitalWrite(J5dirPin, LOW);
        }
        else if (J5rotdir == 0 && J5caldir == 0) {
          digitalWrite(J5dirPin, HIGH);
        }

        // J6 //
        if (J6rotdir == 1 && J6caldir == 1) {
          digitalWrite(J6dirPin, HIGH);
        }
        else if (J6rotdir == 0 && J6caldir == 1) {
          digitalWrite(J6dirPin, LOW);
        }
        else if (J6rotdir == 1 && J6caldir == 0) {
          digitalWrite(J6dirPin, LOW);
        }
        else if (J6rotdir == 0 && J6caldir == 0) {
          digitalWrite(J6dirPin, HIGH);
        }

        int Speed = 1200;

        //DRIVE MOTORS FOR CALIBRATION
        while (digitalRead(J1calPin) == LOW && J1done < J1step || digitalRead(J2calPin) == LOW && J2done < J2step || digitalRead(J3calPin) == LOW && J3done < J3step || digitalRead(J4calPin) == LOW && J4done < J4step || digitalRead(J5calPin) == LOW && J5done < J5step || digitalRead(J6calPin) == LOW && J6done < J6step)
        {
          if (J1done < J1step && (digitalRead(J1calPin) == LOW))
          {
            digitalWrite(J1stepPin, LOW);
          }
          delayMicroseconds(5);
          if (J1done < J1step && (digitalRead(J1calPin) == LOW))
          {
            digitalWrite(J1stepPin, HIGH);
            J1done = ++J1done;
          }
          delayMicroseconds(5);
          if (J2done < J2step && (digitalRead(J2calPin) == LOW))
          {
            digitalWrite(J2stepPin, LOW);
          }
          delayMicroseconds(5);
          if (J2done < J2step && (digitalRead(J2calPin) == LOW))
          {
            digitalWrite(J2stepPin, HIGH);
            J2done = ++J2done;
          }
          delayMicroseconds(5);
          if (J3done < J3step && (digitalRead(J3calPin) == LOW))
          {
            digitalWrite(J3stepPin, LOW);
          }
          delayMicroseconds(5);
          if (J3done < J3step && (digitalRead(J3calPin) == LOW))
          {
            digitalWrite(J3stepPin, HIGH);
            J3done = ++J3done;
          }
          delayMicroseconds(5);
          if (J4done < J4step && (digitalRead(J4calPin) == LOW))
          {
            digitalWrite(J4stepPin, LOW);
          }
          delayMicroseconds(5);
          if (J4done < J4step && (digitalRead(J4calPin) == LOW))
          {
            digitalWrite(J4stepPin, HIGH);
            J4done = ++J4done;
          }
          delayMicroseconds(5);
          if (J5done < J5step && (digitalRead(J5calPin) == LOW))
          {
            digitalWrite(J5stepPin, LOW);
          }
          delayMicroseconds(5);
          if (J5done < J5step && (digitalRead(J5calPin) == LOW))
          {
            digitalWrite(J5stepPin, HIGH);
            J5done = ++J5done;;
          }
          delayMicroseconds(5);
          if (J6done < J6step && (digitalRead(J6calPin) == LOW))
          {
            digitalWrite(J6stepPin, LOW);
          }
          delayMicroseconds(5);
          if (J6done < J6step && (digitalRead(J6calPin) == LOW))
          {
            digitalWrite(J6stepPin, HIGH);
            J6done = ++J6done;
          }
          ///////////////DELAY BEFORE RESTARTING LOOP
          delayMicroseconds(Speed);
        }
        //OVERDRIVE
        int OvrDrv = 0;
        while (OvrDrv <= 10)
        {
          digitalWrite(J1stepPin, LOW);
          digitalWrite(J2stepPin, LOW);
          digitalWrite(J3stepPin, LOW);
          digitalWrite(J4stepPin, LOW);
          digitalWrite(J5stepPin, LOW);
          digitalWrite(J6stepPin, LOW);
          ///////////////DELAY AND SET HIGH
          delayMicroseconds(Speed);
          digitalWrite(J1stepPin, HIGH);
          digitalWrite(J2stepPin, HIGH);
          digitalWrite(J3stepPin, HIGH);
          digitalWrite(J4stepPin, HIGH);
          digitalWrite(J5stepPin, HIGH);
          digitalWrite(J6stepPin, HIGH);
          OvrDrv = ++OvrDrv;
          ///////////////DELAY BEFORE RESTARTING LOOP AND SETTING LOW AGAIN
          delayMicroseconds(Speed);
        }
        //SEE IF ANY SWITCHES NOT MADE
        delay(500);
        if (digitalRead(J1calPin) == HIGH && digitalRead(J2calPin) == HIGH && digitalRead(J3calPin) == HIGH && digitalRead(J4calPin) == HIGH && digitalRead(J4calPin) == HIGH && digitalRead(J5calPin) == HIGH && digitalRead(J6calPin) == HIGH)
        {
          Serial.println("pass\n");
        }
        else
        {
          Serial.println("J1fail\n");
        }
        inData = ""; // Clear recieved buffer
      }



      //-----COMMAND TO MOVE---------------------------------------------------
      //-----------------------------------------------------------------------
      if (function == "MJ")
      {
        int J1start = inData.indexOf('A');
        int J2start = inData.indexOf('B');
        int J3start = inData.indexOf('C');
        int J4start = inData.indexOf('D');
        int J5start = inData.indexOf('E');
        int J6start = inData.indexOf('F');
        int Adstart = inData.indexOf('G');
        int Asstart = inData.indexOf('H');
        int Ddstart = inData.indexOf('I');
        int Dsstart = inData.indexOf('K');
        int SPstart = inData.indexOf('S');
        int J1dir = inData.substring(J1start + 1, J1start + 2).toInt();
        int J2dir = inData.substring(J2start + 1, J2start + 2).toInt();
        int J3dir = inData.substring(J3start + 1, J3start + 2).toInt();
        int J4dir = inData.substring(J4start + 1, J4start + 2).toInt();
        int J5dir = inData.substring(J5start + 1, J5start + 2).toInt();
        int J6dir = inData.substring(J6start + 1, J6start + 2).toInt();
        int J1step = inData.substring(J1start + 2, J2start).toInt();
        int J2step = inData.substring(J2start + 2, J3start).toInt();
        int J3step = inData.substring(J3start + 2, J4start).toInt();
        int J4step = inData.substring(J4start + 2, J5start).toInt();
        int J5step = inData.substring(J5start + 2, J6start).toInt();
        int J6step = inData.substring(J6start + 2, SPstart).toInt();
        float SpeedIn = inData.substring(SPstart + 1, Adstart).toFloat();
        float ACCdur = inData.substring(Adstart + 1, Asstart).toInt();
        float ACCspd = inData.substring(Asstart + 1, Ddstart).toInt();
        float DCCdur = inData.substring(Ddstart + 1, Dsstart).toInt();
        float DCCspd = inData.substring(Dsstart + 1).toInt();
        Serial.print("command recieved");

        //FIND HIGHEST STEP
        int HighStep = J1step;
        if (J2step > HighStep)
        {
          HighStep = J2step;
        }
        if (J3step > HighStep)
        {
          HighStep = J3step;
        }
        if (J4step > HighStep)
        {
          HighStep = J4step;
        }
        if (J5step > HighStep)
        {
          HighStep = J5step;
        }
        if (J6step > HighStep)
        {
          HighStep = J6step;
        }

        int J1_PE = 0;
        int J2_PE = 0;
        int J3_PE = 0;
        int J4_PE = 0;
        int J5_PE = 0;
        int J6_PE = 0;

        int J1_SE_1 = 0;
        int J2_SE_1 = 0;
        int J3_SE_1 = 0;
        int J4_SE_1 = 0;
        int J5_SE_1 = 0;
        int J6_SE_1 = 0;

        int J1_SE_2 = 0;
        int J2_SE_2 = 0;
        int J3_SE_2 = 0;
        int J4_SE_2 = 0;
        int J5_SE_2 = 0;
        int J6_SE_2 = 0;

        int J1_LO_1 = 0;
        int J2_LO_1 = 0;
        int J3_LO_1 = 0;
        int J4_LO_1 = 0;
        int J5_LO_1 = 0;
        int J6_LO_1 = 0;

        int J1_LO_2 = 0;
        int J2_LO_2 = 0;
        int J3_LO_2 = 0;
        int J4_LO_2 = 0;
        int J5_LO_2 = 0;
        int J6_LO_2 = 0;

        //reset
        int J1cur = 0;
        int J2cur = 0;
        int J3cur = 0;
        int J4cur = 0;
        int J5cur = 0;
        int J6cur = 0;

        int J1_PEcur = 0;
        int J2_PEcur = 0;
        int J3_PEcur = 0;
        int J4_PEcur = 0;
        int J5_PEcur = 0;
        int J6_PEcur = 0;

        int J1_SE_1cur = 0;
        int J2_SE_1cur = 0;
        int J3_SE_1cur = 0;
        int J4_SE_1cur = 0;
        int J5_SE_1cur = 0;
        int J6_SE_1cur = 0;

        int J1_SE_2cur = 0;
        int J2_SE_2cur = 0;
        int J3_SE_2cur = 0;
        int J4_SE_2cur = 0;
        int J5_SE_2cur = 0;
        int J6_SE_2cur = 0;

        int highStepCur = 0;



        //SET DIRECTIONS

        /////// J1 /////////
        if (J1dir == 1 && J1rotdir == 1)
        {
          digitalWrite(J1dirPin, LOW);
        }
        else if (J1dir == 1 && J1rotdir == 0)
        {
          digitalWrite(J1dirPin, HIGH);
        }
        else if (J1dir == 0 && J1rotdir == 1)
        {
          digitalWrite(J1dirPin, HIGH);
        }
        else if (J1dir == 0 && J1rotdir == 0)
        {
          digitalWrite(J1dirPin, LOW);
        }

        /////// J2 /////////
        if (J2dir == 1 && J2rotdir == 1)
        {
          digitalWrite(J2dirPin, LOW);
        }
        else if (J2dir == 1 && J2rotdir == 0)
        {
          digitalWrite(J2dirPin, HIGH);
        }
        else if (J2dir == 0 && J2rotdir == 1)
        {
          digitalWrite(J2dirPin, HIGH);
        }
        else if (J2dir == 0 && J2rotdir == 0)
        {
          digitalWrite(J2dirPin, LOW);
        }

        /////// J3 /////////
        if (J3dir == 1 && J3rotdir == 1)
        {
          digitalWrite(J3dirPin, LOW);
        }
        else if (J3dir == 1 && J3rotdir == 0)
        {
          digitalWrite(J3dirPin, HIGH);
        }
        else if (J3dir == 0 && J3rotdir == 1)
        {
          digitalWrite(J3dirPin, HIGH);
        }
        else if (J3dir == 0 && J3rotdir == 0)
        {
          digitalWrite(J3dirPin, LOW);
        }

        /////// J4 /////////
        if (J4dir == 1 && J4rotdir == 1)
        {
          digitalWrite(J4dirPin, LOW);
        }
        else if (J4dir == 1 && J4rotdir == 0)
        {
          digitalWrite(J4dirPin, HIGH);
        }
        else if (J4dir == 0 && J4rotdir == 1)
        {
          digitalWrite(J4dirPin, HIGH);
        }
        else if (J4dir == 0 && J4rotdir == 0)
        {
          digitalWrite(J4dirPin, LOW);
        }

        /////// J5 /////////
        if (J5dir == 1 && J5rotdir == 1)
        {
          digitalWrite(J5dirPin, LOW);
        }
        else if (J5dir == 1 && J5rotdir == 0)
        {
          digitalWrite(J5dirPin, HIGH);
        }
        else if (J5dir == 0 && J5rotdir == 1)
        {
          digitalWrite(J5dirPin, HIGH);
        }
        else if (J5dir == 0 && J5rotdir == 0)
        {
          digitalWrite(J5dirPin, LOW);
        }

        /////// J6 /////////
        if (J6dir == 1 && J6rotdir == 1)
        {
          digitalWrite(J6dirPin, LOW);
        }
        else if (J6dir == 1 && J6rotdir == 0)
        {
          digitalWrite(J6dirPin, HIGH);
        }
        else if (J6dir == 0 && J6rotdir == 1)
        {
          digitalWrite(J6dirPin, HIGH);
        }
        else if (J6dir == 0 && J6rotdir == 0)
        {
          digitalWrite(J6dirPin, LOW);
        }


        /////CALC SPEEDS//////
        float ACCStep = (HighStep * (ACCdur / 100));
        float DCCStep = HighStep - (HighStep * (DCCdur / 100));
        float AdjSpeed = (SpeedIn / 100);
        //REG SPEED
        float CalcRegSpeed = (SpeedMult / AdjSpeed);
        int REGSpeed = int(CalcRegSpeed);

        //ACC SPEED
        float ACCspdT = (ACCspd / 100);
        float CalcACCSpeed = ((SpeedMult + (SpeedMult / ACCspdT)) / AdjSpeed);
        float ACCSpeed = (CalcACCSpeed);
        float ACCinc = (REGSpeed - ACCSpeed) / ACCStep;

        //DCC SPEED
        float DCCspdT = (DCCspd / 100);
        float CalcDCCSpeed = ((SpeedMult + (SpeedMult / DCCspdT)) / AdjSpeed);
        float DCCSpeed = (CalcDCCSpeed);
        float DCCinc = (REGSpeed + DCCSpeed) / DCCStep;
        DCCSpeed = REGSpeed;


        ///// DRIVE MOTORS /////
        while (J1cur < J1step || J2cur < J2step || J3cur < J3step || J4cur < J4step || J5cur < J5step || J6cur < J6step)
        {

          /////// J1 ////////////////////////////////
          ///find pulse every
          J1_PE = (HighStep / J1step);
          ///find left over 1
          J1_LO_1 = (HighStep - (J1step * J1_PE));
          ///find skip 1
          if (J1_LO_1 > 0)
          {
            J1_SE_1 = (HighStep / J1_LO_1);
          }
          else
          {
            J1_SE_1 = 0;
          }
          ///find left over 2
          if (J1_SE_1 > 0)
          {
            J1_LO_2 = HighStep - ((J1step * J1_PE) + ((J1step * J1_PE) / J1_SE_1));
          }
          else
          {
            J1_LO_2 = 0;
          }
          ///find skip 2
          if (J1_LO_2 > 0)
          {
            J1_SE_2 = (HighStep / J1_LO_2);
          }
          else
          {
            J1_SE_2 = 0;
          }
          ////////////////////////
          if (J1cur < J1step)
          {
            if (J1_SE_2 == 0)
            {
              J1_SE_2cur = (J1_SE_2 + 1);
            }
            if (J1_SE_2cur != J1_SE_2)
            {
              J1_SE_2cur = ++J1_SE_2cur;
              if (J1_SE_1 == 0)
              {
                J1_SE_1cur = (J1_SE_1 + 1);
              }
              if (J1_SE_1cur != J1_SE_1)
              {
                J1_SE_1cur = ++J1_SE_1cur;
                J1_PEcur = ++J1_PEcur;
                if (J1_PEcur == J1_PE)
                {
                  J1cur = ++J1cur;
                  J1_PEcur = 0;
                  digitalWrite(J1stepPin, LOW);
                  delayMicroseconds(10);
                  digitalWrite(J1stepPin, HIGH);
                }
              }
              else
              {
                J1_SE_1cur = 0;
              }
            }
            else
            {
              J1_SE_2cur = 0;
            }
          }
          /////////////////////////////////////////


          /////// J2 ////////////////////////////////
          ///find pulse every
          J2_PE = (HighStep / J2step);
          ///find left over 1
          J2_LO_1 = (HighStep - (J2step * J2_PE));
          ///find skip 1
          if (J2_LO_1 > 0)
          {
            J2_SE_1 = (HighStep / J2_LO_1);
          }
          else
          {
            J2_SE_1 = 0;
          }
          ///find left over 2
          if (J2_SE_1 > 0)
          {
            J2_LO_2 = HighStep - ((J2step * J2_PE) + ((J2step * J2_PE) / J2_SE_1));
          }
          else
          {
            J2_LO_2 = 0;
          }
          ///find skip 2
          if (J2_LO_2 > 0)
          {
            J2_SE_2 = (HighStep / J2_LO_2);
          }
          else
          {
            J2_SE_2 = 0;
          }
          ////////////////////////
          if (J2cur < J2step)
          {
            if (J2_SE_2 == 0)
            {
              J2_SE_2cur = (J2_SE_2 + 1);
            }
            if (J2_SE_2cur != J2_SE_2)
            {
              J2_SE_2cur = ++J2_SE_2cur;
              if (J2_SE_1 == 0)
              {
                J2_SE_1cur = (J2_SE_1 + 1);
              }
              if (J2_SE_1cur != J2_SE_1)
              {
                J2_SE_1cur = ++J2_SE_1cur;
                J2_PEcur = ++J2_PEcur;
                if (J2_PEcur == J2_PE)
                {
                  J2cur = ++J2cur;
                  J2_PEcur = 0;
                  digitalWrite(J2stepPin, LOW);
                  delayMicroseconds(10);
                  digitalWrite(J2stepPin, HIGH);
                }
              }
              else
              {
                J2_SE_1cur = 0;
              }
            }
            else
            {
              J2_SE_2cur = 0;
            }
          }
          /////////////////////////////////////////


          /////// J3 ////////////////////////////////
          ///find pulse every
          J3_PE = (HighStep / J3step);
          ///find left over 1
          J3_LO_1 = (HighStep - (J3step * J3_PE));
          ///find skip 1
          if (J3_LO_1 > 0)
          {
            J3_SE_1 = (HighStep / J3_LO_1);
          }
          else
          {
            J3_SE_1 = 0;
          }
          ///find left over 2
          if (J3_SE_1 > 0)
          {
            J3_LO_2 = HighStep - ((J3step * J3_PE) + ((J3step * J3_PE) / J3_SE_1));
          }
          else
          {
            J3_LO_2 = 0;
          }
          ///find skip 2
          if (J3_LO_2 > 0)
          {
            J3_SE_2 = (HighStep / J3_LO_2);
          }
          else
          {
            J3_SE_2 = 0;
          }
          ////////////////////////
          if (J3cur < J3step)
          {
            if (J3_SE_2 == 0)
            {
              J3_SE_2cur = (J3_SE_2 + 1);
            }
            if (J3_SE_2cur != J3_SE_2)
            {
              J3_SE_2cur = ++J3_SE_2cur;
              if (J3_SE_1 == 0)
              {
                J3_SE_1cur = (J3_SE_1 + 1);
              }
              if (J3_SE_1cur != J3_SE_1)
              {
                J3_SE_1cur = ++J3_SE_1cur;
                J3_PEcur = ++J3_PEcur;
                if (J3_PEcur == J3_PE)
                {
                  J3cur = ++J3cur;
                  J3_PEcur = 0;
                  digitalWrite(J3stepPin, LOW);
                  delayMicroseconds(10);
                  digitalWrite(J3stepPin, HIGH);
                }
              }
              else
              {
                J3_SE_1cur = 0;
              }
            }
            else
            {
              J3_SE_2cur = 0;
            }
          }
          /////////////////////////////////////////


          /////// J4 ////////////////////////////////
          ///find pulse every
          J4_PE = (HighStep / J4step);
          ///find left over 1
          J4_LO_1 = (HighStep - (J4step * J4_PE));
          ///find skip 1
          if (J4_LO_1 > 0)
          {
            J4_SE_1 = (HighStep / J4_LO_1);
          }
          else
          {
            J4_SE_1 = 0;
          }
          ///find left over 2
          if (J4_SE_1 > 0)
          {
            J4_LO_2 = HighStep - ((J4step * J4_PE) + ((J4step * J4_PE) / J4_SE_1));
          }
          else
          {
            J4_LO_2 = 0;
          }
          ///find skip 2
          if (J4_LO_2 > 0)
          {
            J4_SE_2 = (HighStep / J4_LO_2);
          }
          else
          {
            J4_SE_2 = 0;
          }
          ////////////////////////
          if (J4cur < J4step)
          {
            if (J4_SE_2 == 0)
            {
              J4_SE_2cur = (J4_SE_2 + 1);
            }
            if (J4_SE_2cur != J4_SE_2)
            {
              J4_SE_2cur = ++J4_SE_2cur;
              if (J4_SE_1 == 0)
              {
                J4_SE_1cur = (J4_SE_1 + 1);
              }
              if (J4_SE_1cur != J4_SE_1)
              {
                J4_SE_1cur = ++J4_SE_1cur;
                J4_PEcur = ++J4_PEcur;
                if (J4_PEcur == J4_PE)
                {
                  J4cur = ++J4cur;
                  J4_PEcur = 0;
                  digitalWrite(J4stepPin, LOW);
                  delayMicroseconds(10);
                  digitalWrite(J4stepPin, HIGH);
                }
              }
              else
              {
                J4_SE_1cur = 0;
              }
            }
            else
            {
              J4_SE_2cur = 0;
            }
          }
          /////////////////////////////////////////


          /////// J5 ////////////////////////////////
          ///find pulse every
          J5_PE = (HighStep / J5step);
          ///find left over 1
          J5_LO_1 = (HighStep - (J5step * J5_PE));
          ///find skip 1
          if (J5_LO_1 > 0)
          {
            J5_SE_1 = (HighStep / J5_LO_1);
          }
          else
          {
            J5_SE_1 = 0;
          }
          ///find left over 2
          if (J5_SE_1 > 0)
          {
            J5_LO_2 = HighStep - ((J5step * J5_PE) + ((J5step * J5_PE) / J5_SE_1));
          }
          else
          {
            J5_LO_2 = 0;
          }
          ///find skip 2
          if (J5_LO_2 > 0)
          {
            J5_SE_2 = (HighStep / J5_LO_2);
          }
          else
          {
            J5_SE_2 = 0;
          }
          ////////////////////////
          if (J5cur < J5step)
          {
            if (J5_SE_2 == 0)
            {
              J5_SE_2cur = (J5_SE_2 + 1);
            }
            if (J5_SE_2cur != J5_SE_2)
            {
              J5_SE_2cur = ++J5_SE_2cur;
              if (J5_SE_1 == 0)
              {
                J5_SE_1cur = (J5_SE_1 + 1);
              }
              if (J5_SE_1cur != J5_SE_1)
              {
                J5_SE_1cur = ++J5_SE_1cur;
                J5_PEcur = ++J5_PEcur;
                if (J5_PEcur == J5_PE)
                {
                  J5cur = ++J5cur;
                  J5_PEcur = 0;
                  digitalWrite(J5stepPin, LOW);
                  delayMicroseconds(10);
                  digitalWrite(J5stepPin, HIGH);
                }
              }
              else
              {
                J5_SE_1cur = 0;
              }
            }
            else
            {
              J5_SE_2cur = 0;
            }
          }
          /////////////////////////////////////////


          /////// J6 ////////////////////////////////
          ///find pulse every
          J6_PE = (HighStep / J6step);
          ///find left over 1
          J6_LO_1 = (HighStep - (J6step * J6_PE));
          ///find skip 1
          if (J6_LO_1 > 0)
          {
            J6_SE_1 = (HighStep / J6_LO_1);
          }
          else
          {
            J6_SE_1 = 0;
          }
          ///find left over 2
          if (J6_SE_1 > 0)
          {
            J6_LO_2 = HighStep - ((J6step * J6_PE) + ((J6step * J6_PE) / J6_SE_1));
          }
          else
          {
            J6_LO_2 = 0;
          }
          ///find skip 2
          if (J6_LO_2 > 0)
          {
            J6_SE_2 = (HighStep / J6_LO_2);
          }
          else
          {
            J6_SE_2 = 0;
          }
          ////////////////////////
          if (J6cur < J6step)
          {
            if (J6_SE_2 == 0)
            {
              J6_SE_2cur = (J6_SE_2 + 1);
            }
            if (J6_SE_2cur != J6_SE_2)
            {
              J6_SE_2cur = ++J6_SE_2cur;
              if (J6_SE_1 == 0)
              {
                J6_SE_1cur = (J6_SE_1 + 1);
              }
              if (J6_SE_1cur != J6_SE_1)
              {
                J6_SE_1cur = ++J6_SE_1cur;
                J6_PEcur = ++J6_PEcur;
                if (J6_PEcur == J6_PE)
                {
                  J6cur = ++J6cur;
                  J6_PEcur = 0;
                  digitalWrite(J6stepPin, LOW);
                  delayMicroseconds(10);
                  digitalWrite(J6stepPin, HIGH);
                }
              }
              else
              {
                J6_SE_1cur = 0;
              }
            }
            else
            {
              J6_SE_2cur = 0;
            }
          }
          /////////////////////////////////////////


          // inc cur step
          highStepCur = ++highStepCur;


          ////DELAY/////
          if (highStepCur <= ACCStep)
          {
            delayMicroseconds(ACCSpeed);
            ACCSpeed = ACCSpeed + ACCinc;
          }
          else if (highStepCur >= DCCStep)
          {
            delayMicroseconds(DCCSpeed);
            DCCSpeed = DCCSpeed + DCCinc;
          }
          else
          {
            delayMicroseconds(REGSpeed);
          }

        }
        ////////MOVE COMPLETE///////////
        inData = ""; // Clear recieved buffer
        Serial.print("Move Done");
      }
      else
      {
        inData = ""; // Clear recieved buffer
      }
    }
  }
}

