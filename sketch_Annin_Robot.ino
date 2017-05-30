/*  AR2 - Stepper motor robot control software
    Copyright (c) 2017, Chris Annin
    All rights reserved.

    You are free to share, copy and redistribute in any medium
    or format.  You are free to remix, transform and build upon
    this material.

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions are met:

        * Redistributions of source code must retain the above copyright
          notice, this list of conditions and the following disclaimer.
        * Redistribution of this software in source or binary forms shall be free
          of all charges or fees to the recipient of this software.
        * Redistributions in binary form must reproduce the above copyright
          notice, this list of conditions and the following disclaimer in the
          documentation and/or other materials provided with the distribution.
        * Neither the name of Chris Annin nor the
          names of its contributors may be used to endorse or promote products
          derived from this software without specific prior written permission.
        * you must give appropriate credit and indicate if changes were made. You may do
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
*/

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

int SpeedMult = 400;

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
        int J1step = (inData.substring(J1start + 1, J2start).toInt()) + 5;
        int J2step = (inData.substring(J2start + 1, J3start).toInt()) + 5;
        int J3step = (inData.substring(J3start + 1, J4start).toInt()) + 5;
        int J4step = (inData.substring(J4start + 1, J5start).toInt()) + 5;
        int J5step = (inData.substring(J5start + 1, J6start).toInt()) + 5;
        int J6step = (inData.substring(J6start + 1).toInt()) + 5;

        //RESET COUNTERS
        int J1done = 0;
        int J2done = 0;
        int J3done = 0;
        int J4done = 0;
        int J5done = 0;
        int J6done = 0;

        String J1calStat = "0";

        //SET DIRECTIONS
        digitalWrite(J1dirPin, LOW);
        digitalWrite(J2dirPin, LOW);
        digitalWrite(J3dirPin, HIGH);
        digitalWrite(J4dirPin, LOW);
        digitalWrite(J5dirPin, LOW);
        digitalWrite(J6dirPin, HIGH);

        int Speed = 1000;

        //DRIVE MOTORS FOR CALIBRATION
        while (digitalRead(J1calPin) == LOW && J1done < J1step || digitalRead(J2calPin) == LOW && J2done < J2step || digitalRead(J3calPin) == LOW && J3done < J3step || digitalRead(J4calPin) == LOW && J4done < J4step || digitalRead(J5calPin) == LOW && J5done < J5step || digitalRead(J6calPin) == LOW && J6done < J6step)
        {
          if (J1done < J1step && (digitalRead(J1calPin) == LOW))
          {
            digitalWrite(J1stepPin, LOW);
          }
          if (J2done < J2step && (digitalRead(J2calPin) == LOW))
          {
            digitalWrite(J2stepPin, LOW);
          }
          if (J3done < J3step && (digitalRead(J3calPin) == LOW))
          {
            digitalWrite(J3stepPin, LOW);
          }
          if (J4done < J4step && (digitalRead(J4calPin) == LOW))
          {
            digitalWrite(J4stepPin, LOW);
          }
          if (J5done < J5step && (digitalRead(J5calPin) == LOW))
          {
            digitalWrite(J5stepPin, LOW);
          }
          if (J6done < J6step && (digitalRead(J6calPin) == LOW))
          {
            digitalWrite(J6stepPin, LOW);
          }
          //#############DELAY AND SET HIGH
          delayMicroseconds(Speed);
          if (J1done < J1step && (digitalRead(J1calPin) == LOW))
          {
            digitalWrite(J1stepPin, HIGH);
            J1done = ++J1done;
          }
          if (J2done < J2step && (digitalRead(J2calPin) == LOW))
          {
            digitalWrite(J2stepPin, HIGH);
            J2done = ++J2done;
          }
          if (J3done < J3step && (digitalRead(J3calPin) == LOW))
          {
            digitalWrite(J3stepPin, HIGH);
            J3done = ++J3done;
          }
          if (J4done < J4step && (digitalRead(J4calPin) == LOW))
          {
            digitalWrite(J4stepPin, HIGH);
            J4done = ++J4done;
          }
          if (J5done < J5step && (digitalRead(J5calPin) == LOW))
          {
            digitalWrite(J5stepPin, HIGH);
            J5done = ++J5done;;
          }
          if (J6done < J6step && (digitalRead(J6calPin) == LOW))
          {
            digitalWrite(J6stepPin, HIGH);
            J6done = ++J6done;
          }
          //#############DELAY BEFORE RESTARTING LOOP AND SETTING LOW AGAIN
          delayMicroseconds(Speed);
        }
        //OVERDRIVE
        int OvrDrv = 0;
        while (OvrDrv <= 5)
        {
          digitalWrite(J1stepPin, LOW);
          digitalWrite(J2stepPin, LOW);
          digitalWrite(J3stepPin, LOW);
          digitalWrite(J4stepPin, LOW);
          digitalWrite(J5stepPin, LOW);
          digitalWrite(J6stepPin, LOW);
          //#############DELAY AND SET HIGH
          delayMicroseconds(Speed);
          digitalWrite(J1stepPin, HIGH);
          digitalWrite(J2stepPin, HIGH);
          digitalWrite(J3stepPin, HIGH);
          digitalWrite(J4stepPin, HIGH);
          digitalWrite(J5stepPin, HIGH);
          digitalWrite(J6stepPin, HIGH);
          OvrDrv = ++OvrDrv;
          //#############DELAY BEFORE RESTARTING LOOP AND SETTING LOW AGAIN
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


        //FIND HIGHEST STEP
        int highStep = J1step;
        if (J2step > highStep)
        {
          highStep = J2step;
        }
        if (J3step > highStep)
        {
          highStep = J3step;
        }
        if (J4step > highStep)
        {
          highStep = J4step;
        }
        if (J5step > highStep)
        {
          highStep = J5step;
        }
        if (J6step > highStep)
        {
          highStep = J6step;
        }

        //COORDINATE MOTORS TO COMPLETE SAME TIME

        //LEVEL 1 SKIPS (PULSE ON Xth)
        int J1skipL1 = (highStep / J1step);
        int J2skipL1 = (highStep / J2step);
        int J3skipL1 = (highStep / J3step);
        int J4skipL1 = (highStep / J4step);
        int J5skipL1 = (highStep / J5step);
        int J6skipL1 = (highStep / J6step);

        int J1LpsRem2 = highStep - (J1step * J1skipL1);
        int J2LpsRem2 = highStep - (J2step * J2skipL1);
        int J3LpsRem2 = highStep - (J3step * J3skipL1);
        int J4LpsRem2 = highStep - (J4step * J4skipL1);
        int J5LpsRem2 = highStep - (J5step * J5skipL1);
        int J6LpsRem2 = highStep - (J6step * J6skipL1);

        int J1skipL2;
        int J2skipL2;
        int J3skipL2;
        int J4skipL2;
        int J5skipL2;
        int J6skipL2;

        //LEVEL 2 SKIPS (SKIP ON Xth)
        ///J1///
        if (J1LpsRem2 = 0)
        {
          int J1skipL2 = 0;
        }
        else
        {
          int J1skipL2 (highStep / J1LpsRem2);
        }
        ///J2///
        if (J2LpsRem2 = 0)
        {
          int J2skipL2 = 0;
        }
        else
        {
          int J2skipL2 (highStep / J2LpsRem2);
        }
        ///J3///
        if (J3LpsRem2 = 0)
        {
          int J3skipL2 = 0;
        }
        else
        {
          int J3skipL2 (highStep / J3LpsRem2);
        }
        ///J4///
        if (J4LpsRem2 = 0)
        {
          int J4skipL2 = 0;
        }
        else
        {
          int J4skipL2 (highStep / J4LpsRem2);
        }
        ///J5///
        if (J5LpsRem2 = 0)
        {
          int J5skipL2 = 0;
        }
        else
        {
          int J5skipL2 (highStep / J5LpsRem2);
        }
        ///J6///
        if (J6LpsRem2 = 0)
        {
          int J6skipL2 = 0;
        }
        else
        {
          int J6skipL2 (highStep / J6LpsRem2);
        }


        int J1LpsRem3;
        int J2LpsRem3;
        int J3LpsRem3;
        int J4LpsRem3;
        int J5LpsRem3;
        int J6LpsRem3;


        //SKIP REM L3
        ///J1///
        if (J1skipL2 = 0)
        {
          int J1LpsRem3 = highStep - (J1step * J1skipL1);
        }
        else
        {
          int J1LpsRem3 = highStep - (J1step / (J1skipL2 + (J1step * J1skipL1)));
        }
        //SKIP REM L3
        ///J2///
        if (J2skipL2 = 0)
        {
          int J2LpsRem3 = highStep - (J2step * J2skipL1);
        }
        else
        {
          int J2LpsRem3 = highStep - (J2step / (J2skipL2 + (J2step * J2skipL1)));
        }
        //SKIP REM L3
        ///J3///
        if (J3skipL2 = 0)
        {
          int J3LpsRem3 = highStep - (J3step * J3skipL1);
        }
        else
        {
          int J3LpsRem3 = highStep - (J3step / (J3skipL2 + (J3step * J3skipL1)));
        }
        //SKIP REM L3
        ///J4///
        if (J4skipL2 = 0)
        {
          int J4LpsRem3 = highStep - (J4step * J4skipL1);
        }
        else
        {
          int J4LpsRem3 = highStep - (J4step / (J4skipL2 + (J4step * J4skipL1)));
        }
        //SKIP REM L3
        ///J5///
        if (J5skipL2 = 0)
        {
          int J5LpsRem3 = highStep - (J5step * J5skipL1);
        }
        else
        {
          int J5LpsRem3 = highStep - (J5step / (J5skipL2 + (J5step * J5skipL1)));
        }
        //SKIP REM L3
        ///J6///
        if (J6skipL2 = 0)
        {
          int J6LpsRem3 = highStep - (J6step * J6skipL1);
        }
        else
        {
          int J6LpsRem3 = highStep - (J6step / (J6skipL2 + (J6step * J6skipL1)));
        }

        int J1skipL3;
        int J2skipL3;
        int J3skipL3;
        int J4skipL3;
        int J5skipL3;
        int J6skipL3;

        //SKIP L3
        ///J1///
        if (J1LpsRem3 = 0)
        {
          int J1skipL3 = 0;
        }
        else
        {
          int J1skipL3 = (highStep / J1LpsRem3);
        }
        //SKIP L3
        ///J2///
        if (J2LpsRem3 = 0)
        {
          int J2skipL3 = 0;
        }
        else
        {
          int J2skipL3 = (highStep / J2LpsRem3);
        }
        //SKIP L3
        ///J3///
        if (J3LpsRem3 = 0)
        {
          int J3skipL3 = 0;
        }
        else
        {
          int J3skipL3 = (highStep / J3LpsRem3);
        }
        //SKIP L3
        ///J4///
        if (J4LpsRem3 = 0)
        {
          int J4skipL3 = 0;
        }
        else
        {
          int J4skipL3 = (highStep / J4LpsRem3);
        }
        //SKIP L3
        ///J5///
        if (J5LpsRem3 = 0)
        {
          int J5skipL3 = 0;
        }
        else
        {
          int J5skipL3 = (highStep / J5LpsRem3);
        }
        //SKIP L3
        ///J6///
        if (J6LpsRem3 = 0)
        {
          int J6skipL3 = 0;
        }
        else
        {
          int J6skipL3 = (highStep / J6LpsRem3);
        }


        //RESET COUNTERS
        int highStepCur = 0;
        int J1done = 0;
        int J2done = 0;
        int J3done = 0;
        int J4done = 0;
        int J5done = 0;
        int J6done = 0;

        //RESET LEVEL 1 SKIP CURRENT
        int J1skipL1Cur = 0;
        int J2skipL1Cur = 0;
        int J3skipL1Cur = 0;
        int J4skipL1Cur = 0;
        int J5skipL1Cur = 0;
        int J6skipL1Cur = 0;

        //RESET LEVEL 2 SKIP CURRENT
        int J1skipL2Cur = 1;
        int J2skipL2Cur = 1;
        int J3skipL2Cur = 1;
        int J4skipL2Cur = 1;
        int J5skipL2Cur = 1;
        int J6skipL2Cur = 1;

        //RESET LEVEL 3 SKIP CURRENT
        int J1skipL3Cur = 1;
        int J2skipL3Cur = 1;
        int J3skipL3Cur = 1;
        int J4skipL3Cur = 1;
        int J5skipL3Cur = 1;
        int J6skipL3Cur = 1;

        //SET DIRECTIONS
        if (J1dir == 1)
        {
          digitalWrite(J1dirPin, LOW);
        }
        else if (J1dir == 0)
        {
          digitalWrite(J1dirPin, HIGH);
        }
        if (J2dir == 1)
        {
          digitalWrite(J2dirPin, LOW);
        }
        else if (J2dir == 0)
        {
          digitalWrite(J2dirPin, HIGH);
        }
        if (J3dir == 1)
        {
          digitalWrite(J3dirPin, LOW);
        }
        else if (J3dir == 0)
        {
          digitalWrite(J3dirPin, HIGH);
        }
        if (J4dir == 1)
        {
          digitalWrite(J4dirPin, LOW);
        }
        else if (J4dir == 0)
        {
          digitalWrite(J4dirPin, HIGH);
        }
        if (J5dir == 1)
        {
          digitalWrite(J5dirPin, LOW);
        }
        else if (J5dir == 0)
        {
          digitalWrite(J5dirPin, HIGH);
        }
        if (J6dir == 1)
        {
          digitalWrite(J6dirPin, LOW);
        }
        else if (J6dir == 0)
        {
          digitalWrite(J6dirPin, HIGH);
        }

        /////CALC SPEEDS//////
        float ACCStep = (highStep * (ACCdur / 100));
        float DCCStep = highStep - (highStep * (DCCdur / 100));
        float AdjSpeed = (SpeedIn / 100);
        //REG SPEED
        float CalcRegSpeed = (SpeedMult / AdjSpeed);
        int REGSpeed = int(CalcRegSpeed) / 2;

        //ACC SPEED
        float ACCspdT = (ACCspd / 100);
        float CalcACCSpeed = ((SpeedMult + (SpeedMult / ACCspdT)) / AdjSpeed);
        float ACCSpeed = (CalcACCSpeed) / 2;
        float ACCinc = (REGSpeed - ACCSpeed) / ACCStep;

        //DCC SPEED
        float DCCspdT = (DCCspd / 100);
        float CalcDCCSpeed = ((SpeedMult + (SpeedMult / DCCspdT)) / AdjSpeed);
        float DCCSpeed = (CalcDCCSpeed) / 2;
        float DCCinc = (REGSpeed + DCCSpeed) / DCCStep;
        DCCSpeed = REGSpeed;


        /////DRIVE MOTORS////////
        while (J1done < J1step || J2done < J2step || J3done < J3step || J4done < J4step || J5done < J5step || J6done < J6step)
        {
          ////////////////////////////////////////////////
          //J1
          if (J1done < J1step)
          {
            if (J1skipL3Cur != (J1skipL3))
            {
              J1skipL3Cur = ++J1skipL3Cur;
              if (J1skipL2Cur != (J1skipL2))
              {
                J1skipL2Cur = ++J1skipL2Cur;
                if (J1skipL1Cur == 0)
                {
                  digitalWrite(J1stepPin, LOW);
                  J1done = ++J1done;
                }
              }
              else
              {
                J1skipL2Cur = 1;
              }
            }
            else
            {
              J1skipL3Cur = 1;
            }
          }
          ////////////////////////////////////////////////
          //J2
          if (J2done < J2step)
          {
            if (J2skipL3Cur != (J2skipL3))
            {
              J2skipL3Cur = ++J2skipL3Cur;
              if (J2skipL2Cur != (J2skipL2))
              {
                J2skipL2Cur = ++J2skipL2Cur;
                if (J2skipL1Cur == 0)
                {
                  digitalWrite(J2stepPin, LOW);
                  J2done = ++J2done;
                }
              }
              else
              {
                J2skipL2Cur = 1;
              }
            }
            else
            {
              J2skipL3Cur = 1;
            }
          }
          ////////////////////////////////////////////////
          //J3
          if (J3done < J3step)
          {
            if (J3skipL3Cur != (J3skipL3))
            {
              J3skipL3Cur = ++J3skipL3Cur;
              if (J3skipL2Cur != (J3skipL2))
              {
                J3skipL2Cur = ++J3skipL2Cur;
                if (J3skipL1Cur == 0)
                {
                  digitalWrite(J3stepPin, LOW);
                  J3done = ++J3done;
                }
              }
              else
              {
                J3skipL2Cur = 1;
              }
            }
            else
            {
              J3skipL3Cur = 1;
            }
          }
          ////////////////////////////////////////////////
          //J4
          if (J4done < J4step)
          {
            if (J4skipL3Cur != (J4skipL3))
            {
              J4skipL3Cur = ++J4skipL3Cur;
              if (J4skipL2Cur != (J4skipL2))
              {
                J4skipL2Cur = ++J4skipL2Cur;
                if (J4skipL1Cur == 0)
                {
                  digitalWrite(J4stepPin, LOW);
                  J4done = ++J4done;
                }
              }
              else
              {
                J4skipL2Cur = 1;
              }
            }
            else
            {
              J4skipL3Cur = 1;
            }
          }
          ////////////////////////////////////////////////
          //J5
          if (J5done < J5step)
          {
            if (J5skipL3Cur != (J5skipL3))
            {
              J5skipL3Cur = ++J5skipL3Cur;
              if (J5skipL2Cur != (J5skipL2))
              {
                J5skipL2Cur = ++J5skipL2Cur;
                if (J5skipL1Cur == 0)
                {
                  digitalWrite(J5stepPin, LOW);
                  J5done = ++J5done;
                }
              }
              else
              {
                J5skipL2Cur = 1;
              }
            }
            else
            {
              J5skipL3Cur = 1;
            }
          }
          ////////////////////////////////////////////////
          //J6
          if (J6done < J6step)
          {
            if (J6skipL3Cur != (J6skipL3))
            {
              J6skipL3Cur = ++J6skipL3Cur;
              if (J6skipL2Cur != (J6skipL2))
              {
                J6skipL2Cur = ++J6skipL2Cur;
                if (J6skipL1Cur == 0)
                {
                  digitalWrite(J6stepPin, LOW);
                  J6done = ++J6done;
                }
              }
              else
              {
                J6skipL2Cur = 1;
              }
            }
            else
            {
              J6skipL3Cur = 1;
            }
          }
          //#############DELAY BEFORE RESTARTING LOOP
          highStepCur = ++highStepCur;
          J1skipL1Cur = ++J1skipL1Cur;
          J2skipL1Cur = ++J2skipL1Cur;
          J3skipL1Cur = ++J3skipL1Cur;
          J4skipL1Cur = ++J4skipL1Cur;
          J5skipL1Cur = ++J5skipL1Cur;
          J6skipL1Cur = ++J6skipL1Cur;
          //if skiped enough times set back to zero
          if (J1skipL1Cur == J1skipL1)
          {
            J1skipL1Cur = 0;
          }
          if (J2skipL1Cur == J2skipL1)
          {
            J2skipL1Cur = 0;
          }
          if (J3skipL1Cur == J3skipL1)
          {
            J3skipL1Cur = 0;
          }
          if (J4skipL1Cur == J4skipL1)
          {
            J4skipL1Cur = 0;
          }
          if (J5skipL1Cur == J5skipL1)
          {
            J5skipL1Cur = 0;
          }
          if (J6skipL1Cur == J6skipL1)
          {
            J6skipL1Cur = 0;
          }

          ////DELAY 1/////
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

          /////RESET STEPS AND DELAY 2////
          digitalWrite(J1stepPin, HIGH);
          digitalWrite(J2stepPin, HIGH);
          digitalWrite(J3stepPin, HIGH);
          digitalWrite(J4stepPin, HIGH);
          digitalWrite(J5stepPin, HIGH);
          digitalWrite(J6stepPin, HIGH);

          if (highStepCur <= ACCStep)
          {
            delayMicroseconds(ACCSpeed);
          }
          else if (highStepCur >= DCCStep)
          {
            delayMicroseconds(DCCSpeed);
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





