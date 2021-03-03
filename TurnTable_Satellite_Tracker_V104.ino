// TurnTable Satellite Tracker
//
// It is currently fully functioning and can track satellites across the sky reading telemetry from Orbitron
//
// It is however still a work in progress, and needs some more refinement


#include <LiquidCrystal.h>
LiquidCrystal lcd(8, 9, 4, 5, 6, 7);


#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_HMC5883_U.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
int ADXLAddress = 0x1E;
Adafruit_HMC5883_Unified mag = Adafruit_HMC5883_Unified(12345);
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28);

float headingDegrees;
float Bn55;

//--------------------------------------------------------------



const int stepPinX = 22;  //X.Step
const int dirPinX = 23;  //X.Dir
const int stepPinY = 24;  //Y.Step
const int dirPinY = 25;  //Y.Dir
int CompStep = 1;
int ElevStep = 1;
int AZstepDelaySpeed = 500;
int ELstepDelaySpeed = 500;






//--------------------------------------------------------------

// averaging filter Compass
const int CnumReadings = 20;

int Creadings[CnumReadings];      // the readings from the analog input
int CreadIndex = 0;              // the index of the current reading
int Ctotal = 0;                  // the running total
int Compassaverage = 0;                // the average




//--------------------------------------------------------------
// Receive with start- and end-markers combined with parsing
const byte numChars = 32;
char receivedChars[numChars];
char tempChars[numChars];        // temporary array for use when parsing
// variables to hold the parsed data
char messageFromPC[numChars] = {0};
float orbAzi = 0.0;
float orbElv = 0.0;
boolean newData = false;

//----------------------------------------------------------------


void recvWithStartEndMarkers() {
  static boolean recvInProgress = false;
  static byte ndx = 0;
  char startMarker = '<';
  char endMarker = '>';
  char rc;
  while (Serial.available() > 0 && newData == false) {
    rc = Serial.read();
    if (recvInProgress == true) {
      if (rc != endMarker) {
        receivedChars[ndx] = rc;
        ndx++;
        if (ndx >= numChars) {
          ndx = numChars - 1;
        }
      }
      else {
        receivedChars[ndx] = '\0'; // terminate the string
        recvInProgress = false;
        ndx = 0;
        newData = true;
      }
    }
    else if (rc == startMarker) {
      recvInProgress = true;
    }
  }
}
//---------------------------------------------------------

void parseData() {      // split the data into its parts
  char * strtokIndx; // this is used by strtok() as an index
  strtokIndx = strtok(tempChars, ",");     // get the first part - the string
  strcpy(messageFromPC, strtokIndx); // copy it to messageFromPC
  strtokIndx = strtok(NULL, ","); // this continues where the previous call left off
  orbAzi = atof(strtokIndx);     // convert this part to a float
  strtokIndx = strtok(NULL, ",");
  orbElv = atof(strtokIndx);     // convert this part to a float


}


//-------------------------------------------------------------

void Inclinometer() {   // Elevation
  sensors_event_t event;
  bno.getEvent(&event);
  Bn55 = (90 + ((float)event.orientation.y));
  //Serial.print("Inclinometer (degrees): "); Serial.print(Bn55);
  uint8_t sys, gyro, accel, mag = 0;
  bno.getCalibration(&sys, &gyro, &accel, &mag);
  delay(100);

}

//--------------------------------------

void Compass()  {
  sensors_event_t event;
  mag.getEvent(&event);
  float heading = atan2(event.magnetic.y, event.magnetic.x);
  float declinationAngle = 5.23;
  heading += declinationAngle;
  if (heading < 0)
    heading += 2 * PI;
  if (heading > 2 * PI)
    heading -= 2 * PI;
  headingDegrees = heading * 180 / M_PI;

  delay(10);
  compassfilter();

}

//-------------------------------------------

void compassfilter() { //compass
  // subtract the last reading:
  Ctotal = Ctotal - Creadings[CreadIndex];
  // read from the sensor:
  Creadings[CreadIndex] = headingDegrees;
  // add the reading to the total:
  Ctotal = Ctotal + Creadings[CreadIndex];
  // advance to the next position in the array:
  CreadIndex = CreadIndex + 1;

  // if we're at the end of the array...
  if (CreadIndex >= CnumReadings) {
    // ...wrap around to the beginning:
    CreadIndex = 0;
  }

  // calculate the average:
  Compassaverage = Ctotal / CnumReadings;
  // send it to the computer as ASCII digits
  //Serial.println(average);
  delay(1);        // delay in between reads for stability


}





//-------------------------------------------------------------

void lcdDisplay() {
  if (Compassaverage > 0 && Compassaverage < 360) {
    delay(10);
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("");
    lcd.setCursor(0, 0);
    lcd.print(orbAzi);
    lcd.setCursor(0, 1);  // set the cursor to column 0, line 1
    lcd.print("");
    lcd.setCursor(0, 1);             // (note: line 1 is the second row, since counting begins with 0):
    lcd.print(orbElv);
    //-------sensors-----------------
    lcd.setCursor(7, 0);
    lcd.print("A");
    lcd.setCursor(10, 0);
    lcd.print(Compassaverage);
    lcd.setCursor(7, 1);  // set the cursor to column 0, line 1
    lcd.print("E");
    lcd.setCursor(10, 1);             // (note: line 1 is the second row, since counting begins with 0):
    lcd.print(Bn55);


  }
  else {
    lcd.setCursor(0, 1);
    lcd.print("nuttin");
  }
}


//****************************************************

void AZforward() {
  digitalWrite(dirPinX, HIGH);
  for (int x = 0; x < CompStep; x++) {
    digitalWrite(stepPinX, HIGH);
    delayMicroseconds(AZstepDelaySpeed);
    digitalWrite(stepPinX, LOW);
    delayMicroseconds(AZstepDelaySpeed);
  }
}

//---------------------------------------------------

void AZbackward() {
  digitalWrite(dirPinX, LOW);
  for (int x = 0; x < CompStep; x++) {
    digitalWrite(stepPinX, HIGH);
    delayMicroseconds(AZstepDelaySpeed);
    digitalWrite(stepPinX, LOW);
    delayMicroseconds(AZstepDelaySpeed);
  }
}


//****************************************************


void ELforward() {
  digitalWrite(dirPinY, HIGH);
  for (int x = 0; x < ElevStep; x++) {
    digitalWrite(stepPinY, HIGH);
    delayMicroseconds(ELstepDelaySpeed);
    digitalWrite(stepPinY, LOW);
    delayMicroseconds(ELstepDelaySpeed);
  }
}

//------------------------------------------------------

void ELbackward() {
  digitalWrite(dirPinY, LOW);
  for (int x = 0; x < ElevStep; x++) {
    digitalWrite(stepPinY, HIGH);
    delayMicroseconds(ELstepDelaySpeed);
    digitalWrite(stepPinY, LOW);
    delayMicroseconds(ELstepDelaySpeed);
  }
}

//***********************************************************************************

void StartupProgram() {

  while ((Compassaverage > 1) && (Compassaverage < 5)) {

    if (Compassaverage > 0 && Compassaverage  < 180) {
      AZforward();
    }

    else if (Compassaverage > 180  && Compassaverage  < 360) {
      AZbackward();
    }
  }

}

//***********************************************************************************


void setup() {

  Serial.begin(9600);
  mag.begin();
  bno.begin();
  bno.setExtCrystalUse(true);


  pinMode(stepPinX, OUTPUT);   // X Motor / Azimuth
  pinMode(dirPinX, OUTPUT);
  pinMode(stepPinY, OUTPUT);   // Y Motor / Elevation
  pinMode(dirPinY, OUTPUT);



  // Setup the LCD display
  lcd.begin(16, 2);
  lcd.setCursor(0, 0);
  // Print message on first line
  lcd.print("startup");

  for (int CthisReading = 0; CthisReading < CnumReadings; CthisReading++) { //compass filter
    Creadings[CthisReading] = 0;
  }


  // StartupProgram();


}


//***********************************************************************************


void loop() {


  Compass();
  Inclinometer();
  lcdDisplay();


  recvWithStartEndMarkers();
  if (newData == true) {
    strcpy(tempChars, receivedChars);
    parseData();          //send to strtok to split data

    Compass();
    Inclinometer();
    lcdDisplay();


    if (orbElv > 170  && orbElv < 10) {
      delay(6000);
      Compass();
      Inclinometer();
      lcdDisplay();

    }

    else if (orbElv < 169  && orbElv > 11) {
      Compass();
      Inclinometer();
      lcdDisplay();



      while  (Compassaverage > orbAzi) {
        AZforward();
        Compass();
        
        

      }
      while (Compassaverage < orbAzi) {

        AZbackward();
        Compass();
        
        

      }


      while ( Bn55 > orbElv) {
        ELbackward();
        
        Inclinometer();
        lcdDisplay();


      }

      while ( Bn55 < orbElv) {

        ELforward();
        
        Inclinometer();
        lcdDisplay();


      }

    }

    newData = false;
  }



}
