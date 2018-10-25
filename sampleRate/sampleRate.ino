
/*
   Creation & Computation - Digital Futures, OCAD University
   Kate Hartman / Nick Puckett

   Orientation Sensor: https://github.com/adafruit/Adafruit_Sensor
   Simple Read and print of the X Y Z angles of orientation in degrees
   For Arduino Micro
   O Sensor       Arduino
   SDA*            D2
   SCL*            D3

 * *You must connect a 4.7K resistor between the SDA connection on the sensor and +5V
 * *You must connect a 4.7K resistor between the SCL connection on the sensor and +5V

*/
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
#include <Adafruit_NeoPixel.h>

Adafruit_BNO055 orientationSensor = Adafruit_BNO055();  //create a orienation sensor object

int C2 = 1500;
int C3 = 131;
int buzzerPin = 9;
int motorPin = 4;
int pixelPin = 6;

int r;
int g;
int b;

unsigned long lastRead;
unsigned long lastBlinked; //used for the sampleRate timer
int sampleRate = 75;     //the sampleRate for reading the sensor.  Without this it will crash.


float xOrientation;     //holds the X orientation    Degrees
float yOrientation;     //holds the Y orientation    Degrees
float zOrientation;      //holds the Z orientation   Degrees

float lastX;
float lastY;
float lastZ;
float velocityX;
float velocityY;
float velocityZ;
float averageVelocity = 0;
float lastVelocity;
float reverseVelocity;
int thisPitch = 1500;
int thisMotorSpeed = 255;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(20, pixelPin, NEO_GRB + NEO_KHZ800);



void setup()
{
  Serial.begin(9600);

  pinMode (buzzerPin, OUTPUT);
  if (!orientationSensor.begin()) //connect to the sensor and print an error if there is a problem
  {

    Serial.println("Can't connect to the Sensor. Check the Wiring");
    while (1);
  }

  delay(1000);  ///wait for one second for everything to start up.

  strip.begin();
  strip.show();
  orientationSensor.setExtCrystalUse(true);
}

void loop()
{
  //theaterChase(, 50);


  if (millis() - lastRead >= sampleRate)
  {


    for (int j = 0; j < 1; j++) { //do 10 cycles of chasing
      //for (int q = 0; q < 3; q++) {
      int q = 0;
      while (q < 3) {
        for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
          strip.setPixelColor(i + q, strip.Color(r, g , b)); //turn every third pixel on
        }
        if (millis() - lastBlinked >= 10)
        {
          strip.show();
          for (uint16_t i = 0; i < strip.numPixels(); i = i + 3) {
            strip.setPixelColor(i + q, 0);      //turn every third pixel off
          }
          q ++;
          lastBlinked = millis();
        }
      }

    }


    sensors_event_t event; //create an event variable
    orientationSensor.getEvent(&event); //pass it to the BNO055 object

    //get the values
    xOrientation = event.orientation.x;
    yOrientation = event.orientation.y;
    zOrientation = event.orientation.z;

    // set the orientation to a positive number for the purpose of math;
    if (xOrientation < 0) {
      xOrientation = abs(xOrientation);
    }
    if (yOrientation < 0) {
      yOrientation = abs(yOrientation);
    }
    if (zOrientation < 0) {
      zOrientation = abs(zOrientation);
    }

    // find the difference between the last angle read, and the current angle read.
    velocityX = lastX - xOrientation;
    velocityY = lastY - yOrientation;
    velocityZ = lastZ - zOrientation;

    // find velocity of each angle, and if it is a negative set it to the positive value;
    if (velocityX < 0) {
      velocityX = abs(velocityX);
    }

    if (velocityY < 0) {
      velocityY = abs(velocityY);
    }

    if (velocityZ < 0) {
      velocityZ = abs(velocityZ);
    }

    averageVelocity = ((velocityX + velocityY + velocityZ) / 3);

    Serial.print("average: ");
    Serial.print(averageVelocity);
    Serial.print(" last: ");
    Serial.print(lastVelocity);
    Serial.print(" last - average: ");
    Serial.print(lastVelocity - averageVelocity);
    Serial.print(" average - last: ");
    Serial.println(averageVelocity - lastVelocity);

    // if the last velocity is slower than the velocity before, make a louder noise
    if ((averageVelocity < lastVelocity) || (averageVelocity == lastVelocity)) {
      if (0.75 > (lastVelocity - averageVelocity) < 7) {
        // if the difference is less than two change the pitch;
        reverseVelocity = abs((7 - averageVelocity));

        thisPitch = map(reverseVelocity, 0.75, 7, 0, 1500);
        thisMotorSpeed = map(reverseVelocity, 0.75, 7, 0, 150);

        r = 255;
        b = 0;
        g = 0;

        tone(buzzerPin, thisPitch, 100);
        analogWrite(motorPin, thisMotorSpeed);
      }
      // if it is less then 0.75 then it is still and blare a full loud tone'
    } else if (0 > (averageVelocity - lastVelocity) < 0.75)  {
      r = map(velocityX, 0, 255, 0, 255);
      g = map(velocityY, 0, 255, 0, 255);
      b = map(velocityZ, 0, 255, 0, 255);
      tone(buzzerPin, thisPitch, 100);
      analogWrite(motorPin, thisMotorSpeed);

    } else {

      thisPitch = 0;
      noTone(buzzerPin);

      analogWrite(motorPin, 0);
    }

    lastRead = millis();
    lastVelocity = averageVelocity;
    lastX = xOrientation;
    lastY = yOrientation;
    lastZ = zOrientation;
    //save the value of the current time so the timer works
  }
}
