
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

//establish the LED colours
int r = 255;
int g = 0;
int b = 0;

unsigned long lastRead;
//used for the sampleRate timer
int sampleRate = 100;     //the sampleRate for reading the sensor.  Without this it will crash.
boolean lightsOn = true; // used to determine whether or not the lights should blink;


float xOrientation;     //holds the X orientation    Degrees
float yOrientation;     //holds the Y orientation    Degrees
float zOrientation;      //holds the Z orientation   Degrees

float lastX; // holds the last X orientation sample
float lastY; // holds the last Y orientation sample
float lastZ; // holds the last Z orientation sample
float velocityX; // holds the last velocity of X orientation sample
float velocityY; // holds the last velocity of Y orientation sample
float velocityZ; // holds the last velocity of Z orientation sample
float averageVelocity = 0;  // the average of velocity x, y, z,
float lastVelocity; // the average of the previous last velocity x, y, z
float reverseVelocity; // reverse the velocity so that low movement means high pitch
int thisPitch = 1500;

Adafruit_NeoPixel strip = Adafruit_NeoPixel(18, pixelPin, NEO_GRB + NEO_KHZ800);



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
  if (millis() - lastRead >= sampleRate)
  {
    // set the strip golcor
    if (lightsOn) {
      for (uint16_t i = 0; i < strip.numPixels(); i = i + 2) {
        strip.setPixelColor(i + 1, strip.Color(r, g , b)); //turn every third pixel on
        strip.setPixelColor(i - 2, strip.Color(0, 0, 0));
        strip.show();
      }
      lightsOn = !lightsOn;
    } else {
      for (uint16_t i = 0; i < strip.numPixels(); i = i + 2) {
        strip.setPixelColor(i + 1, strip.Color(0, 0, 0)); //turn every third pixel on
        strip.setPixelColor(i - 2, strip.Color(r, g , b));
        strip.show();
      }
      lightsOn = !lightsOn;
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
      // if the difference between lastVelocity and averageVelocity is within a "slowing" range,
      if (0.75 > (lastVelocity - averageVelocity) < 5) {
        // if the difference is less than two change the pitch;

        reverseVelocity = abs((7 - averageVelocity));
        thisPitch = map(reverseVelocity, 0.75, 7, 0, 1500);

        r = r + 50; // increase the colour slowly to red;

        tone(buzzerPin, thisPitch, 100);

      }

      // if it is less then 0.75 then it is still and blare a full loud tone
    } else if (0 > (averageVelocity - lastVelocity) < 0.75)  {

      // correlate the velocity of each orientation to either R, G, B to randomly regenerate colours
      r = map(velocityX, 0, 255, 0, 255);
      g = map(velocityY, 0, 255, 0, 255);
      b = map(velocityZ, 0, 255, 0, 255);
      tone(buzzerPin, thisPitch, 100);

    } else {

      // if it's going fast make the LED lights white;
      r = 255;
      g = 255;
      b = 255;

      // set pitch to 0
      thisPitch = 0;
      noTone(buzzerPin);
    }
    // store and reset the variables for the next loop;
    lastRead = millis();
    lastVelocity = averageVelocity;
    lastX = xOrientation;
    lastY = yOrientation;
    lastZ = zOrientation;
  }
}
