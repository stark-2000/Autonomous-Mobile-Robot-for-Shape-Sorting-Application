#include <Wire.h> //Arduino i2c lib
#include <Adafruit_Sensor.h> //adafruit sensor lib
#include <Adafruit_BNO055.h> //adafruit BNO005 lib
#include <utility/imumaths.h> //Arduino math lib
  
Adafruit_BNO055 bno = Adafruit_BNO055(55); //i2c address of BNO055 sensor is 0x29 and bno is an object of class Adafruit_BNO055
 
void setup(void) 
{
  Serial.begin(9600); //Baud rate 9600
  // Serial.println("Orientation Sensor Test"); 
  // Serial.println("");
  
  /* Initialise the sensor */
  while(!bno.begin()) 
  {
    /* There was a problem detecting the BNO055 ... check your connections */
    Serial.print("Ooops, no BNO055 detected ... Check your wiring or I2C ADDR!");
    if (bno.begin())
    {
      break;
    }
  }
  
  delay(1000); //delay of 1 sec
    
  bno.setExtCrystalUse(true); //use external crystal oscillator
}


void loop(void) 
{
  /* Create a sensors_event_t object in memory to hold our results */
  sensors_event_t event; //sensors_event_t is a object to which the sensor data is stored

  /* Get a new sensor event, passing in our 'event' placeholder */ 
  bno.getEvent(&event);
  
  /* Display the floating point data */
  // Serial.print("X: ");
  // Serial.print(event.orientation.x, 4); //print x axis data
  // Serial.print("\tY: ");

  event.or
  Serial.write(event.orientation.y); //print y axis data
  // Serial.print("\tZ: ");
  // Serial.print(event.orientation.z, 4); //print z axis data
  // Serial.println("");
  
  delay(100); //delay of 100 ms of stable operation of sensor
}