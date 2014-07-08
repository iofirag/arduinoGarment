/************************************************************************************/
#include <OneWire.h> 
// Data -----^^^----- + 
// ^^^^ + 5v
// ^^^^ Data digital 8
// Gnd
/**HOT sensor********************************************************************************/
int DS18S20_Pin = 8; //DS18S20 Signal pin on digital 2

// digital pin's
int batteryled1 = 3;
int batteryled2 = 4;
int batteryled3 = 5;
int batteryled4 = 6;
int batteryled5 = 7;

int vibPin = 9;
//int ledPin = 11;

//first time checker
boolean firstTimeRun=true;

float tempMakeTired=29.0;
boolean tired=false;
int pihukCounter=0;
const int pihukCounterMAX = 2;

int flexSensor = 3;
float const FLEX_MAX_TIRED=8.2;

int const MAXTraining=2;

// Vibrator
boolean isVib = false;


//Temperature chip i/o
OneWire ds(DS18S20_Pin);  // on digital pin 2




/************************************************************************************
 * 	
 * 	Name    : MMA8453_n0m1 Library Example: DataMode                       
 * 	Author  : Noah Shibley, Michael Grant, NoMi Design Ltd. http://n0m1.com                       
 * 	Date    : Feb 10th 2012                                    
 * 	Version : 0.1                                              
 * 	Notes   : Arduino Library for use with the Freescale MMA8453Q via i2c. 
 *
 * Hardware setup:
 * MMA8452 Breakout ------------ Arduino
 * 3.3V --------------------- 3.3V
 * SDA ----------------------- A4
 * SCL ----------------------- A5
 * INT2 ---------------------- D3
 * INT1 ---------------------- D2
 * GND ---------------------- GND
 ***********************************************************************************/
#include <I2C.h>
#include <MMA8453_n0m1.h>

MMA8453_n0m1 accel;

float const MAX_training_x_degree=180;
float x,y,z;

void setup()
{
  pinMode(batteryled1, OUTPUT);    
  pinMode(batteryled2, OUTPUT);    
  pinMode(batteryled3, OUTPUT);    
  pinMode(batteryled4, OUTPUT);    
  pinMode(batteryled5, OUTPUT);  
 //turn on 5 leds 
  digitalWrite(batteryled1, HIGH);
  digitalWrite(batteryled2, HIGH);
  digitalWrite(batteryled3, HIGH);
  digitalWrite(batteryled4, HIGH);
  digitalWrite(batteryled5, HIGH); 
  
   pinMode(vibPin, OUTPUT);
   analogWrite(vibPin, LOW);
   
    //pinMode(ledPin, OUTPUT);
    //analogWrite(ledPin, LOW);

  //--Accelerometer-----------------------------
  Serial.begin(9600);
//  pinMode(led_r, OUTPUT);
//  pinMode(led_y, OUTPUT);
//  pinMode(led_g, OUTPUT);
  accel.setI2CAddr(0x1D); //change your device address if necessary, default is 0x1C
  accel.dataMode(true, 2); //enable highRes 10bit, 2g range [2g,4g,8g]
  Serial.println("MMA8453_n0m1 library");
  Serial.println("XYZ Data Example");
  Serial.println("n0m1.com");
}

void loop()
{
  // Temperature Sensor
  // check sensor 10 times and calculate AVG from the 10 result
    float temperatureMone=0;                //init
    float tempAvg=0;                        //init
    float flexValueMone=0;                //init
    float flexValueAVG=0;                //init
    
    for(int i=1; i<=10; i++){
            // Make AVG of TEMPERATURE sensor values
            temperatureMone += getTemp();
            tempAvg= temperatureMone/i;
            Serial.println(tempAvg);
            
            // Make AVG of FLEX sensor values
            flexValueMone += analogRead(flexSensor);
            flexValueAVG= flexValueMone/i;
            Serial.println(flexValueAVG);
            
            delay (200);
    }
    if (firstTimeRun){
              // init 'tempMakeTired'
             //this variable use to check if man is tired
              tempMakeTired=tempAvg+1.5;
              Serial.print("The Temperature will make Tired is: ");
              Serial.print(tempMakeTired);
              
              firstTimeRun=false;
    }
  
  /* if the temperature average > temperature can make person tired */
  if (  tempAvg > tempMakeTired ) {
          pihukCounter++;
          digitalWrite(batteryled5, LOW);
          delay(500);
          digitalWrite(batteryled5, HIGH);
          Serial.println("Person is about to fall asleep......");
          //show that was 
           if (pihukCounter==pihukCounterMAX){
                   tired=true;  
                   Serial.println("Person is tired!");
                   pihukCounter=0;        //init
           }else{
                    delay (3000);
           }   
  }
  if  ( (pihukCounter>= pihukCounterMAX-1) && flexValueAVG<= FLEX_MAX_TIRED) {
          tired=true;
          pihukCounter=0;        //init
          Serial.println("Person is tired!");
  }
  
  int training=0;
  if (tired==true){
          for (int i=1; i<=5; i++){
                     int power = i%2;
                     digitalWrite(batteryled1, power);
                     digitalWrite(batteryled2, power);
                     digitalWrite(batteryled3, power);
                     digitalWrite(batteryled4, power);
                     digitalWrite(batteryled5, power);
                     delay(500);
                     if (i==5){
                             digitalWrite(batteryled5, LOW);
                             delay(300);
                             digitalWrite(batteryled4, LOW);
                             delay(300);
                             digitalWrite(batteryled3, LOW);
                             delay(300);
                             digitalWrite(batteryled2, LOW);
                             delay(300);
                             digitalWrite(batteryled1, LOW);
                     }
           }
  }
  while (tired==true){
           if (isVib==true) {  
                     analogWrite(vibPin, 0);
                     isVib=false;
           }
           else {    
                      analogWrite (vibPin, 255-training*51);
                      isVib=true;         
           }
           delay(500);

            //check 'fresh sensor' to initialize the tired to-> false again
            //check accelerometer detection values
           
            //--ACCELEROMETER--------------------------------------
          accel.update();
          x= accel.x();
          y= accel.y();
          z= accel.z();
          
          Serial.print("x: ");
          Serial.print(accel.x());
          Serial.print(" y: ");
          Serial.print(accel.y());
          Serial.print(" z: ");
          Serial.println(accel.z());
  
          if (x>=MAX_training_x_degree){
                  training++;
                  Serial.print(training);
                  Serial.print("  of ");
                  Serial.println(MAXTraining); 
                  digitalWrite(batteryled1, HIGH);
                  if (training==MAXTraining) {
                            tired=false;
                            analogWrite(vibPin, 0);
                            Serial.println("Person is waking up ! :)");
                            
                            digitalWrite(batteryled1, HIGH);
                             delay(300);
                             digitalWrite(batteryled2, HIGH);
                             delay(300);
                             digitalWrite(batteryled3, HIGH);
                             delay(300);
                             digitalWrite(batteryled4, HIGH);
                             delay(300);
                             digitalWrite(batteryled5, HIGH);
                            
                            // fade in the led
                            //for(int fadeValue = 0 ; fadeValue <= 255; fadeValue +=5) { 
                            //        analogWrite(ledPin, fadeValue);
                            //        delay(30);                            
                            //} 
                            //analogWrite(ledPin, LOW);      // close the led
                   }
                   delay(1500);
          }
  }




  
  //---TEST-------------------------------------------------
  //  Serial.print("x: ");
  //  Serial.print(accel.x());
  //  Serial.print(" y: ");
  //  Serial.print(accel.y());
  //  Serial.print(" z: ");
  //  Serial.println(accel.z());

  //  //init
  //  x_max= accel.x();
  //  y_max = accel.y();
  //  z_max = accel.z();
  //  while (true) {
  //    accel.update();
  //    //-------------------
  //    if ( accel.x() >x_max )
  //      x_max= accel.x();
  //    if ( accel.y() >y_max )
  //      y_max = accel.y();
  //    if ( accel.z() >z_max )
  //      z_max = accel.z();
  //      
  //      Serial.print("x: ");
  //      Serial.print(x_max);
  //      Serial.print(" y: ");
  //      Serial.print(y_max);
  //      Serial.print(" z: ");
  //      Serial.println(z_max);
  //  }
  //------------------------------------------
}

float getTemp(){
  //returns the temperature from one DS18S20 in DEG Celsius

  byte data[12];
  byte addr[8];

  if ( !ds.search(addr)) {
    //no more sensors on chain, reset search
    ds.reset_search();
    return -1000;
  }

  if ( OneWire::crc8( addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return -1000;
  }

  if ( addr[0] != 0x10 && addr[0] != 0x28) {
    Serial.print("Device is not recognized");
    return -1000;
  }

  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); // start conversion, with parasite power on at the end

  byte present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE); // Read Scratchpad


  for (int i = 0; i < 9; i++) { // we need 9 bytes
    data[i] = ds.read();
  }

  ds.reset_search();

  byte MSB = data[1];
  byte LSB = data[0];

  float tempRead = ((MSB << 8) | LSB); //using two's compliment
  float TemperatureSum = tempRead / 16;

  return TemperatureSum;
}

