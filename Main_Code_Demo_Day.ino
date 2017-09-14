#include <Wire.h>
#include<math.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>

const int motorPin = 9; 
const int GNDPin=13;
const int trigpin=22;  //obstacle sensor
const int echopin=24;
const int trigpin2=30; //Gap sensor
const int echopin2=32;
double inclination;
double inclination_old=0;


/////////Encder variables/////////////////
const int encoderIn = 8; // input pin for the interrupter 
int detectState=0; // Variable for reading the encoder status
int perfocount=0;
int previousState=0;
float oldtime=0;
float newtime=0;
float readingTimeStart=0;
float readingTimeFinish=0;
float rpm;
int count=0;
bool toLong=false;
/////////Encder variables/////////////////

Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);

void displaySensorDetails(void)
{
  sensor_t sensor;
  accel.getSensor(&sensor);
  Serial.println("------------------------------------");
  Serial.print  ("Sensor:       "); Serial.println(sensor.name);
  Serial.print  ("Driver Ver:   "); Serial.println(sensor.version);
  Serial.print  ("Unique ID:    "); Serial.println(sensor.sensor_id);
  Serial.print  ("Max Value:    "); Serial.print(sensor.max_value); Serial.println(" m/s^2");
  Serial.print  ("Min Value:    "); Serial.print(sensor.min_value); Serial.println(" m/s^2");
  Serial.print  ("Resolution:   "); Serial.print(sensor.resolution); Serial.println(" m/s^2");  
  Serial.println("------------------------------------");
  Serial.println("");
  delay(500);
}

void displayDataRate(void)
{
  Serial.print  ("Data Rate:    "); 
  
  switch(accel.getDataRate())
  {
    case ADXL345_DATARATE_3200_HZ:
      Serial.print  ("3200 "); 
      break;
    case ADXL345_DATARATE_1600_HZ:
      Serial.print  ("1600 "); 
      break;
    case ADXL345_DATARATE_800_HZ:
      Serial.print  ("800 "); 
      break;
    case ADXL345_DATARATE_400_HZ:
      Serial.print  ("400 "); 
      break;
    case ADXL345_DATARATE_200_HZ:
      Serial.print  ("200 "); 
      break;
    case ADXL345_DATARATE_100_HZ:
      Serial.print  ("100 "); 
      break;
    case ADXL345_DATARATE_50_HZ:
      Serial.print  ("50 "); 
      break;
    case ADXL345_DATARATE_25_HZ:
      Serial.print  ("25 "); 
      break;
    case ADXL345_DATARATE_12_5_HZ:
      Serial.print  ("12.5 "); 
      break;
    case ADXL345_DATARATE_6_25HZ:
      Serial.print  ("6.25 "); 
      break;
    case ADXL345_DATARATE_3_13_HZ:
      Serial.print  ("3.13 "); 
      break;
    case ADXL345_DATARATE_1_56_HZ:
      Serial.print  ("1.56 "); 
      break;
    case ADXL345_DATARATE_0_78_HZ:
      Serial.print  ("0.78 "); 
      break;
    case ADXL345_DATARATE_0_39_HZ:
      Serial.print  ("0.39 "); 
      break;
    case ADXL345_DATARATE_0_20_HZ:
      Serial.print  ("0.20 "); 
      break;
    case ADXL345_DATARATE_0_10_HZ:
      Serial.print  ("0.10 "); 
      break;
    default:
      Serial.print  ("???? "); 
      break;
  }  
  Serial.println(" Hz");  
}

void displayRange(void)
{
  Serial.print  ("Range:         +/- "); 
  
  switch(accel.getRange())
  {
    case ADXL345_RANGE_16_G:
      Serial.print  ("16 "); 
      break;
    case ADXL345_RANGE_8_G:
      Serial.print  ("8 "); 
      break;
    case ADXL345_RANGE_4_G:
      Serial.print  ("4 "); 
      break;
    case ADXL345_RANGE_2_G:
      Serial.print  ("2 "); 
      break;
    default:
      Serial.print  ("?? "); 
      break;
  }  
  Serial.println(" g");  
}


int motorspeed=180;
int old_motorspeed=180;
long duration;
long duration2;
int distance;
int distance2;

void setup(void) {
  // nothing happens in setup
  Serial.begin(9600);
  pinMode(encoderIn, INPUT);
  pinMode(GNDPin, OUTPUT);
  pinMode(trigpin,OUTPUT);
  pinMode(echopin,INPUT);
  pinMode(trigpin2,OUTPUT);
  pinMode(echopin2,INPUT);

  Serial.println("Accelerometer Test"); Serial.println("");
  
  /* Initialise the sensor */
  if(!accel.begin())
  {
    /* There was a problem detecting the ADXL345 ... check your connections */
    Serial.println("Ooops, no ADXL345 detected ... Check your wiring!");
    while(1);
  }

  /* Set the range to whatever is appropriate for your project */
  accel.setRange(ADXL345_RANGE_16_G);
  // displaySetRange(ADXL345_RANGE_8_G);
  // displaySetRange(ADXL345_RANGE_4_G);
  // displaySetRange(ADXL345_RANGE_2_G);
  
  /* Display some basic information on this sensor */
  displaySensorDetails();
  
  /* Display additional settings (outside the scope of sensor_t) */
  displayDataRate();
  displayRange();
  Serial.println("");
  
  Serial.begin(9600);


  
}

///////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////Start Of Main Code////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////
void loop() {

  if(motorspeed<0)
  {
   motorspeed=0;
    }
 digitalWrite(GNDPin,LOW);
///////////////////////////Measuring distance of Nearest Gap and Obstacle//////////////////////////////
  digitalWrite(trigpin,LOW);
  digitalWrite(trigpin2,LOW);
 
  delayMicroseconds(2);
  
  digitalWrite(trigpin,HIGH);
 
  
  delayMicroseconds(10);
  
// here we are just writing the distance  
  digitalWrite(trigpin,LOW);
  
  duration=pulseIn(echopin,HIGH);
  
  distance=duration*0.034/2;    //Refers to distance of nearest obstacle
 
  digitalWrite(trigpin2,LOW);
  digitalWrite(trigpin2,HIGH);
  delayMicroseconds(10);
  
  digitalWrite(trigpin2,LOW);
  duration2=pulseIn(echopin2,HIGH);
  distance2=duration2*0.034/2;
  
///////////////////////////Measuring distance of Nearest Gap and Obstacle//////////////////////////////

  analogWrite(motorPin,motorspeed);


  Serial.print("Obstacle Distance: ");
  Serial.println(distance);

  Serial.print("Gap distance:");
  Serial.println(distance2);  

  
////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// control the motor speed due to distances (obstacles)


// This is the case of an obstacle detected  
        if(distance<10 )
          
          {
            
              while(motorspeed>0)         //Slow down a motor
                {
                    //digitalWrite(GNDPin,LOW);
                    analogWrite(motorPin,motorspeed);
                    if(motorspeed>5)
                    {motorspeed-=5;}
                    else 
                    {motorspeed=0;}
                    delay(100); 
                }
          }


          else if (distance>15 && distance2<50)
          {
           motorspeed=180;
          }



// This is for the case of the gap: due to the hardware implementation, the distance that sensor 2 should read is 11cm. in case of a sudden change let's say 50 cm, we can talk about a gap observed.
     
        if(distance2>50)
          {
                while(motorspeed>0)
                {
                    analogWrite(motorPin,motorspeed);
                    if(motorspeed>5)
                    {
                    motorspeed-=5;
                    }
                    else 
                    {
                      motorspeed=0;
                      }
                    delay(100); 
                }
          }          

/////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

// In this case we are measuring the inclination angle and accordingly, we control the speed of the motor

//  /* Get a new sensor event */ 
  sensors_event_t event; 
  accel.getEvent(&event);

  digitalWrite(GNDPin,LOW);
 
  /* Display the results (acceleration is measured in m/s^2) */
  //Serial.print("X: "); Serial.print(event.acceleration.x-0.83); Serial.print("  ");
  Serial.print("Y: "); Serial.print(event.acceleration.y-0.32); Serial.print("  ");
  inclination= (asin((event.acceleration.y)/10))*180/3.14; Serial.print("Inclination: ");Serial.println(inclination);  // arc tangent of x
  
  if(inclination_old>0 && (distance>15)) ///We need to include && distance2<50
  {
  motorspeed=motorspeed+5*(inclination-inclination_old);

  if(motorspeed<0)
  {
    motorspeed=0;
    }
  
  if( inclination>=60 || inclination_old>=60 )
  {
    motorspeed=255;
  }
  }
  
  if (inclination_old<0)
  {
    motorspeed=0;
    }
  Serial.print("MotorSpeed: ");Serial.println(motorspeed);
  analogWrite(motorPin,motorspeed);
  inclination_old=inclination;


////////////////////////////////////////////////////////
////////////////////Encoder/////////////////////////////
////////////////////////////////////////////////////////
  oldtime = millis();

   readingTimeStart=millis();
    while (perfocount<40 && (!toLong))
   {  
       
       detectState=digitalRead(encoderIn);
       if (detectState !=previousState) { //If encoder output is high
          perfocount+=1;
          }

          
         previousState=detectState;  

       if(perfocount==40)
       { newtime = millis();

        rpm=(60000.0)/(newtime-oldtime);
           

           Serial.println("rpm=");
            Serial.println(rpm);

           oldtime=newtime;

         
            }
            readingTimeFinish=millis();
        if((readingTimeFinish-readingTimeStart)>3000)
           {
            Serial.println("rpm=");
            Serial.println(0);
            toLong=1;
            
            
           }
        
   }
   //readingTime=0;
   toLong=0;
   perfocount=0;
   
////////////////////////////////////////////////////////
////////////////////Encoder/////////////////////////////
////////////////////////////////////////////////////////
  

  delay(100);

}
///////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////Start Of Main Code////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////////////////////////////////


