
#include <SoftwareSerial.h>

#include <TinyGPS++.h>

int buttonpin = 4;

float lattitude, longitude;

float a[2];

float *p;

SoftwareSerial gpsSerial(2, 3);

SoftwareSerial gsmSerial(5, 6);

TinyGPSPlus gps;

//servo

#define trigPin  A1
#define echoPin A0
#define buzzer 13
float new_delay; 
#include <Servo.h>
Servo bavi;


int pbuttonPin = 7;
int relayPin = 8;

int val = 0; 
int lightON = 0;
int pushed = 0;

#include <SPI.h>
#include <Wire.h>
#include <Servo.h>
#define MPU 0x68  // I2C address of the MPU-6050

Servo ServoX, ServoY;
double AcX, AcY, AcZ;
int Pitch, Roll;



void setup()

{

  

  pinMode(buttonpin, INPUT_PULLUP);


  Serial.begin(9600);

  delay(1000);

  gpsSerial.begin(9600);

  delay(1000);

  gsmSerial.begin(9600);

  delay(1000);

  Serial.print("—Tracking–");

  Serial.print("***Location***");

  gsmSerial.println("AT+CNMI=2,2,0,0,0");

  delay(3000);

  Serial.print("Initializing……");

  delay(2000);

  Serial.print("System Ready  ");

  delay(1000);


  //servo

   Serial.begin (9600); 
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);
  pinMode(buzzer,OUTPUT);
  Serial.begin(9600);
  pinMode(pbuttonPin, INPUT_PULLUP); 
  pinMode(relayPin, OUTPUT);
 digitalWrite(relayPin, HIGH);
  bavi.attach(9);

    Serial.begin(9600);
  ServoX.attach(12);
  ServoY.attach(11);
  init_MPU(); 
}

void loop()

{
  if (digitalRead(buttonpin) == LOW)

  {

    Serial.println("button pressed");

    delay(2000);

    SendMessage();
  }

  if (gsmSerial.available() > 0)

    Serial.write(gsmSerial.read());

  while (gsmSerial.available())

  {

    gsmSerial.read();
  }

  while (Serial.available())

  {

    Serial.read();
  }

  get_gsm();



  //servo
 long duration, distance;
  digitalWrite(trigPin, LOW);        
  delayMicroseconds(2);              
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);           
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  distance = (duration/2) / 29.1;
  new_delay= (distance *3) +30;
  Serial.print(distance);
  Serial.println("  cm");
  if (distance < 30)
  {
   digitalWrite(buzzer,HIGH);
   delay(500);
   digitalWrite(buzzer,LOW);
  }
  else
  {
    digitalWrite(buzzer,LOW);

  }
  
 delay(200);

{

  val = digitalRead(pbuttonPin);// read the push button value

  if(val == HIGH && lightON == LOW){

    pushed = 1-pushed;
    delay(300);//switch delay time
  }    

  lightON = val;

      if(pushed == HIGH){
        Serial.println("Light ON");
        digitalWrite(relayPin, HIGH); 
        bavi.write(160);
       
      }else{
        Serial.println("Light OFF");
        digitalWrite(relayPin, LOW);
        bavi.write(10);
 
   
      }     



  delay(200);
}


  FunctionsMPU(); 

  Roll = FunctionsPitchRoll(AcX, AcY, AcZ);   
  Pitch = FunctionsPitchRoll(AcY, AcX, AcZ);  

  int ServoRoll = map(Roll, -90, 90, 0, 179);
  int ServoPitch = map(Pitch, -90, 90, 179, 0);

  ServoX.write(ServoRoll);
  ServoY.write(ServoPitch);


  Serial.print("Pitch: "); Serial.print(Pitch);
  Serial.print("\t");
  Serial.print("Roll: "); Serial.print(Roll);
  Serial.print("\n");


  
}
void init_MPU() {
  Wire.begin();
  Wire.beginTransmission(MPU);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);
  delay(1000);
}

double FunctionsPitchRoll(double A, double B, double C) {
  double DatoA, DatoB, Value;
  DatoA = A;
  DatoB = (B * B) + (C * C);
  DatoB = sqrt(DatoB);

  Value = atan2(DatoA, DatoB);
  Value = Value * 180 / 3.14;

  return (int)Value;
}


void FunctionsMPU() {
  Wire.beginTransmission(MPU);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU, 6, true); // request a total of 14 registers
  AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
  AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)






}


 

float *get_gps()

{

  gpsSerial.listen();

  Serial.println("INSIDE get_gps");

  while (1)

  {

    while (gpsSerial.available() > 0)

    {
      gps.encode(gpsSerial.read());
    }

    if (gps.location.isUpdated())

    {
    

      Serial.print("LAT=");
      Serial.println(gps.location.lat(), 6);

      Serial.print("LONG=");
      Serial.println(gps.location.lng(), 6);

      lattitude = gps.location.lat();

      longitude = gps.location.lng();

      break;
    }
  }

  a[0] = lattitude;

  a[1] = longitude;

  return a;
}

void get_gsm()

{

  gsmSerial.listen();

  while (gsmSerial.available() > 0)


  {
    Serial.println("INSIDE gsmSerial.available");

    if (gsmSerial.find("Track"))

    {
      Serial.println("INSIDE track");

      gsmSerial.println("AT+CMGF=1");  

      delay(1000);  

      gsmSerial.println("AT+CMGS=\"x\"\r");  //replace x with moblie number

      delay(1000);

      p = get_gps();

      gsmSerial.listen();

      Serial.print("Your Car Location: ");

      gsmSerial.print("Your Car Location: ");

      Serial.print("");
      Serial.print(*p, 6);

      gsmSerial.print("");
      gsmSerial.print(*p, 6);
      gsmSerial.print(",");  // The SMS text you want to send

      Serial.print("");
      Serial.print(*(p + 1), 6);

      gsmSerial.print("");
      gsmSerial.print(*(p + 1), 6);  // The SMS text you want to send

      delay(100);

      gsmSerial.println((char)26);  // ASCII code of CTRL+Z for saying the end of sms to  the module

      delay(1000);
    }
  }
}

void SendMessage()

{

  gsmSerial.println("AT+CMGF=1");  //Sets the GSM Module in Text Mode

  delay(1000);  // Delay of 1000 milli seconds or 1 second

  gsmSerial.println("AT+CMGS=\"x\"\r");  // Replace x with mobile number

  delay(1000);

  gsmSerial.println("Please help me!");  // The SMS text you want to send

  delay(1000);

  p = get_gps();

  gsmSerial.listen();

  Serial.print("Your Position is : ");

  gsmSerial.print("position is : ");

  Serial.print("LATTITUDE=");
  Serial.print(*p, 6);

  gsmSerial.print("");
  gsmSerial.print(*p, 6);
  gsmSerial.print(",");  // The SMS text you want to send

  Serial.print("LONGITUDE=");
  Serial.print(*(p + 1), 6);

  gsmSerial.print("");
  gsmSerial.print(*(p + 1), 6);  // The SMS text you want to send

  delay(100);


  gsmSerial.println((char)26);
}
