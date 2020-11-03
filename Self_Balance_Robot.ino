#include <Wire.h>




#define RESTRICT_PITCH // Comment out to restrict roll to ±90deg instead - please read: http://www.freescale.com/files/sensors/doc/app_note/AN3461.pdf



//Declaring some global variables
int gyro_x, gyro_y, gyro_z;
long acc_x, acc_y, acc_z, acc_total_vector;
int temperature;
long gyro_x_cal, gyro_y_cal, gyro_z_cal;
long loop_timer;
float angle_pitch, angle_roll;
int angle_pitch_buffer, angle_roll_buffer;
boolean set_gyro_angles;
float angle_roll_acc, angle_pitch_acc;
float angle_pitch_output, angle_roll_output;

//********************************************************
// connect motor controller pins to Arduino digital pins
// motor one
int PWMA = 6;
int ForwardR = 8;
int BackwardR = 9;
// motor two
int PWMB = 10;
int ForwardL = 11;
int BackwardL = 12;
//*********************************************************

double accX, accY, accZ;
double gyroX, gyroY, gyroZ;

double gyroXangle, gyroYangle; // Angle calculate using the gyro only
double compAngleX, compAngleY; // Calculated angle using a complementary filter
uint32_t timer;

unsigned long dt;
unsigned long T=4000;
double pInput= 0;
double SetPoint = 0.00;
double Input = 0;
double Output;
double Kp = 0;
double Ki = 0;
double Kd = 0;
double p, i = 0, d;  
double angle;
double power;
int number=0;
double lastPower1,lastPower2,Power;
uint8_t i2cData[14]; 
//PID myPID(&Input , &Output , &SetPoint , Kp , Ki , Kd , DIRECT);
 
 
 int A,B,C,D,ea=0,eb=0,ec=0,ed=0,last=1;
 

void setup() {
  Wire.begin();                                                        //Start I2C as master
  
#if ARDUINO >= 157
  Wire.setClock(400000UL); // Set I2C frequency to 400kHz
#else
  TWBR = ((F_CPU / 400000UL) - 16) / 2; // Set I2C frequency to 400kHz
#endif

  i2cData[0] = 7; // Set the sample rate to 1000Hz - 8kHz/(7+1) = 1000Hz
  i2cData[1] = 0x00; // Disable FSYNC and set 260 Hz Acc filtering, 256 Hz Gyro filtering, 8 KHz sampling
  i2cData[2] = 0x00; // Set Gyro Full Scale Range to ±250deg/s
  i2cData[3] = 0x11; // Set Accelerometer Full Scale Range to ±16g
  while (i2cWrite(0x19, i2cData, 4, false)); // Write to all four registers at once
  while (i2cWrite(0x6B, 0x01, true)); // PLL with X axis gyroscope reference and disable sleep mode
  Serial.begin(115200);                                               
  
  //myPID.SetMode(AUTOMATIC);
  //myPID.SetSampleTime(400);
  //myPID.SetOutputLimits(-255,255);      
                       
       

  //*********************************************
  // set all the motor control pins to outputs
  pinMode(PWMA, OUTPUT);
  pinMode(PWMB, OUTPUT);
  pinMode(ForwardR, OUTPUT);
  pinMode(BackwardR, OUTPUT);
  pinMode(ForwardR, OUTPUT);
  pinMode(BackwardR, OUTPUT); 
  pinMode(2,INPUT);
  dt = micros() + T;                        
}

void loop(){
  A=digitalRead(A0);
  B=digitalRead(A1);
  C=digitalRead(A2);
  D=digitalRead(2);

if(A)ea=1;
if(B)eb=1;
if(C)ec=1;
if(D)ed=1;

if(!A && ea)
{
ea=0;
Kp+=5;
last=1;
}
if(!B && eb)
{
eb=0;
Ki+=0.2;
last=2;
}
if(!C && ec)
{
ec=0;
Kd+=0.1;
last=3;
}
if(!D && ed)
{
ed=0;
if(last==1)Kp-=5;
if(last==2)Ki-=0.2;
if(last==3)Kd-=0.1;
}

/*Serial.print("Kp = ");
Serial.print(Kp);

Serial.print("\tKi = ");
Serial.print(Ki);

Serial.print("\tKd = ");
Serial.println(Kd);*/

 angle = IMUReading()-5.055;
 
Serial.println(angle);

Input = angle;

//myPID.Compute();

//dt = micros() - dt;


d = (Input - pInput) / T ;

i = i + Input * T;
i = constrain(i, -100, 100);

Output = Kp * Input + Kd * d + Ki * i;
Output = constrain(Output, -255, 255);

pInput = Input;

// Serial.println(angle);
 if(abs(Output) < 70)
 power = 0;
 else power = abs(Output);
 //Serial.println(power);
 
if(angle < -2 & angle > -40){
  
  motor(1,2, power);
  motor(2,2, power);
}
else if(angle > 2 & angle < 40){
 
  motor(1,1,power);
  motor(2,1,power);
}
else{
  motor(1,0,0);
  motor(2,0,0);
}

while(dt>micros());

dt = micros()+T;
//number+=1;
//Serial.println(number);
}



double IMUReading(){
  /* Update all the values */
  while (i2cRead(0x3B, i2cData, 14));
  accX = ((i2cData[0] << 8) | i2cData[1]);
  accY = ((i2cData[2] << 8) | i2cData[3]);
  accZ = ((i2cData[4] << 8) | i2cData[5]);
  
  gyroX = (i2cData[8] << 8) | i2cData[9];
  gyroY = (i2cData[10] << 8) | i2cData[11];
  gyroZ = (i2cData[12] << 8) | i2cData[13];

  double dt = (double)(millis() - timer) / 1000; // Calculate delta time
  timer = millis();

  #ifdef RESTRICT_PITCH // Eq. 25 and 26
  double roll  = atan2(accY, accZ) * RAD_TO_DEG;
  double pitch = atan(-accX / sqrt(accY * accY + accZ * accZ)) * RAD_TO_DEG;
#else // Eq. 28 and 29
  double roll  = atan(accY / sqrt(accX * accX + accZ * accZ)) * RAD_TO_DEG;
  double pitch = atan2(-accX, accZ) * RAD_TO_DEG;
#endif

  double gyroXrate = gyroX / 131.0; // Convert to deg/s
  double gyroYrate = gyroY / 131.0; // Convert to deg/s

gyroXangle += gyroXrate * dt; // Calculate gyro angle without any filter
  gyroYangle += gyroYrate * dt;


   compAngleX = 0.93 * (compAngleX + gyroXrate * dt) + 0.07 * roll; // Calculate the angle using a Complimentary filter
  compAngleY = 0.93 * (compAngleY + gyroYrate * dt) + 0.07 * pitch;

#ifdef RESTRICT_PITCH
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((roll < -90 && compAngleX > 90) || (roll > 90 && compAngleX < -90)) {
  
    compAngleX = roll;
 
    gyroXangle = roll;
  } else
    
  if (abs(compAngleX) > 90)
    gyroYrate = -gyroYrate; // Invert rate, so it fits the restriced accelerometer reading
  
#else
  // This fixes the transition problem when the accelerometer angle jumps between -180 and 180 degrees
  if ((pitch < -90 && compAngleY > 90) || (pitch > 90 && compAngleY < -90)) {
  
    compAngleY = pitch;
  
    gyroYangle = pitch;
  } else
    
  if (abs(compAngleY) > 90)
    gyroXrate = -gyroXrate; // Invert rate, so it fits the restriced accelerometer reading
 
#endif



  

  // Reset the gyro angle when it has drifted too much
  if (gyroXangle < -180 || gyroXangle > 180)
    gyroXangle = compAngleX;
  if (gyroYangle < -180 || gyroYangle > 180)
    gyroYangle = compAngleY;

    return compAngleY;
   // delay(10);
}





void motor(int motor , int dir , int sped){
  switch(motor){
    case 1:
    if(dir == 0){
  digitalWrite(ForwardR, LOW);
  digitalWrite(BackwardR, LOW);
    }
    else if(dir == 1){
  digitalWrite(ForwardR, HIGH);
  digitalWrite(BackwardR, LOW);
  analogWrite(PWMA, sped);
    }
    else if(dir == 2){
  digitalWrite(ForwardR, LOW);
  digitalWrite(BackwardR, HIGH);
  analogWrite(PWMA, sped);
    }
    break;

    case 2:
    if(dir == 0){
  digitalWrite(ForwardL, LOW);
  digitalWrite(BackwardL, LOW);
    }
    else if(dir == 1){
  digitalWrite(ForwardL, HIGH);
  digitalWrite(BackwardL, LOW);
  analogWrite(PWMB, sped);
    }
    else if(dir == 2){
  digitalWrite(ForwardL, LOW);
  digitalWrite(BackwardL, HIGH);
  analogWrite(PWMB, sped);
    }
    break;
  }
 
}

