#include <MPULib.h>
#include <Wire.h>


//PID VALUES:

float Kp = 30;                    
float Ki = 0.61;                  
float Kd = 9;                     
float Moving_Speed = 20;         
float Max_Speed = 160;            
int Acc_Offset = -6893;           

//VARIABLES:

byte Activated, Received_byte;
int left_motor;
int Left_Motor_Speed;
int CC_Speed_Left_Motor; 
int Left_Motor_Speed_Prev;
int right_motor;
int Right_Motor_Speed;
int CC_Speed_Right_Motor;
int Right_Motor_Speed_Prev;
int Received_Since;
unsigned long Loop_Time;
float Gyro_Angle, Auto_Setpoint;
float Temp_Error, PID_I, Setpoint, gyro_input, PID_Value, Last_D_Error;
float PID_Value_left, PID_Value_right;                        
MPULib mpulib(Acc_Offset, Activated);



//Input/Output:

int DIR_L = 2;      
int STEP_L = 3;     
int DIR_R = 4;      
int STEP_R = 5;     
int Enable = 6;    

void setup() {
  Serial.begin(9600);       
  Wire.begin();             
  TWBR = 12;                

  pinMode(DIR_L, OUTPUT);
  pinMode(STEP_L, OUTPUT);
  pinMode(DIR_R, OUTPUT);
  pinMode(STEP_R, OUTPUT);
 
  TCCR2A = 0;               
  TCCR2B = 0;               
  TIMSK2 |= (1 << OCIE2A);  
  TCCR2B |= (1 << CS21);    
  OCR2A = 39;               
  TCCR2A |= (1 << WGM21);   
  mpulib.setupMPU();
  mpulib.calcOffSet();
  
  delay(200);   
  digitalWrite(13, HIGH);
  delay(1000);
  digitalWrite(13, LOW);
  delay(100);

  Loop_Time = micros() + 4000;                            
}
void loop() {
  mpulib.calcAngle();

  Gyro_Angle = mpulib.Gyro_Angle;
  Auto_Setpoint = mpulib.Auto_Setpoint;
  Temp_Error = Gyro_Angle - Auto_Setpoint - Setpoint;

  if (PID_Value > 10 || PID_Value < -10) {
    Temp_Error += PID_Value * 0.015 ;
  }
  
  PID_I += Ki * Temp_Error;                                                 
  if (PID_I > 400)PID_I = 400;                                             
  else if (PID_I < -400)PID_I = -400;

  PID_Value = Kp * Temp_Error + PID_I + Kd * (Temp_Error - Last_D_Error);
  if (PID_Value > 400)PID_Value = 400;                                      
  else if (PID_Value < -400)PID_Value = -400;

  Last_D_Error = Temp_Error;
                                                  
  if (PID_Value < 6 && PID_Value > - 6)PID_Value = 0;                      
  if (Gyro_Angle > 30 || Gyro_Angle < -30 || Activated == 0) {              
    PID_Value = 0;                                                         
    PID_I = 0;                                                              
    Activated = 0;                                                          
    Auto_Setpoint = 0;                                                      
  }
  
//HC-05 :

  PID_Value_left = PID_Value;                             
  PID_Value_right = PID_Value;                             

  if (Received_byte & B00000001) {                         
    PID_Value_left += Moving_Speed;                        
    PID_Value_right -= Moving_Speed;                        
  }
  if (Received_byte & B00000010) {                         
    PID_Value_left -= Moving_Speed;                        
    PID_Value_right += Moving_Speed;                       
  }

  if (Received_byte & B00000100) {                          
    if (Setpoint > -2.5)Setpoint -= 0.05;                   
    if (PID_Value > Max_Speed * -1)Setpoint -= 0.005;      
  }
  if (Received_byte & B00001000) {                          
    if (Setpoint < 2.5)Setpoint += 0.05;                   
    if (PID_Value < Max_Speed)Setpoint += 0.005;            
  }

  if (!(Received_byte & B00001100)) {                      
    if (Setpoint > 0.5)Setpoint -= 0.05;                    
    else if (Setpoint < -0.5)Setpoint += 0.05;             
    else Setpoint = 0;                                      
  }

  if (Setpoint == 0) {                                     
    if (PID_Value < 0)Auto_Setpoint += 0.001;               
    if (PID_Value > 0)Auto_Setpoint -= 0.001;              
  }



  //MOTORS CONTROL:
  
  if (PID_Value_left > 0){
    PID_Value_left = 405 - (1 / (PID_Value_left + 9)) * 5500;
  }
  else if (PID_Value_left < 0){
    PID_Value_left = -405 - (1 / (PID_Value_left - 9)) * 5500;
  }
  if (PID_Value_right > 0){
    PID_Value_right = 405 - (1 / (PID_Value_right + 9)) * 5500;
  }
  else if (PID_Value_right < 0){
    PID_Value_right = -405 - (1 / (PID_Value_right - 9)) * 5500;
  }

  if (PID_Value_left > 0){
    left_motor = 400 - PID_Value_left;
  }
  else if (PID_Value_left < 0){
    left_motor = -400 - PID_Value_left;
  }
  else left_motor = 0;

  if (PID_Value_right > 0){
    right_motor = 400 - PID_Value_right;
  }
  else if (PID_Value_right < 0){
    right_motor = -400 - PID_Value_right;
  }
  else right_motor = 0;
  
  Left_Motor_Speed = left_motor;
  Right_Motor_Speed = right_motor;
  while (Loop_Time > micros());
  Loop_Time += 4000;
}


ISR(TIMER2_COMPA_vect) {
  
  CC_Speed_Left_Motor ++;                                       
  if (CC_Speed_Left_Motor > Left_Motor_Speed_Prev) {            
    CC_Speed_Left_Motor = 0;                                    
    Left_Motor_Speed_Prev = Left_Motor_Speed;                  
    if (Left_Motor_Speed_Prev < 0) {                           
      PORTD &= 0b11111011;                                     
      Left_Motor_Speed_Prev *= -1;                             
    }
    else PORTD |= 0b00000100;                                   
  }
  else if (CC_Speed_Left_Motor == 1)PORTD |= 0b00001000;        
  else if (CC_Speed_Left_Motor == 2)PORTD &= 0b11110111;        

  CC_Speed_Right_Motor ++;                                      
  if (CC_Speed_Right_Motor > Right_Motor_Speed_Prev) {          
    CC_Speed_Right_Motor = 0;                                  
    Right_Motor_Speed_Prev = Right_Motor_Speed;                 
    if (Right_Motor_Speed_Prev < 0) {                           
      PORTD &= 0b11101111;                                      
      Right_Motor_Speed_Prev *= -1;                             
    }
    else PORTD |= 0b00010000;                                   
  }
  else if (CC_Speed_Right_Motor == 1)PORTD |= 0b00100000;       
  else if (CC_Speed_Right_Motor == 2)PORTD &= 0b11011111;       
}
