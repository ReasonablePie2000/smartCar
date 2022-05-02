#include <Arduino.h>
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels

// Declaration for an SSD1306 display connected to I2C (SDA, SCL pins)
#define OLED_RESET     28 //4 // Reset pin # (or -1 if sharing Arduino reset pin)
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
int oldV=1, newV=0;
#include <SoftwareSerial.h>
//UNO: (2, 3)
//SoftwareSerial mySerial(4, 6); // RX, TX
int pan = 90;
int tilt = 150;
int last_pan = 90;
long window_size = 0;
int BT_alive_cnt = 0;
int voltCount = 0;
#include <Servo.h>
Servo servo_pan;
Servo servo_tilt;
Servo servo_base;
Servo servo_left;
Servo servo_right;
Servo servo_claw;
int servo_min = 20;
int servo_max = 160;
bool found = false;
int k = 0;

int Rdone = 1;
int Ldone = 1;
unsigned long Rstart_time = 0;
unsigned long Lstart_time = 0;
long  RD, LD, tempR, tempL;
long Fdistance_in_cm = 100;
bool inStation = false;
bool isClose = false;
char charValue[] = "";

#define REcho A6
#define RTrig A7


unsigned long time;

//FaBoPWM faboPWM;
int pos = 0;
int MAX_VALUE = 2000;
int MIN_VALUE = 300;

// Define motor pins
#define PWMA 12    //Motor A PWM
#define DIRA1 34
#define DIRA2 35  //Motor A Direction
#define PWMB 8    //Motor B PWM
#define DIRB1 37
#define DIRB2 36  //Motor B Direction
#define PWMC 9   //Motor C PWM
#define DIRC1 43
#define DIRC2 42  //Motor C Direction
#define PWMD 5    //Motor D PWM
#define DIRD1 A4  //26  
#define DIRD2 A5  //27  //Motor D Direction

#define MOTORA_FORWARD(pwm)    do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,HIGH);analogWrite(PWMA,pwm);}while(0)
#define MOTORA_STOP(x)         do{digitalWrite(DIRA1,LOW); digitalWrite(DIRA2,LOW); analogWrite(PWMA,0);}while(0)
#define MOTORA_BACKOFF(pwm)    do{digitalWrite(DIRA1,HIGH);digitalWrite(DIRA2,LOW); analogWrite(PWMA,pwm);}while(0)

#define MOTORB_FORWARD(pwm)    do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,HIGH);analogWrite(PWMB,pwm);}while(0)
#define MOTORB_STOP(x)         do{digitalWrite(DIRB1,LOW); digitalWrite(DIRB2,LOW); analogWrite(PWMB,0);}while(0)
#define MOTORB_BACKOFF(pwm)    do{digitalWrite(DIRB1,HIGH);digitalWrite(DIRB2,LOW); analogWrite(PWMB,pwm);}while(0)

#define MOTORC_FORWARD(pwm)    do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,HIGH);analogWrite(PWMC,pwm);}while(0)
#define MOTORC_STOP(x)         do{digitalWrite(DIRC1,LOW); digitalWrite(DIRC2,LOW); analogWrite(PWMC,0);}while(0)
#define MOTORC_BACKOFF(pwm)    do{digitalWrite(DIRC1,HIGH);digitalWrite(DIRC2,LOW); analogWrite(PWMC,pwm);}while(0)

#define MOTORD_FORWARD(pwm)    do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,HIGH);analogWrite(PWMD,pwm);}while(0)
#define MOTORD_STOP(x)         do{digitalWrite(DIRD1,LOW); digitalWrite(DIRD2,LOW); analogWrite(PWMD,0);}while(0)
#define MOTORD_BACKOFF(pwm)    do{digitalWrite(DIRD1,HIGH);digitalWrite(DIRD2,LOW); analogWrite(PWMD,pwm);}while(0)

#define SERIAL  Serial
#define BTSERIAL Serial3

#define LOG_DEBUG

#ifdef LOG_DEBUG
#define M_LOG SERIAL.print
#else
#define M_LOG BTSERIAL.println
#endif

//PWM Definition
#define MAX_PWM   2000
#define MIN_PWM   100


int Motor_PWM = 300;


//    ↑A-----B↑
//     |  ↑  |
//     |  |  |
//    ↑C-----D↑
void BACK()
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);
  delay(15);
  STOP();
}

//    ↓A-----B↓
//     |  |  |
//     |  ↓  |
//    ↓C-----D↓
void ADVANCE()
{
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
  delay(15);
  STOP();
}
//    =A-----B↑
//     |   ↖ |
//     | ↖   |
//    ↑C-----D=
void LEFT_1()
{
  MOTORA_STOP(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_STOP(Motor_PWM);
}

//    ↓A-----B↑
//     |  ←  |
//     |  ←  |
//    ↑C-----D↓
void RIGHT_2()
{
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
  delay(15);
  STOP();
}
//    ↓A-----B=
//     | ↙   |
//     |   ↙ |
//    =C-----D↓
void LEFT_3()
{
  MOTORA_FORWARD(Motor_PWM); 
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
}
//    ↑A-----B=
//     | ↗   |
//     |   ↗ |
//    =C-----D↑
void RIGHT_1()
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);
}
//    ↑A-----B↓
//     |  →  |
//     |  →  |
//    ↓C-----D↑
void LEFT_2()
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_FORWARD(Motor_PWM);
  delay(15);
  STOP();
}
//    =A-----B↓
//     |   ↘ |
//     | ↘   |
//    ↓C-----D=
void RIGHT_3()
{
  MOTORA_STOP(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM); 
  MOTORD_STOP(Motor_PWM);
}

//    ↑A-----B↓
//     | ↗ ↘ |
//     | ↖ ↙ |
//    ↑C-----D↓
void rotate_1()  //tate_1(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_BACKOFF(Motor_PWM); 
  MOTORB_BACKOFF(Motor_PWM);
  MOTORC_BACKOFF(Motor_PWM); 
  MOTORD_BACKOFF(Motor_PWM);
  delay(15);
  STOP();
}

//    ↓A-----B↑
//     | ↙ ↖ |
//     | ↘ ↗ |
//    ↓C-----D↑
void rotate_2()  // rotate_2(uint8_t pwm_A,uint8_t pwm_B,uint8_t pwm_C,uint8_t pwm_D)
{
  MOTORA_FORWARD(Motor_PWM);
  MOTORB_FORWARD(Motor_PWM);
  MOTORC_FORWARD(Motor_PWM);
  MOTORD_FORWARD(Motor_PWM);
  delay(15);
  STOP();
}
//    =A-----B=
//     |  =  |
//     |  =  |
//    =C-----D=
void STOP()
{
  MOTORA_STOP(Motor_PWM);
  MOTORB_STOP(Motor_PWM);
  MOTORC_STOP(Motor_PWM);
  MOTORD_STOP(Motor_PWM);
}

float getDistanceF(){
  long Rduration;

  if(Rdone){
    Rdone = 0;
    Rstart_time = millis();
    digitalWrite(RTrig, LOW);
  }

  if(millis() > Rstart_time + 2){
    digitalWrite(RTrig, HIGH);  
  }

  if(millis() > Rstart_time + 10){
    digitalWrite(RTrig, LOW);
    Rduration = pulseIn(REcho, HIGH);
    tempR = (Rduration / 2.0) / 29.1;
     if(tempR < 30 and tempR > 0){
        Fdistance_in_cm = tempR; 
     }
    Rdone = 1;  
  }
  
}

void UART_Control()
{
  String myString;
  char BT_Data = 0;
  // USB data
  /****
   * Check if USB Serial data contain brackets
   */

  if (SERIAL.available())
  {
    char inputChar = SERIAL.read();
    if (inputChar == '(') { // Start loop when left bracket detected
      myString = "";
      inputChar = SERIAL.read();
      while (inputChar != ')')
      {
        myString = myString + inputChar;
        inputChar = SERIAL.read();
        if (!SERIAL.available()) {
          break;
        }// Break when bracket closed
      }
    }
    int commaIndex = myString.indexOf(','); //Split data in bracket (a, b, c)
    //Search for the next comma just after the first
    int secondCommaIndex = myString.indexOf(',', commaIndex + 1);
    String firstValue = myString.substring(0, commaIndex);
    String secondValue = myString.substring(commaIndex + 1, secondCommaIndex);
    String thirdValue = myString.substring(secondCommaIndex + 1); // To the end of the string
    
    if ((firstValue.toInt() > servo_min and firstValue.toInt() < servo_max) and  //Convert them to numbers
        (secondValue.toInt() > servo_min and secondValue.toInt() < servo_max)) {
      pan = firstValue.toInt();
      tilt = secondValue.toInt();
      window_size = thirdValue.toInt();
    }
    
    SERIAL.flush();
  }

}

void rotateCarCam(){
  if(window_size == 0){
    rotate_2();
  }
  
  if(pan>110){
    rotate_1();
    delay(10);
    moveCarCam();
  }else if(pan<110){
    rotate_2();
    delay(10);
    moveCarCam();
  }else{
    moveCarCam();  
  }
  
}

void moveCarCam(){
  if(window_size < 270000){
    if(Fdistance_in_cm < 15){
        BACK();
    }else{
      ADVANCE();
    }
    ADVANCE();
  }else if(window_size > 350000){
     BACK();
  }else{
    moveCarU();
    STOP();  
  }  
}

void moveCarU(){
  if(Fdistance_in_cm > 20){
    if(last_pan >= 110){
      for(int i = 0; i < 3; i++){
        RIGHT_2();
      }
    }else if(last_pan < 110){
      for(int i = 0; i < 3; i++){
        LEFT_2();
      }
    }
  }else if(Fdistance_in_cm > 18){
    
    ADVANCE();
    k = 0;
  }else if(Fdistance_in_cm < 15){
    BACK();  
    k = 0;
  }else{
    k++;
    if(k > 10){
      found = true;
    }
  }
   
}



//Robotic arm starts here ------------------------------------------------------------------------------------------------------------------------------------------------

#define SERVOS      (4)     // 机械臂需要的舵机个数


Servo arm_servos[SERVOS];   // 声明SERVOS个舵机
/*
  arm_servos[0]: pin 7 -- Servo base   底座舵机
  arm_servos[1]: pin 4 -- Servo left   左臂舵机
  arm_servos[2]: pin  3 -- Servo right  右臂舵机
  arm_servos[3]: pin  5 -- Servo claw   爪子舵机

  servo_base.write(75); // 75 middle, > left, < right
   servo_claw.write(50); // 20 close, < open, 
  servo_right.write(80); // decrease high
  servo_left.write(110); // incrase high
*/
int servo_pins[SERVOS] = {40,32,33,41};         
int servo_target_angle[SERVOS] = {75,170,30,60};  
int servo_min_angle[SERVOS] = {10, 70, 30, 40};  
int servo_max_angle[SERVOS] = {80, 180, 150, 70};   
int servo_init_angle[SERVOS] = {75,170,30,60};   
int servo_now_angle[SERVOS] = {75,170,30,60};  
int offset = 1;
int j = 0;



void initialization() {
  arm_servos[0] = servo_base;
  arm_servos[1] = servo_left;
  arm_servos[2] = servo_right;
  arm_servos[3] = servo_claw;

  init_servos();
}


void init_servos() {
  for (int i = 0; i < SERVOS; i++)
  {
    arm_servos[i].attach(servo_pins[i]);      // 把舵机关联到对应的PWM引脚上
    arm_servos[i].write(servo_init_angle[i]); // 写入舵机的初始角度
  }
}

//Where the program starts
void setup()
{
  SERIAL.begin(115200); // USB serial setup
  SERIAL.println("Start");
  STOP(); // Stop the robot
  //Serial3.begin(9600); // BT serial setup
  //Pan=PL4=>48, Tilt=PL5=>47
   servo_pan.attach(48);
   servo_tilt.attach(47);
   
   servo_base.attach(40);
   servo_left.attach(32);
   servo_right.attach(33);
   servo_claw.attach(41);
  //////////////////////////////////////////////
  //OLED Setup//////////////////////////////////
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { // Address 0x3C for 128x32
    Serial.println(F("SSD1306 allocation failed"));
  }
  display.clearDisplay();
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setCursor(0, 0);     // Start at top-left corner
  display.println("AI Robot");
  display.display();

  pinMode(RTrig, OUTPUT);
  pinMode(REcho, INPUT);

  initialization();
  
}

void updateArm(){
  arm_servos[1].write(servo_now_angle[1]);
  arm_servos[2].write(servo_now_angle[2]);
  UART_Control();
}


void getObj(){
  while(servo_now_angle[1] != servo_target_angle[1] or servo_now_angle[2] != servo_target_angle[2]){
    SERIAL.println("ojb-------------------");
    for (int i = 1; i < SERVOS - 1; i++){
      if(servo_now_angle[i] > servo_target_angle[i]){
          servo_now_angle[i]--; 
          SERIAL.println("before arm-------------------");
          updateArm();
          SERIAL.println("after arm-------------------");
      }else if(servo_now_angle[i] < servo_target_angle[i]){
          servo_now_angle[i]++;
          SERIAL.println("before arm2-------------------");
          updateArm();
          SERIAL.println("after arm2-------------------");
      }

    }

  }
}

void updateClaw(){
  while(servo_now_angle[3] != servo_target_angle[3]){
    if(servo_now_angle[3] > servo_target_angle[3]){
      offset = -1;
    }else if(servo_now_angle[3] < servo_target_angle[3]){
      offset = 1;
    }
    servo_now_angle[3] = servo_now_angle[3] + offset;
    arm_servos[3].write(servo_now_angle[3]);
    updateArm();
    delay(50);
  }
}

void updateBase(){
  while(servo_now_angle[0] != servo_target_angle[0]){
    if(servo_now_angle[0] > servo_target_angle[0]){
      offset = -1;
    }else if(servo_now_angle[0] < servo_target_angle[0]){
      offset = 1;
    }
    servo_now_angle[0] = servo_now_angle[0] + offset;
    arm_servos[0].write(servo_now_angle[0]);
    updateArm();
    delay(50);
  }
}

void controlArm(){
  if(servo_now_angle[1] != servo_target_angle[1] or servo_now_angle[2] != servo_target_angle[2]){
      getObj();
    }else if(servo_now_angle[0] != servo_target_angle[0]){  
      updateBase();
    }else if(servo_now_angle[3] != servo_target_angle[3]){  
      if(j == 1){
        for(int i = 0; i< 15; i++){
          ADVANCE();
        }
      }
      updateClaw();
    }else{
      if(j == 0){
        
        servo_target_angle[0] = 75;
        servo_target_angle[1] = 180;
        servo_target_angle[2] = 120;
        servo_target_angle[3] = 35;
        j++;
      }else if(j == 1){
        servo_target_angle[0] = 75;
        servo_target_angle[1] = 180;
        servo_target_angle[2] = 40;
        servo_target_angle[3] = 35;
        j++;
      }else if(j == 2){
        servo_target_angle[0] = 0;
        servo_target_angle[1] = 180;
        servo_target_angle[2] = 40;
        servo_target_angle[3] = 35;
        j++;
      }else if(j == 3){
        servo_target_angle[0] = 0;
        servo_target_angle[1] = 180;
        servo_target_angle[2] = 10;
        servo_target_angle[3] = 60;
        j++;
      }else if(j == 4){
        found = false;
        j = 0;
        Fdistance_in_cm = 100;
        for (int i = 0; i < SERVOS; i++)
        {
          servo_target_angle[i] = servo_init_angle[i];
          servo_now_angle[i] = servo_target_angle[i];
          arm_servos[i].write(servo_now_angle[i]);
        }
        delay(30000);
      }
      
    }
}


void loop()
{
  if(Fdistance_in_cm > 0 and Fdistance_in_cm < 30){
    moveCarU();
  }
  if(not found){
    rotateCarCam();
  }
  
  getDistanceF();
  display.clearDisplay();
  display.setTextSize(2);      // Normal 1:1 pixel scale
  display.setTextColor(SSD1306_WHITE); // Draw white text
  display.cp437(true);         // Use full 256 char 'Code Page 437' font
  display.setCursor(0, 0);     // Start at top-left corner
  display.println("Distance:");
  display.println(Fdistance_in_cm);
  display.display();
  
  // run the code in every 20ms

  time = millis();
  UART_Control(); //get USB and BT serial data

  //constrain the servo movement

  pan = constrain(pan, servo_min, servo_max);
  tilt = 140;//constrain(tilt, servo_min, servo_max); //From 120 --> 140

  if(pan > 110 or pan < 110){
    last_pan = pan;
  }
  
  //send signal to servo
  servo_pan.write(pan);
  servo_tilt.write(tilt);

  if(found){
    controlArm();
  }

}
