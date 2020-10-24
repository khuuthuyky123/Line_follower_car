#include <Servo.h>

int sensor1 = 6;   // Trái nhất
int sensor2 = 7;
int sensor3 = 8;
int sensor4 = 10;
int sensor5 = 11;  // Phải nhất

int IN1 = 5;
int IN2 = 4;
int IN3 = 3;
int IN4 = 2;

int servo = 9;
int echo =  12;
int trigger = 13 ;

int Kp = 30, Ki = 5, Kd = 2, Ks = 25;

int pos = 150;
int motor_speed = 210;

int error = 0;
int P = 0, I = 0, D = 0, PID_value = 0;
int chuyenLan=0;
int previous_error = 0, previous_I = 0;

Servo myservo;

void setup()
{
  Serial.begin(9600);
  pinMode(sensor1, INPUT);
  pinMode(sensor2, INPUT);
  pinMode(sensor3, INPUT);
  pinMode(sensor4, INPUT);
  pinMode(sensor5, INPUT);

  pinMode(IN1, OUTPUT);
  pinMode(IN2, OUTPUT);
  pinMode(IN3, OUTPUT);
  pinMode(IN4, OUTPUT);

  myservo.attach(servo);
  myservo.write(150);
  delay(15);
}

void loop()
{

  read_sensor_values();

  if (error == 103 && chuyenLan!=3)       // Nếu chỉ thấy màu trắng thì dừng lại
  {
    do
    {
      read_sensor_values();
      stop_bot();
      myservo.write(pos);
    } while (error == 103);

  }
  else
  {
      if (chuyenLan!=0)
      motor_speed = 100;
      else 
       motor_speed = 210;
     if (chuyenLan == 2)
     {
      
      delay(15);
        do
      {
        read_sensor_values();
        forward(150, 150);
        myservo.write(pos + 2*Ks);
      } while (error==102);
      chuyenLan=0;
      stop_bot();
      delay(200);
      int count=0;
      do 
      {
        read_sensor_values();
        forward(65,65);
        myservo.write(pos + error*Ks);
        delay(20);
        if (error==0) 
          count++;
      }while (count<10 || error==103);
      
     }
     else if (chuyenLan == -2)
     {
      
      delay(15);
        do
      {
        read_sensor_values();
        forward(150, 150);
        myservo.write(pos - 2*Ks);
      } while (error==102);
      chuyenLan=0;
      stop_bot();
      delay(200);
      int count=0;
      do 
      {
        read_sensor_values();
        forward(65,65);
        myservo.write(pos + error*Ks);
        delay(20);
        if (error==0) 
          count++;
      }while (count<10 || error==103);
      
     }
     else if (chuyenLan == 3)
     {
      while(1) {stop_bot();}
      stop_bot();
      delay(200);
      
      int count=0;
      do 
      {
        read_sensor_values();
        forward(65,65);
        myservo.write(pos + error*Ks);
        delay(20);
        if (error==0) 
          count++;
      }while (count<10);
      while(1) {stop_bot();}
      chuyenLan=0;
     }
    if (error == 102 && previous_error > 0 && previous_error <= 2) // nếu chỉ thấy màu đen mà trước đó đang đi cua trái thì tiếp tục cua trái đến khi thấy lại line trắng
    {
      do
      {
        read_sensor_values();
        forward(100, 100);
        myservo.write(pos + error*Ks);

      } while (error == 102);
    }
    else if (error == 102 && previous_error < 0 &&  previous_error >= -2) //// nếu chỉ thấy màu đen mà trước đó đang đi cua phải thì tiếp tục cua phải đến khi thấy lại line trắng
    {
      do
      {
        read_sensor_values();
        forward(100, 100);
        myservo.write(pos + error*Ks);

      } while (error == 102);
    }
    else
    {
      calculate_pid();
      motor_control();
    }
  }
}


void read_sensor_values()
{
  if (error <=2)
    previous_error = error;
  byte s1 = digitalRead(sensor1);
  byte s2 = digitalRead(sensor2);
  byte s3 = digitalRead(sensor3);
  byte s4 = digitalRead(sensor4);
  byte s5 = digitalRead(sensor5);
//  Serial.print(s1);
//    Serial.print(s2);
//    Serial.print(s3);
//    Serial.print(s4);
//    Serial.println(s5);
  if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 1)
    error = -2;
  else if ((s1 == 0 && s2 == 0 && s3 == 0 && s4 == 1 && s5 == 1)
           || (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 1 && s5 == 0))
    error = -1;
  else if ((s1 == 0 && s2 == 1 && s3 == 1 && s4 == 1 && s5 == 0)
           || (s1 == 0 && s2 == 0 && s3 == 1 && s4 == 0 && s5 == 0)
           || (s1 == 0 && s2 == 1 && s3 == 1 && s4 == 0 && s5 == 0)
           || (s1 == 0 && s2 == 0 && s3 == 1 && s4 == 1 && s5 == 0) )
    error = 0;
  else if ((s1 == 1 && s2 == 1 && s3 == 0 && s4 == 0 && s5 == 0)
           || (s1 == 0 && s2 == 1 && s3 == 0 && s4 == 0 && s5 == 0))
    error = 1;
  else if (s1 == 1 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 0)
    error = 2;
  else if (s1 == 0 && s2 == 0 && s3 == 0 && s4 == 0 && s5 == 0)
    error = 102;
  else if (s1 == 1 && s2 == 1 && s3 == 1 && s4 == 1 && s5 == 1)
    error = 103;
  if ((s1 == 0 && s2 == 0 && s3 == 1 && s4 == 1 && s5 == 1 
        ||  s1 == 0 && s2 == 1 && s3 == 1 && s4 == 1 && s5 == 1))
        {
          chuyenLan = -1;
        }
  else if ((s1 == 1 && s2 == 1 && s3 == 1 && s4 == 1 && s5 == 0 
        ||  s1 == 1 && s2 == 1 && s3 == 1 && s4 == 0 && s5 == 0))
       chuyenLan=1;
  if (chuyenLan!=0 && chuyenLan!=3 && error==102)
  {
    if (chuyenLan>0)
      chuyenLan=2;
    else 
      chuyenLan=-2;
  }
  if (chuyenLan!=0 && chuyenLan!=2 && error==103)
  {
    if (chuyenLan>0)
      chuyenLan=3;
    else 
      chuyenLan=-3;
  }
  Serial.println(chuyenLan);
  Serial.println(error);
}

void calculate_pid()
{
  P = error;
  I = I + previous_I;
  D = error - previous_error;

  PID_value = (Kp * P) + (Ki * I) + (Kd * D);

  previous_I = I;
}

void motor_control()
{
  myservo.write(pos + error * Ks);

  int left_motor_speed = motor_speed - PID_value;
  int right_motor_speed = motor_speed + PID_value;

  left_motor_speed = constrain(left_motor_speed, 0, 255);
  right_motor_speed = constrain(right_motor_speed, 0, 255);

  forward(left_motor_speed, right_motor_speed);
}

void forward(int SpeedA, int SpeedB)
{
  analogWrite(IN1, SpeedA);
  digitalWrite(IN2, LOW);
  analogWrite(IN3, SpeedB);
  digitalWrite(IN4, LOW);
}

void right(int SpeedA = HIGH)
{
  analogWrite(IN1, SpeedA);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}

void left(int SpeedB = HIGH)
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  analogWrite(IN3, SpeedB);
  digitalWrite(IN4, LOW);
}

void stop_bot()
{
  digitalWrite(IN1, LOW);
  digitalWrite(IN2, LOW);
  digitalWrite(IN3, LOW);
  digitalWrite(IN4, LOW);
}
