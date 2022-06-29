#include <ezButton.h>
#include <Servo.h>
#include <NewPing.h>
ezButton toggleSwitch(3);  
int pos = 0;    
int pump=2;
int ser=11;
Servo servo_motor;
//flame initialize
 
int FlamePinL = 8;
int FlameL;
int FlamePin1L = 12;
int Flame1L;
int FlamePinC = 13;
int FlameC;
int FlamePin1R = 9;
int Flame1R;
int FlamePinR = 10;
int FlameR; 

//ultrasonic initialize

  
int trigPinL = A5;
int echoPinL = A4;

int trigPin = A3;
int echoPin = A2;

int trigPinR = A1;
int echoPinR = A0;
int maximumcm = 500;

NewPing sonarL(trigPinL, echoPinL, maximumcm);
NewPing sonar(trigPin, echoPin, maximumcm);
NewPing sonarR(trigPinR, echoPinR, maximumcm);


//motor initialize
int in4 = 5;
int in3 = 4;
int in2 = 6;
int in1 = 7;
int cmC;
int cmR;
int cmL;
int a=0;

//setup code here
void setup() 
{
 pinMode(pump, OUTPUT);
 digitalWrite(pump,LOW);
  pinMode(trigPinL, OUTPUT);
  pinMode(echoPinL, INPUT);

  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);

  pinMode(trigPinR, OUTPUT);
  pinMode(echoPinR, INPUT);
  
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
   

  pinMode(FlamePinL, INPUT);
  pinMode(FlamePin1L, INPUT);
  pinMode(FlamePinC, INPUT);
  pinMode(FlamePinR, INPUT);
  pinMode(FlamePin1R, INPUT);

  servo_motor.attach(ser); 

  Serial.begin(9600);

}
int iC=0;
int iL=0;
int iR=0;
void loop() 
{
  if(digitalRead(3)==HIGH)
  { 
 // Hardreset();

   Serial.print("start");
   
      FlameL = digitalRead(FlamePinL);
      Flame1L = digitalRead(FlamePin1L);
      FlameC = digitalRead(FlamePinC);
      FlameR = digitalRead(FlamePinR);
      Flame1R = digitalRead(FlamePin1R);
    if((FlameC==1)||(FlameL==1)||(Flame1L==1)||(FlameR==1)||(Flame1R==1) )
    {     Serial.print("phase1");
      detectfire();
    } 
    else
    {
      for(int i=0;i<=10;i++)
      { Serial.print("phase2");
        detectfire();
        Serial.print("phase3");
      //Move();
        //delay(10);
        Serial.print("i = ");Serial.println(i);
       // if(cmC==0){Serial.print("cmC = ");Serial.println(cmC);Serial.print("iC = ");iC++;Serial.println(iC);}
      //  if(cmL==0){Serial.print("cmL = ");Serial.println(cmL);Serial.print("iL = ");iL++;Serial.println(iL);}
     //   if(cmR==0){Serial.print("cmR = ");Serial.println(cmR);Serial.print("iR = ");iR++;Serial.println(iR);}
      }
      Stop();
     Serial.print("phase4");
     
       digitalWrite(pump,HIGH);
      servo();
      Serial.print("phase5");
    }
  }
  else if(digitalRead(3)==LOW)
  {
   reset();
  }
}


//functions

//Function to move servo
void servo()
{  
      a=0;
      FlameL = digitalRead(FlamePinL);
      Flame1L = digitalRead(FlamePin1L);
      FlameC = digitalRead(FlamePinC);
      FlameR = digitalRead(FlamePinR);
      Flame1R = digitalRead(FlamePin1R); 
  
      while(((FlameC== 0)&&(FlameL== 0)&&(Flame1L== 0)&&(FlameR== 0)&&(Flame1R==0))&&(a<80))
      { 
        
        servo_motor.write(45);
        delay(100);
        Serial.print("a = ");Serial.println(a);
        a++;
        servo_motor.write(90);
        delay(0.0125);
  
        FlameL = digitalRead(FlamePinL);
        Flame1L = digitalRead(FlamePin1L);
        FlameC = digitalRead(FlamePinC); 
        FlameR = digitalRead(FlamePinR);
        Flame1R = digitalRead(FlamePin1R); 
      }
      if(a>=79)
      {
        servo_motor.write(135);
        delay(7800);
        a=0;
        Serial.println(a);
        servo_motor.write(90);
        delay(0.0125);
      }
      else
      {
        Stop();
       detectfire();
       digitalWrite(pump,HIGH);delay(1);
       servo_motor.write(90);
       delay(100);
       servo_motor.write(135);
       delay(a*100);
       servo_motor.write(90); 
       delay(1); 
       a=0;
      }
}

//Function to move robot
void Move()
{
  delay(50);
  cmC = readPing();Serial.print("cmC = ");Serial.println(cmC);
  delay(50);
  cmL = readPingL();Serial.print("cmL = ");Serial.println(cmL);
  delay(50);
  cmR = readPingR();Serial.print("cmR = ");Serial.println(cmR); 
  while((cmC==0)||(cmL==0)||(cmR==0))
  {
    Stop();
    cmC = readPing();
    cmL = readPingL();
    cmR = readPingR();
    Serial.print("loop");
  }
  if ( cmC < 50 )
  {
    Stop();
    delay(500);
    Backward();
    delay(100);
    if(cmL>cmR)
    {
      Left();Serial.println("BackwardR");
      delay(100);  
    }
    else
    {
      Right();Serial.println("BackwardL");
      delay(100);    
    }
    Stop();
  }
  else if (cmR < 40 )
  {
    Stop();
    delay(100);
    Left();Serial.println("Left");
    delay(100);
    Stop();
  }
  else if (cmL <40)
  {
    Stop();
    delay(100);
    Right();Serial.println("Right");  
    delay(100);
    Stop();
  }
  else
  {
    Forward();Serial.println("forward");  
    delay(100);
   
  }
}

//Function to stop robot
void Stop()
{
  digitalWrite(in1, LOW);
  digitalWrite(in3, LOW);
  analogWrite(in2, 0);
  analogWrite(in4, 0);
  return;
}
//Function to move forward robot
void Forward()
{
  digitalWrite(in1,LOW);
  digitalWrite(in3,LOW);
  analogWrite(in2, 128);
  analogWrite(in4, 128);
  delay(100);
  return; 
}
//Function to move backwards robot
void Backward()
{
  digitalWrite(in1,HIGH);
  digitalWrite(in3, HIGH);
  analogWrite(in2, 128);
  analogWrite(in4, 128);
  delay(500);
  return; 
}


//Function to move Right robot
void Right()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in3,LOW);
  analogWrite(in2, 30);//50
  analogWrite(in4, 225);//205
  delay(5000);
  return; 
}
//Function to move Left robot
void Left()
{
  digitalWrite(in1, LOW);
  digitalWrite(in3,HIGH);
  analogWrite(in2, 225);//200
  analogWrite(in4, 30);//50
  delay(5000);
  return; 
}
void RightFire()
{
  digitalWrite(in1, HIGH);
  digitalWrite(in3,LOW);
  analogWrite(in2, 30);//50
  analogWrite(in4, 225);//205
  delay(1000);
  return; 
}
//Function to move Left robot
void LeftFire()
{
  digitalWrite(in1, LOW);
  digitalWrite(in3,HIGH);
  analogWrite(in2, 225);//200
  analogWrite(in4, 30);//50
  delay(1000);
  return; 
}
//Function to reset robot
void reset()
{
  Stop();
   digitalWrite(pump,HIGH);
   delay(1);
  servo_motor.write(90);
  delay(1);
}
//
int readPing()
{
  int cm = sonar.ping_cm();
  delay(100);
  return cm;
  
}
//
int readPingL()
{
  int cm = sonarL.ping_cm();
  return cm;
}
//
int readPingR()
{
  int cm = sonarR.ping_cm();
  delay(100);
  return cm;
}
//
void detectfire()
{
    FlameL = digitalRead(FlamePinL);Serial.println("FlamePinL = ");Serial.println( FlameL );
    Flame1L = digitalRead(FlamePin1L);Serial.println("FlamePin1L = ");Serial.println( Flame1L );
    FlameC = digitalRead(FlamePinC); Serial.println("FlamePinC = ");Serial.println(FlameC);
    FlameR = digitalRead(FlamePinR);Serial.println("FlamePinR = ");Serial.println( FlameR );
    Flame1R = digitalRead(FlamePin1R);Serial.println("FlamePin1R = ");Serial.println( Flame1R );
  //delay(5000);
   if (FlameC== 1)
  { 
    Serial.println("HIGH FLAMEC");
   Stop();
   while(FlameC==1)
   {
    FlameC = digitalRead(FlamePinC); Serial.println(FlameC);
 digitalWrite(pump,LOW); 
 delay(500);
 
 FlameC = digitalRead(FlamePinC); Serial.println(FlameC);
    }digitalWrite(pump,HIGH);delay(1);
  }
  
  else if(FlameL== 1 )
  { Serial.println("HIGH FLAMEL");
  Stop();
    while(FlameC==0)
    {
      FlameL = digitalRead(FlamePinL);
      LeftFire();Serial.print("lefty");
      FlameC = digitalRead(FlamePinC);
       if(FlameC==1){Stop();}
    }
    while(FlameC==1)
   {
    FlameC = digitalRead(FlamePinC); Serial.println(FlameC);
  digitalWrite(pump,LOW); 
  delay(500);
 
 FlameC = digitalRead(FlamePinC); 
 Serial.println(FlameC);
    }digitalWrite(pump,HIGH);delay(1);
    }
    
   else if(Flame1L== 1 )
  { Serial.println("HIGH FLAME1L");
 Stop();
    while(FlameC==0)
    {
      Flame1L = digitalRead(FlamePin1L);
      LeftFire();Serial.print("lefty1");
      FlameC = digitalRead(FlamePinC);
       if(FlameC==1){Stop();}
    }
    while(FlameC==1)
   {
    FlameC = digitalRead(FlamePinC); Serial.println(FlameC);
  digitalWrite(pump,LOW);
 delay(500);

 FlameC = digitalRead(FlamePinC); 
 Serial.println(FlameC);
    }digitalWrite(pump,HIGH);delay(1);
  }
  
   else if(FlameR== 1 )
  { Serial.println("HIGH FLAMER");
   Stop();
    while(FlameC==0)
    {
      FlameR = digitalRead(FlamePinR);
      RightFire();Serial.print("Righty");
      FlameC = digitalRead(FlamePinC);
      if(FlameC==1){Stop();}
    }
    while(FlameC==1)
   {
    FlameC = digitalRead(FlamePinC); Serial.println(FlameC);
 digitalWrite(pump,LOW);
 delay(500);

 FlameC = digitalRead(FlamePinC); 
 Serial.println(FlameC);
    }digitalWrite(pump,HIGH);delay(1);
  } 
  
  else if(Flame1R== 1 )
  { Serial.println("HIGH FLAME1R");
   Stop();
    while(FlameC==0)
    {
      Flame1R = digitalRead(FlamePin1R);
      RightFire();Serial.print("Righty1");
      FlameC = digitalRead(FlamePinC);
       if(FlameC==1){Stop();}
    }
    while(FlameC==1)
   {
    FlameC = digitalRead(FlamePinC); Serial.println(FlameC);
  digitalWrite(pump,LOW);
 delay(500);

 FlameC = digitalRead(FlamePinC); 
  Serial.println(FlameC);
    }digitalWrite(pump,HIGH);delay(1);
  }
      FlameL = digitalRead(FlamePinL);
      Flame1L = digitalRead(FlamePin1L);
      FlameC = digitalRead(FlamePinC);
      FlameR = digitalRead(FlamePinR);
      Flame1R = digitalRead(FlamePin1R);
      digitalWrite(pump,HIGH);delay(1);
}

void Hardreset()
{
  
  servo_motor.write(45);
  delay(2000);
  servo_motor.write(90);
  delay(1000); 
}
