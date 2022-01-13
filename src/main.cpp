/*    
           ------- bai tap lon Nhom 6 --------
        ####### code arduino tren platform IO ########
   De tai: he thong dieu chinh muc nuoc trong bon chua su dung 
                cam bien sieu am HC-SR04
        Mon: dieu khien va do luong bang may tinh
                                 GVHD: Lê Thị Ngọc Quyên

*/

#include <Arduino.h>

#define led_muc_1 A0
#define led_muc_2 5
#define led_muc_3 4
#define led_muc_4 3
#define led_muc_full 2

#define pin_van 12       
#define pin_gnd_bom 11       
#define pin_pwm_bom 10        

#define pin_bao_dong_tren 13
#define pin_bao_dong_duoi 9  

const int trig=8;
const int echo=7;

String inString="";
boolean stringComplete=false;
String commandString="";

float muc_1=6.0;
float muc_2=9.0;
float muc_3=12.0;
float muc_4=15.0;
float muc_full=18.0;
float muc_bao_dong_tren=19.0;
float muc_bao_dong_duoi=4.0;

float value_dk_led;
float value_muc_nuoc;
float value_muc;

float T;
float E,E1,E2,alpha,gamma,beta;
float Kp,Ki,Kd;
volatile float Output=0;
float LastOutput=0;

void setup() {
  Serial.begin(9600);
  pinMode(trig,OUTPUT);
  pinMode(echo,INPUT);

  pinMode(pin_gnd_bom,OUTPUT);
  pinMode(pin_pwm_bom,OUTPUT);
  pinMode(pin_van,OUTPUT);

  pinMode(led_muc_1,OUTPUT);
  pinMode(led_muc_2,OUTPUT);
  pinMode(led_muc_3,OUTPUT);
  pinMode(led_muc_4,OUTPUT);
  pinMode(led_muc_full,OUTPUT);
  pinMode(pin_bao_dong_tren,OUTPUT);
  pinMode(pin_bao_dong_duoi,OUTPUT);
  pinMode(6,OUTPUT);
  
  digitalWrite(pin_bao_dong_tren,HIGH);
  digitalWrite(pin_van,HIGH);
  digitalWrite(pin_bao_dong_duoi,HIGH);

  analogWrite(pin_pwm_bom,0);
  digitalWrite(pin_gnd_bom,LOW);
  digitalWrite(6,HIGH);

  // set thong so PID
    //thong so dieu khien cua pid
    Kp=157;Kd=5;Ki=9;
    //Kp=2;Kd=0.1;Ki=0.1;
    E=0;E1=0;E2=0;
    T=0.1; // thoi gian lay mau khoang 0.1 s
}

// bom va van
boolean get_pin_state()
{
  boolean state=false;
  if(inString.substring(4,6).equals("ON"))
  {
    state=true;
  }
  else if(inString.substring(4,6).equals("OF"))
  {
    state=false;
  }
  return state;
}

float get_muc_state()
{ 
  float value_1;
  if(inString.substring(4,6).equals("M1"))
  {
    value_1=muc_1;
  }
  else if(inString.substring(4,6).equals("M2"))
  {
    value_1=muc_2;
  }
  else if(inString.substring(4,6).equals("M3"))
  {
    value_1=muc_3;
  }
  else if(inString.substring(4,6).equals("M4"))
  {
    value_1=muc_4;
  }
   else if(inString.substring(4,6).equals("M5"))
  {
    value_1=muc_full;
  }
  else {
    value_1=inString.substring(4,8).toFloat();
  }
  return value_1;
}

void control_led()
{
  if(value_dk_led<=muc_bao_dong_duoi)
  {
    digitalWrite(pin_van,HIGH); 
  }
  if(value_dk_led<=muc_bao_dong_duoi+1.0)
  {
    digitalWrite(pin_bao_dong_duoi,LOW);
  }
  if(value_dk_led>muc_bao_dong_duoi+1.0)
  {
    digitalWrite(pin_bao_dong_duoi,HIGH);
  }
  if((value_dk_led<muc_1))
  {
    for(int i=2;i<=5;i++)
    {
      digitalWrite(i,LOW);
    }
    digitalWrite(led_muc_1,LOW);
  }
  if((value_dk_led>=muc_1) && (value_dk_led<muc_2))
  {
    digitalWrite(led_muc_1,HIGH);
    for(int i=2;i<=5;i++)
    {
      digitalWrite(i,LOW);
    }
  }
  if((value_dk_led>=muc_2) && (value_dk_led<muc_3))
  {
    digitalWrite(led_muc_1,LOW);
    digitalWrite(led_muc_2,HIGH);
    for(int i=2;i<=4;i++)
    {
      digitalWrite(i,LOW);
    }
  }
  if((value_dk_led>=muc_3) && (value_dk_led<muc_4))
  {
    digitalWrite(led_muc_1,LOW);
    digitalWrite(led_muc_2,LOW);
    digitalWrite(led_muc_3,HIGH);
    for(int i=2;i<=3;i++)
    {
      digitalWrite(i,LOW);
    }
  }
  if((value_dk_led>=muc_4) && (value_dk_led<muc_full))
  {
    digitalWrite(led_muc_1,LOW);
    digitalWrite(led_muc_4,HIGH);
    digitalWrite(led_muc_full,LOW);
    for(int i=4;i<=5;i++)
    {
      digitalWrite(i,LOW);
    }
  }
  if((value_dk_led>=muc_full))
  {
    digitalWrite(led_muc_1,LOW);
    digitalWrite(led_muc_full,HIGH);
    for(int i=3;i<=5;i++)
    {
      digitalWrite(i,LOW);
    }
  }
  else if(value_dk_led>=muc_bao_dong_tren)
  {
    analogWrite(pin_pwm_bom,0);
  }
  if(value_dk_led>=muc_bao_dong_tren-1.0)
  {
    digitalWrite(pin_bao_dong_tren,LOW);
  }
  else if(value_dk_led<muc_bao_dong_tren-1.0)
  {
    digitalWrite(pin_bao_dong_tren,HIGH);
  }
}

void muc_nuoc()
{
  float read_sum=0;
  float distance=0;
  unsigned long duration;
  for(int i=0;i<10;i++)
  {
    digitalWrite(trig,0);
    delayMicroseconds(5);
    digitalWrite(trig,1);
    delayMicroseconds(10);
    digitalWrite(trig,0);
    duration=pulseIn(echo,HIGH);
    distance=float(duration/2/29.41);
    read_sum=read_sum+distance;
    delay(30);
  }
  value_muc_nuoc=read_sum/10.0;
  value_dk_led=23.5-value_muc_nuoc;
  Serial.println(value_dk_led);
  delay(100);
}

void serialEvent()
{
  while(Serial.available()>0)
  {
    char val=(char)Serial.read();
    inString+=val;
    if(val=='\n')
    {
      stringComplete=true;
    }
  }
}

void get_Command()
{
  if(inString.length()>0)
  {
    commandString=inString.substring(1,4);
  }
}

void control_pin()
{
  value_muc=get_muc_state();
   if(value_muc-0.1>=value_dk_led)
    {
      analogWrite(pin_pwm_bom,255);
      digitalWrite(pin_van,HIGH);
    }
    else if(value_muc+0.1<=value_dk_led)
    {
      analogWrite(pin_pwm_bom,0);
      digitalWrite(pin_van,LOW);
    }
    else 
    {
      digitalWrite(pin_van,HIGH);
      analogWrite(pin_pwm_bom,0);
      value_muc=0;
    }
}

void control_PID()
{
  while(!commandString.equals("NOP"))
  {
    serialEvent();
    if(stringComplete)
    {
      stringComplete=false;
      get_Command();
      inString="";
    }
    muc_nuoc();
    control_led();
    float value_pid=get_muc_state();

    E=value_pid-value_dk_led;
    alpha=2*T*Kp+Ki*T*T+2*Kd;
    beta=T*T*Ki-4*Kd-2*T*Kp;
    gamma=Kd*2;
    Output=(alpha*E+beta*E1+gamma*E2+2*T*LastOutput)/(2*T);
    LastOutput=Output;
    E2=E1;
    E1=E;
    // thi khi xung cho mach cau l298N cho dong co quay
    if(value_pid+0.05>=value_dk_led)
    {
      
        if(Output>255||(Output<=255&&Output>=60))
        {
          Output=255;
        }
        if(Output>0 && Output<60)
        {
          Output=75;
        }
        if(Output<0)
        {
          Output=0;
          alpha=0;
          beta=0;
          gamma=0;
          LastOutput=0;
        }
        noInterrupts();
        analogWrite(pin_pwm_bom,Output);
        interrupts();
    }
    else if(value_pid+0.05<value_dk_led)
    {
      analogWrite(pin_pwm_bom,0);
    }
  } 
}

void xu_li()
{
  if(stringComplete)
  {
    stringComplete=false;
    get_Command();
    if(commandString.equals("STA"))
    {
    }
    else if(commandString.equals("STO"))
    {
      analogWrite(pin_pwm_bom,0);
      digitalWrite(pin_van,HIGH);
    }
    else if(commandString.equals("BOM"))
    {
      boolean bom_state=get_pin_state();
      if(bom_state==true)
      {
        analogWrite(pin_pwm_bom,255);
      }
      else {
        analogWrite(pin_pwm_bom,0);
      }
    }
    else if(commandString.equals("VAN"))
    {
      boolean van_state=get_pin_state();
      if(van_state==true)
      {
        digitalWrite(pin_van,LOW);
      }
      else{
        digitalWrite(pin_van,HIGH);
      }
    } 
    else if(commandString.equals("MUC"))
    {
      control_pin();
    }
    else if(commandString.equals("PID"))
    {
      control_PID();
    }
    else if(commandString.equals("SE1"))
    {
      muc_bao_dong_tren=get_muc_state();
    }
    else if(commandString.equals("SE2"))
    {
      muc_bao_dong_duoi=get_muc_state();
    }
    else if(commandString.equals("RES"))
    {
      muc_bao_dong_tren=19.0;
      muc_bao_dong_duoi=4.0;
    }
  inString="";
  }
}
void loop() {
  muc_nuoc();
  serialEvent();
  xu_li();
  control_led();
}