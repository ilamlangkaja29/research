//MKINVETIONS MADHAN KUMAR CHIRUGURI//
//FOLLOW ME ON FACEBOOK(MADHAN KUMAR CHIRUGURI)//
//FOLLOW ME ON ISTAGRAM(MADHAN CHIRUGURI)//
//FOLLOW ME ON TWITTER(MADHAN KUMAR CHIRUGURI)//
//I HOPE THIS CODE IS VERY HELPFULL TO YOU .//
//@@@@@THANK YOU@@@@@@@//

int cs;// CENTER SENSOR
int lmt1=5;// LEFT MOTOR 1
int lmt2=3;// LEFT MOTOR 2
int rmt1=6;// RIGHT MOTOR 1
int rmt2=11;// RIGHT MOTOR 2

void setup() {
  // put your setup code here, to run once:

pinMode(8,INPUT);
pinMode(lmt1,OUTPUT);
pinMode(lmt2,OUTPUT);
pinMode(rmt1,OUTPUT);
pinMode(rmt2,OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
 cs=digitalRead(8);
  
if(cs==LOW)
{
 stpleft();
}
else if(cs==HIGH)
{
  stpright();
}
}
void forward(){
  analogWrite(lmt1,150);
  analogWrite(lmt2,0);
  analogWrite(rmt1,150);
  analogWrite(rmt2,0);
  
}
void backward(){
  analogWrite(lmt1,0);
  analogWrite(lmt2,150);
  analogWrite(rmt1,0);
  analogWrite(rmt2,150);
}
void left (){
analogWrite(lmt1,0);
analogWrite(lmt2,150);
analogWrite(rmt1,150);
analogWrite(rmt2,0);
}
void right(){
  analogWrite(lmt1,150);
  analogWrite(lmt2,0);
  analogWrite(rmt1,0);
  analogWrite(rmt2,150);
}
void stp(){
 analogWrite(lmt1,0);
  analogWrite(lmt2,0);
  analogWrite(rmt1,0);
  analogWrite(rmt2,0); 
}
void stpright(){
analogWrite(lmt1,150);
  analogWrite(lmt2,0);
  analogWrite(rmt1,0);
  analogWrite(rmt2,0); 
}
void stpleft(){
analogWrite(lmt1,0);
  analogWrite(lmt2,0);
  analogWrite(rmt1,150);
  analogWrite(rmt2,0);   
}
