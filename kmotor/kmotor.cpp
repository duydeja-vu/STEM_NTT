#include "kmotor.h"
kmotor::kmotor(bool msg)
{
  _msg=msg;
}
void kmotor::cauhinh()
{
  pinMode(3,OUTPUT);
  pinMode(6,OUTPUT);
  pinMode(8,OUTPUT);
  pinMode(7,OUTPUT);
}
void kmotor::tien(int a,int b)
{
  if(a==0&&b>=0)
  {
    digitalWrite(7,HIGH);
    analogWrite(6,b);
  }
  if(a==0&&b<0)
 {
   digitalWrite(7,LOW);
    analogWrite(6,(-b));
 } 
   if(a==1&&b>=0)
  {
    digitalWrite(8,HIGH);
    analogWrite(3,b);
  }
  if(a==1&&b<0)
 {
   digitalWrite(8,LOW);
    analogWrite(3,(-b));
 }
}
void kmotor::run(int a,int b)
{
  if(a==0)
  {
	   digitalWrite(7,1);
	   analogWrite(6,b);  
	   digitalWrite(8,1);
	   analogWrite(3,b); 
  }
  if(a==1)
  {
	   digitalWrite(7,0);
	   analogWrite(6,b);  
	   digitalWrite(8,0);
	   analogWrite(3,b); 
  }
  if(a==2)
  {
	   digitalWrite(7,0);
	   analogWrite(6,b);  
	   digitalWrite(8,1);
	   analogWrite(3,b); 
  }
  if(a==3)
  {
	   digitalWrite(7,1);
	   analogWrite(6,b);  
	   digitalWrite(8,0);
	   analogWrite(3,b); 
  }
}
void kmotor::stop()
{
	   digitalWrite(7,1);
	   analogWrite(6,0);  
	   digitalWrite(8,1);
	   analogWrite(3,0); 
  }