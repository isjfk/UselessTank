#include "delay.h"


void delay(u16 t)
{
	while(t--);
	return;
}

void delay_ns(u16 t)
{
	while(t--);
	return;
}

void delay_us(u16 t) 
{  
   while(t--) 
	{
      delay(6);    
   }	
}

void delay_ms(u16 t) 
{ 
   while(t--) 
		{
      delay_us(1000);    
   }
}














































