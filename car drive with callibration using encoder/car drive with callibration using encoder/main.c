#define F_CPU 14745600UL
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <math.h>
#include <stdbool.h>
#include "lcd.h"


uint8_t i,j;
bool temp_en = 0;
bool on = 0,flag = 1,flag1 = 1;
bool count_call = 0;
int frl = 200,frr = 200,bal =200,bar = 200;
long curr0,curr1,curr2,curr3,prev0,prev1,prev2,prev3;
long disp0=0,disp1=0,disp2=0,disp3=0,prev_disp0=0,prev_disp1=0,prev_disp2=0,prev_disp3=0;
long disp0p=0,disp1p=0,disp2p=0,disp3p=0,prev_disp0p=0;
float val0,val1,val2,val3;
long low = 0;

void callibration();
void timer_init_motor();
void delay_interrupt();
void inital_rpm();
void rpm();
uint16_t ADC_read(uint8_t ch);
void ADC_initiate();
void num_rotation();
void change_speed();


int main(void)
{
	DDRF = 0x00;
	DDRK = 0xFF;
	DDRE = 0xFF;
	DDRH = 0xFF;
	DDRL = 0xFF;
	PORTF = 0xFF;
	init_ports();
	lcd_init();
	timer_init_motor();
	ADC_initiate();
	delay_interrupt();
	sei();
	prev0 = ADC_read(0);
	prev1 = ADC_read(1);
	prev2 = ADC_read(2);
	prev3 = ADC_read(3);
	
    while (1)
    {
		callibration();
		lcd_print(1,1,val0,5);
		lcd_print(1,7,val1,5);
		lcd_print(2,1,val2,5);
		lcd_print(2,7,val3,5);
    }
}


void callibration()
{
	num_rotation();
	if(count_call == 1)
	{
		rpm();
		OCR4A = frl;
		OCR4B = bal;
		OCR4C = frr;
		OCR5B = bar;
		PORTK = 0x55;
		count_call = 0;
	}
}


void delay_interrupt()
{
	TCCR3A |= (1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);
	TCCR3B |= (1<<WGM32)|(1<<CS32)|(1<<CS30);
	OCR3A = 57600;
	TCNT3 = 0;
	TIMSK3 |= (1<<OCIE3A)|(1<<OCIE3B);
}


ISR (TIMER3_COMPA_vect)
{
	count_call = 1;
	TCNT3 = 0;
	OCR3A = 57600;
	PORTE ^= 0xFF;
}


void rpm()
{
	if(disp0 < 0)
	{
		disp0p = -disp0;
	}
	else
	{
		disp0p = disp0;
	}
	if(disp1 < 0)
	{
		disp1p = -disp1;
	}
	else
	{
		disp1p = disp1;
	}
	if(disp2 < 0)
	{
		disp2p = -disp2;
	}
	else
	{
		disp2p = disp2;
	}
	if(disp3 < 0)
	{
		disp3p = -disp3;
	}
	else
	{
		disp3p = disp3;
	}
	val0 = ((disp0p - prev_disp0)/1023)*15;
	val1 = ((disp1p - prev_disp1)/1023)*15;
	val2 = ((disp2p - prev_disp2)/1023)*15;
	val3 = ((disp3p - prev_disp3)/1023)*15;
	low = 10000;
	if(val0 < low)
	{
		low = val0;
	}
	if(val1 < low)
	{
		low = val1;
	}
	if(val2 < low)
	{
		low = val2;
	}
	if(val3 < low)
	{
		low = val3;
	}
	prev_disp0 = disp0p;
	prev_disp1 = disp1p;
	prev_disp2 = disp2p;
	prev_disp3 = disp3p;
	change_speed();
}


void num_rotation()
{
		curr0 = ADC_read(0);
		disp0 += (curr0 - prev0);
		if(curr0 - prev0 <=-500)
		{
			disp0 -= (curr0-prev0);
			disp0--;
		}
		else if(curr0 - prev0 >=500)
		{
			disp0 -= (curr0-prev0);
			disp0++;
		}
		prev0 = curr0;
		curr1 = ADC_read(1);
		disp1 += (curr1 - prev1);
		if(curr1 - prev1 <=-500)
		{
			disp1 -= (curr1-prev1);
			disp1--;
		}
		else if(curr1 - prev1 >=500)
		{
			disp1 -= (curr1-prev1);
			disp1++;
		}
		prev1 = curr1;
		curr2 = ADC_read(2);
		disp2 += (curr2 - prev2);
		if(curr2 - prev2 <=-500)
		{
			disp2 -= (curr2-prev2);
			disp2--;
		}
		else if(curr2 - prev2 >=500)
		{
			disp2 -= (curr2-prev2);
			disp2++;
		}
		prev2 = curr2;
		curr3 = ADC_read(3);
		disp3 += (curr3 - prev3);
		if(curr3 - prev3 <=-500)
		{
			disp3 -= (curr3-prev3);
			disp3--;
		}
		else if(curr3 - prev3 >=500)
		{
			disp3 -= (curr3-prev3);
			disp3++;
		}
		prev3 = curr3;
}


void timer_init_motor()
{
	TCCR4A |= (1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1)|(1<<WGM40);
	TCCR4B |= (1<<WGM42)|(1<<CS41)|(1<<CS40);
	TCCR5A |= (1<<COM5A1)|(1<<COM5B1)|(1<<COM5C1)|(1<<WGM50);
	TCCR5B |= (1<<WGM52)|(1<<CS51)|(1<<CS50);
	PORTH = 0xFF;
	PORTL = 0xFF;
}
//3 5
//4 4

void ADC_initiate()
{
	DDRF = 0x00;
	ADMUX = (0<<REFS1)|(1<<REFS0)|(0<<ADLAR);  // AVcc //  right adjusted
	ADCSRA = (1<<ADEN)|(0<<ADATE)|(0<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // bit4 ADC EOC flag // prescalar- 111 - 128 division factor
	ADCSRB = 0x00;
}


uint16_t ADC_read(uint8_t ch)
{
	ADMUX = ADMUX & 0b11100000;    //Clearing all the mux
	ADCSRB = ADCSRB & 0b11110111;  //------"-"-----------
	ch = ch & 0b00001111;
	if ( ch <= 7 )
	{
		ch = ch & 0b00000111; //
		ADMUX = ADMUX | ch;
		ADCSRB=0x00;
	}
	else
	{
		ch = ch-8;
		ch = ch & 0b00000111;
		ADMUX = ADMUX | ch;
		ADCSRB=0x00;
		ADCSRB = ADCSRB | (1<<MUX5);
	}
	ADCSRA = ADCSRA | (1<<ADSC);    //Bit 6 to start conversion-ADSC
	while( !(ADCSRA & (1<<ADIF)) ); // Wait for conversion to complete
	return(ADC);
}

void change_speed()
{
	if((val0 - low) >=20)
	{
		frl -= 20;
	}
	if((val1 - low) >=20)
	{
		frr -= 20;
	}
	if((val2 - low) >=20)
	{
		bal -= 20;
	}
	if((val3 - low) >=20)
	{
		bar -= 20;
	}
}