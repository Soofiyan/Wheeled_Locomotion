/*
 * x_wheel_drive.c
 *
 * Created: 13-09-2018 05:24:58 PM
 * Author : soofi
 */ 
#define F_CPU 14745600UL
#define BAUDRATE ((F_CPU/(BAUD*16UL)-1))
#define MISO PB3
#define MOSI PB2
#define SS PB0
#define BAUD 9600
#define SCK PB1
#define SPI_DDR DDRB
#define SPI_PORT PORTB
#include <avr/io.h>
#include <util/delay.h>
#include <avr/interrupt.h>
#include <stdbool.h>
#include <math.h>
#include "lcd.h"
int c = 0,ang_target = 0;
int tl,tr,bl,br;
uint8_t a = 0,b = 0;
uint16_t angles = 0,base_ang = 0;
int error_yaw = 0,error_add = 0,prev = 0;
int P = 0,I = 0,kp = 3,ki = 0.005;
bool currstatea1  = 0,currstateb1 = 0,currstatea2  = 0,currstateb2 = 0;
bool ang_flag = 0,firstc = 0;
int counteracw1 = 0,countercw1 = 0,counteracw2 = 0,countercw2 = 0;
void drivecross_4wheel(int vert,int hor,int rot);
void top_leftwheel(int speed,int l_limit,int h_limit);
void top_rightwheel(int speed,int l_limit,int h_limit);
void bottom_leftwheel(int speed,int l_limit,int h_limit);
void bottom_rightwheel(int speed,int l_limit,int h_limit);
long limit_var(long in_var, long l_limit, long h_limit);
void spi_init();
int spi_master_receive();
void spi_work();
void encoder_pid();
void exti_init();
void timer_init_motor();
void error_mag();
void delay_interrupt();
int main(void)
{
    while (1) 
    {
		spi_init();
		delay_interrupt();
		exti_init();
		timer_init_motor();
		if(ang_flag == 1)
		{
			encoder_pid();
		}
		drivecross_4wheel(100,0,error_yaw);
    }
}
void delay_interrupt()
{
	TCCR3A |= (1<<COM3A1)|(1<<COM3B1)|(1<<COM3C1);
	TCCR3B |= (1<<WGM32)|(1<<CS32)|(1<<CS30);
	OCR3A = 28800;
	TCNT3 = 0;
	TIMSK3 |= (1<<OCIE3A)|(1<<OCIE3B);
}
void drivecross_4wheel(int vert,int hor,int rot)
{
	tl = vert + hor + rot;
	tr = (-vert) + hor + rot;
	bl = vert + (-hor) + rot;
	br = (-vert) + (-hor) + rot;
	top_leftwheel(tl,-255,255);
	top_leftwheel(tr,-255,255);
	top_leftwheel(bl,-255,255);
	top_leftwheel(br,-255,255);
}
void top_leftwheel(int speed,int l_limit,int h_limit)
{
	speed = limit_var(speed,l_limit,h_limit);
	if(speed > 5)
	{
		PORTK |= (1<<0);
		PORTK &= ~(1<<1);
	}
	else if(speed < -5)
	{
		PORTK |= (1<<1);
		PORTK &= ~(1<<0);
		speed = -speed;
	}
	OCR4A = speed;
}
void top_rightwheel(int speed,int l_limit,int h_limit)
{
	speed = limit_var(speed,l_limit,h_limit);
	if(speed > 5)
	{
		PORTK |= (1<<2);
		PORTK &= ~(1<<3);
	}
	else if(speed < -5)
	{
		PORTK |= (1<<3);
		PORTK &= ~(1<<2);
		speed = -speed;
	}
	OCR4B = speed;	
}
void bottom_leftwheel(int speed,int l_limit,int h_limit)
{
	speed = limit_var(speed,l_limit,h_limit);
	if(speed > 5)
	{
		PORTK |= (1<<4);
		PORTK &= ~(1<<5);
	}
	else if(speed < -5)
	{
		PORTK |= (1<<5);
		PORTK &= ~(1<<4);
		speed = -speed;
	}
	OCR4C = speed;
}
void bottom_rightwheel(int speed,int l_limit,int h_limit)
{
	speed = limit_var(speed,l_limit,h_limit);
	if(speed > 5)
	{
		PORTK |= (1<<6);
		PORTK &= ~(1<<7);
	}
	else if(speed < -5)
	{
		PORTK |= (1<<7);
		PORTK &= ~(1<<6);
		speed = -speed;
	}
	OCR5B = speed;
}
long limit_var(long in_var, long l_limit, long h_limit)
{
	if (in_var>h_limit)
	{
		in_var=h_limit;
	}
	else if (in_var<l_limit)
	{
		in_var=l_limit;
	}
	return in_var;
}
void exti_init()
{
	EIMSK |=(1<<INT0)|(1<<INT1)|(1<<INT2)|(1<<INT3)|(1<<INT4);
	EICRA |=(1<<ISC00)|(1<<ISC10)|(1<<ISC20)|(1<<ISC30);
	EICRB |=(1<<ISC41);
}

void timer_init_motor()
{
	TCCR4A |= (1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1)|(1<<WGM40);
	TCCR4B |= (1<<WGM42)|(1<<CS41)|(1<<CS40);
}
ISR(INT1_vect)
{
	if((PIND & 0x02) == 0x02)
	{
		currstatea1 = 1;
		if(currstateb1 == 0)
		{
			countercw1 --;
		}
		else
		{
			countercw1 ++;
		}
	}
	else
	{
		currstatea1 = 0;
		if(currstateb1 == 0)
		{
			countercw1 ++;
		}
		else
		{
			countercw1 --;
		}
	}
}
ISR(INT0_vect)
{
	if((PIND & 0x01) == 0x01)
	{
		currstateb1 = 1;
		if(currstatea1 == 0)
		{
			countercw1 ++;
		}
		else
		{
			countercw1 --;
		}
	}
	else
	{
		currstateb1 = 0;
		if(currstatea1 == 0)
		{
			countercw1 --;
		}
		else
		{
			countercw1 ++;
		}
	}
}
ISR(INT3_vect)
{
	if((PIND & 0x08) == 0x08)
	{
		currstatea2 = 1;
		if(currstateb2 == 0)
		{
			countercw2 --;
		}
		else
		{
			countercw2 ++;
		}
	}
	else
	{
		currstatea2 = 0;
		if(currstateb2 == 0)
		{
			countercw2 ++;
		}
		else
		{
			countercw2 --;
		}
	}
}
ISR(INT2_vect)
{
	if((PIND & 0x04) == 0x04)
	{
		currstateb2 = 1;
		if(currstatea2 == 0)
		{
			countercw2 ++;
		}
		else
		{
			countercw2 --;
		}
	}
	else
	{
		currstateb2 = 0;
		if(currstatea2 == 0)
		{
			countercw2 --;
		}
		else
		{
			countercw2 ++;
		}
	}
}
void encoder_pid()
{
	spi_work();
	error_mag();
	P = error_yaw*kp;
	I += error_yaw*ki;
	error_add = P+I;
}
void error_mag()
{
	if(firstc == 0)
	{
		base_ang = angles;
	}
	error_yaw = (ang_target - angles);
	if((error_yaw >= 180) && (error_yaw <= 360))
	{
		error_yaw = error_yaw - 360;
	}
	else if((error_yaw <= -180) && (error_yaw >= -360))
	{
		error_yaw = error_yaw + 360;
	}
	else
	{
		error_yaw = error_yaw;
	}
}
void spi_init()
{
	SPI_DDR &= (~(1<<MOSI)) & (~(1<<SS)) & (~(1<<SCK));  //set it as slave
	SPI_DDR |= (1<<MISO);
	SPCR |= (1<<SPE);
}
int spi_master_receive()
{
	while(!(SPSR & (1<<SPIF)));
	c = SPDR;
	return c;
}
void spi_work()
{
	for(int i=0;i<=1;i++)
	{
		if(i == 0)
		{
			b = spi_master_receive();
		}
		else
		{
			a =  spi_master_receive();
		}
	}
	if(a == 0 && b != 0)
	{
		angles = b + 255;
	}
	else if(a != 0 && b == 0)
	{
		angles = a;
	}
	else if(a ==0 && b == 0)
	{
		angles = 0;
    }
	if(angles > 360)
	{
		angles = prev;
	}
	prev = angles;
}
ISR (TIMER3_COMPA_vect)
{
	ang_flag = 1;
}
