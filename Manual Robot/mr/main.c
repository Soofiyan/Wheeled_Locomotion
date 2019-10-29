/*
 * ps2 usart.c
 *
 * Created: 01-11-2017 02:57:43 PM
 * Author : Soofiyan Atar
 */
#define F_CPU 14745600UL
#define BAUD 9600
#define microstep 2
#define steps 1680
#define rpm 0.22
#define kp	1
#define kpg 3.25
#define kig 0.009
#define kpg_rot 1.42
#define BAUDRATE ((F_CPU/(BAUD*16UL)-1))
#define rot_range 75
#define pwm_range 200
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
void usart_init();
long map_value(long in_value, long in_min, long in_max, long out_min, long out_max);
void ADC_initiate();
uint16_t ADC_read(uint8_t ch);
void enrun();
void en_pid();
void servorun();
void usart_init1();
void drivecross(int x_vect, int y_vect , int m_vect , int errg_vect , int errg_side);
void gyro();
void rackrun();
void receive();
void encoder_setup();
void drivewheel_en(long sp_vect, long l_lim, long h_lim);
void drivewheel_r(long sp_vect, long l_lim, long h_lim);
void drivewheel_rextra(long sp_vect, long l_lim, long h_lim);
void drivewheel_grip(long sp_vect, long l_lim, long h_lim);
void main1();
void while_wala();
void drivewheel_1(long sp_vect, long l_lim, long h_lim);
void drivewheel_2(long sp_vect, long l_lim, long h_lim);
void drivewheel_3(long sp_vect, long l_lim, long h_lim);
long limit_var(long in_var, long l_limit, long h_limit);
int butt[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int x=0,y=0,z=0;
int RX_raw=-1,RX_ad=-1,m=0,RX_ad1=-1,RX_range=200;
int xj1=0,yj1=0,xj2=0,yj2=0,x_vect=0,y_vect =0,e=1;
uint8_t RX[16]={100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100};
int flag1=0,flag =0,errg=0,f_pwmg=0,f_pwmg1=0,f_pwmg2=0,f_pwmg3 =0,g=0,intg=0,pwmg=0;
int servomode=0,ip=0,ip1=0,enmode_prev = 1,enmode_curr=1,dispensing = 0,gyromode = 1000,gripmode = 0,gyro_rot = 0,rackmode=0,jg = 100;
long  DISP=0,prev=0,curr=0,TARG,X=0,P=0,I=0;
int i=1,en_flag =0;
int main(void)
{
	sei();
	DDRJ |= (1<<PJ0)|(1<<PJ1);
	PORTJ |= (1<<PJ1)|(1<<PJ0);//bluetooth
	DDRE |= (1<<PE0)|(1<<PE1);
	PORTE |= (1<<PE0)|(1<<PE1);//gyro
	DDRC = 0xFF;//direction drive
	DDRH = 0xFF;//pwm pw and rack
	DDRA = 0xFF;//direction of pw and rack
	DDRB = 0xFF;//grip servo
	DDRL = 0xFF;//pwm drive
	PORTL = 0xFF;//high pwm of drive
	DDRD = 0xFF;//led
	DDRK = 0xFF;
	TCCR1B |= (1<<CS10) | (1<<WGM12) | (1<<WGM13) | (1<<CS11);
	TCCR1A |=(1<<WGM11) | (1<<COM1A1);
	PORTB = 0xFF;  //pwm grip
	TCCR4A |= (1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1)|(1<<WGM40);
	TCCR4B |= (1<<WGM42)|(1<<CS42)|(1<<CS40);//015E
	PORTH = 0xFF;//high pwm output
	DDRF  = 0x00;   //rotary encoder input
	PORTB = 0xFF;   //servo port high
    TCCR5A |= (1<<COM5B1)|(1<<COM5A1)|(1<<COM5C1)|(1<<WGM50);
    TCCR5B |= (1<<CS52)|(1<<CS50)|(1<<WGM52);
	TCCR2A |= (1<<COM2A1)|(0<<COM2A0)|(1<<WGM21)|(1<<WGM20)|(0<<COM2B1)|(0<<COM2B0);
	TCCR2B |= (1<<WGM22)|(1<<CS22)|(1<<CS20)|(1<<CS21);
	TCCR0A |= (1<<COM0A1)|(0<<COM0A0)|(1<<WGM01)|(1<<WGM00)|(0<<COM0B1)|(0<<COM0B0);
	TCCR0B |= (1<<WGM02)|(1<<CS02)|(1<<CS00)|(1<<CS01);
	ADC_initiate();
    usart_init();
   usart_init1();
	ICR1=4607;
 	OCR1A = 90;
 	_delay_ms(50);
	 PORTB = 0x00;
     prev = ADC_read(1);
    while (1) 
    {
		while_wala();
	}
}
void while_wala()
{
	main1();
	if(ip==1 && ip1 ==0)
	{
		gyro();
	}
	else
	{
		drivecross(xj1,yj1,xj2,0,0);
	}
	if(ip1 ==1)
	{
		PORTC = 0x00;
	}
	if(rackmode==1)
	{
		/*drivewheel_r(yj2,-255,255);*/
		drivewheel_grip(yj2,-255,255);
		drivewheel_rextra(xj2,-255,255);
		dispensing = 0;
		// 	PORTA &= (~(1<<PA4));
		// 	PORTA &= (~(1<<PA5));
	}
	else if(rackmode ==0)
	{
			PORTA &= (~(1<<PA1));
			PORTA &= (~(1<<PA0));
			PORTA &= (~(1<<PA2));
			PORTA &= (~(1<<PA3));
			PORTB &= (~(1<<PB0));
			PORTB &= (~(1<<PB1));
			dispensing = 1;
	}
	PORTD = enmode_curr;
}
void main1()
{
	xj1=map_value(RX[0],0,RX_range,(-pwm_range),pwm_range);
	yj1=map_value(RX[1],0,RX_range,(-pwm_range),pwm_range);
	xj2=map_value(RX[2],0,RX_range,(-rot_range),(rot_range));
	yj2=map_value(RX[3],0,RX_range,(-pwm_range),pwm_range);
	if (butt[0]==1)
    {
		ip^=1;
	    butt[0]=0;
    }
    else if (butt[1]==1)
    {
		ip1 ^=1;
	    butt[1]=0;
    }
    else if (butt[2]==1)
    {
		enmode_curr++;
		if(enmode_curr==7)
		{
		enmode_curr = 1;
		}
	    butt[2]=0;
    }
   else if (butt[3]==1)
    {
	    gyromode++;
		if(gyromode%2 == 0)
		{
			gyro_rot = 2;
		}
		else if(gyromode%2 == 1)
		{
			gyro_rot = 0;
		}
	    butt[3]=0;
    }
   else if (butt[4]==1)
    {
	    gyromode--;
		if(gyromode%2 == 0)
		{
			gyro_rot = 1;
		}
		else if(gyromode%2 == 1)
		{
			gyro_rot = 0;
		}
	    butt[4]=0;
    }
    else if (butt[5]==1)
    {
		enmode_curr--;
		if(enmode_curr==0)
		{
		enmode_curr =6;
		}
	    butt[5]=0;
    }
    else if (butt[6]==1)
    {
	    butt[6]=0;
    }
    else if (butt[7]==1)
    {
	    butt[7]=0;
    }
    else if (butt[8]==1)//l2
    {
      butt[8] = 0;
    }
    else if (butt[9]==1)//r2
    {
		 butt[9]=0;
    }
   else if (butt[10]==1)
    {
      butt[10]=0;
    }
    else if (butt[11]==1)
    {
	    butt[11]=0;
    }
    else if (butt[12]==1)
    {
      servorun();
	  gripmode ^=1;
	    servomode ++;
	    if(servomode==4)
	    {
		    servomode = 0;
	    }
       butt[12] = 0;
    }
    else if (butt[13]==1)//circle
    {
      enrun();
    }
    else if (butt[14]==1)//square
    {
      rackmode ^= 1;
		butt[14]=0;
    }
    else if (butt[15]==1)
    {
		butt[15] =0;
    }
}
void enrun()
{
	if (enmode_curr < enmode_prev)
	{
		if (en_flag == 0)
		{
			prev = ADC_read(1);
			en_flag = 1;
		}
			TARG = ((-enmode_curr+enmode_prev)*393);
			if(enmode_prev == 6 && enmode_curr <= 5)
			{
				TARG = ((-enmode_curr+enmode_prev)*393-75);
			}
		curr = ADC_read(1);
		DISP+= (curr- prev);
		if((curr - prev) > 500)
		{
			DISP -= 1024;
		}
		else if((curr - prev) < -500)
		{
			DISP += 1024;
        }
        X = TARG + DISP;
	}
	else if (enmode_curr > enmode_prev)
	{
		if(en_flag ==0)
		{
			prev = ADC_read(1);
			en_flag = 1;
		}
			TARG = ((enmode_curr-enmode_prev)*393);
			if(enmode_curr == 6&& enmode_prev <=5)
			{
			TARG = ((enmode_curr-enmode_prev)*393-75);		
			}
		curr = ADC_read(1);
		DISP+= (curr- prev);
		if((curr - prev) > 500)
		{
		DISP -= 1024;
		}
		else if((curr - prev) < -500)
		{
		DISP += 1024;
		}
		X = TARG - DISP;
	}
	prev = curr;
	en_pid();
}
void servorun()
{
	if(servomode == 0)
	{
		for(int is=90;is<=156.99;)
		{
			OCR1A = is;
			_delay_ms(20);
			is=is+22.33;
		}
		PORTB = 0;
		_delay_ms(20);
	}
	else if(servomode == 1)
	{
		for(int is=156.99;is<=401.94;)
		{
			OCR1A = is;
			_delay_ms(20);
			is=is+22.33;
		}
		PORTB = 0;
		_delay_ms(20);
	}
	else if(servomode==2)
	{
		for(int is=410.94;is>=223.3;)
		{
			OCR1A = is;
			_delay_ms(20);
			is=is-22.33;
		}
		PORTB = 0;
		_delay_ms(20);
	}
	else if(servomode==3)
	{
		for(int is=223.3;is>=90;)
		{
			OCR1A = is;
			_delay_ms(20);
			is=is-22.33;
		}
		PORTB = 0;
		_delay_ms(20);
		OCR1A = 0;
	}
}
void receive()
{
		if ((RX_raw>200) && (RX_raw<255))					//201 to 216 for addresses of analog values, 231 to 246 for buttons;
		{
			RX_ad1=RX_raw;
			if ((RX_raw>230) && (RX_raw<247))
			{
				uint8_t r_temp0=(RX_raw-231);
				butt[r_temp0]=1;
			}
		}
		else if ((RX_raw>=0) && (RX_raw<201))
		{
			uint8_t r_temp1=(RX_ad1-201);
			if (r_temp1<16)
			{
				RX[r_temp1]=RX_raw;
			}
		}
}
ISR(USART3_RX_vect)
{
	RX_raw=UDR3;
	receive();
}
void usart_init()
{
	UBRR3H=BAUDRATE>>8;
	UBRR3L=BAUDRATE;
    UCSR3B=0b10011000;//enable RXEN TXEN
	UCSR3C=0b00000110;// UCSZ1 UCSZ0
}
long map_value(long in_value, long in_min, long in_max, long out_min, long out_max)
{	
	return (in_value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
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
void drivecross(int x_vect,int y_vect,int m_vect,int errg_vect,int errg_side)
{
	if(dispensing ==1)
	{
		m_vect = 0;
	}
	x=-(-m_vect/1.1+x_vect + errg_side);			//horizontal wheel
	z=-(m_vect+(y_vect/1.168)+(x_vect/2.5) + (errg_vect/1.168) + (errg_side/2.5));					//right wheel
	y=-(-m_vect + y_vect-(x_vect/2.25)-(errg_vect) - (errg_side/2.25));			//left wheel
	drivewheel_1(x,-200,200);
	drivewheel_2(y,-200,200);
	drivewheel_3(z,-200,200);
}
void ADC_initiate()
{
	ADMUX = (0<<REFS1)|(1<<REFS0)|(0<<ADLAR);  // AVcc //  right adjusted
	ADCSRA = (1<<ADEN)|(0<<ADATE)|(0<<ADIE)|(1<<ADPS2)|(1<<ADPS1)|(1<<ADPS0); // bit4 ADC EOC flag // prescalar- 111 - 128 division factor
	ADCSRB = 0x00;
}
void drivewheel_1(long sp_vect, long l_lim, long h_lim)//black uper red niche
{     
	/*sp_vect=map_value(sp_vect,l_lim,h_lim,-255,255);*/
	sp_vect=limit_var(sp_vect,-255,255);
	if (sp_vect<(-15))
	{
		PORTC&=(~(1<<PC0));
		PORTC|=(1<<PC1);
		sp_vect=(-sp_vect);
	}
	else if (sp_vect>15)
	{
		PORTC&=(~(1<<PC1));
		PORTC|=(1<<PC0);
	}
	else
	{
		PORTC&=(~(1<<PC0));
		PORTC&=(~(1<<PC1));
		sp_vect=0;
	}
	OCR5C=sp_vect;
}
void drivewheel_2(long sp_vect, long l_lim, long h_lim)//red upar black niche
{
	/*sp_vect=map_value(sp_vect,l_lim,h_lim,-225,225);*/
	sp_vect=limit_var(sp_vect,-255,255);																																																																																												sp_vect=limit_var(sp_vect,-255,255);
	if (sp_vect<(-15))
	{
		PORTC&=(~(1<<PC3));
		PORTC|=(1<<PC2);
		sp_vect=(-sp_vect);
	}
	else if (sp_vect>15)
	{
		PORTC&=(~(1<<PC2));
		PORTC|=(1<<PC3);
	}
	else
	{
		PORTC&=(~(1<<PC2));
		PORTC&=(~(1<<PC3));
		sp_vect=0;
	}
	OCR5A=sp_vect;
}
void drivewheel_3(long sp_vect, long l_lim, long h_lim)//red upar black niche
{
	/*sp_vect=map_value(sp_vect,l_lim,h_lim,-245,245);*/
	sp_vect=limit_var(sp_vect,-255,255);
	
	if (sp_vect<(-15))
	{
		PORTC&=(~(1<<PC4));
		PORTC|=(1<<PC5);
		sp_vect=(-sp_vect);
	}
	else if (sp_vect>15)
	{
		PORTC&=(~(1<<PC5));
		PORTC|=(1<<PC4);
	}
	else
	{
		PORTC&=(~(1<<PC4));
		PORTC&=(~(1<<PC5));
		sp_vect=0;
	}
	OCR5B=sp_vect;
}
uint16_t ADC_read(uint8_t ch)
{
	ADMUX = ADMUX & 0b11100000;    //Clearing all the mux
	ADCSRB = ADCSRB & 0b11110111;  //------"-"-----------
	ch = ch & 0b00001111;
	if ( ch <= 7 )
	{
		ch = ch & 0b00000111;
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
void drivewheel_en(long sp_vect, long l_lim, long h_lim)
{
	   sp_vect=map_value(sp_vect,l_lim,h_lim,25,70);
	    sp_vect=limit_var(sp_vect,25,70);
	    if(X<5)
		{
			PORTA&=(~(1<<PA0));
			PORTA&=(~(1<<PA1));
			sp_vect = 0;
			enmode_prev = enmode_curr;
			en_flag = 0;
			butt[13] = 0;
			X = 0;
			DISP = 0;
		}
		if(enmode_curr > enmode_prev)//aisa <  1 se start karne ke liye
		{
			PORTA&=(~(1<<PA0));
			PORTA|=(1<<PA1);
		}
		else if(enmode_prev > enmode_curr)
		{
			PORTA&=(~(1<<PA1));
			PORTA|=(1<<PA0);
		}
	    OCR4C=sp_vect;
}
void en_pid()
{
   P=X*kp;	
   P = limit_var(P,0,255);
   drivewheel_en(P,0,255);
}
void usart_init1()
{
	UBRR0H=BAUDRATE>>8;
	UBRR0L=BAUDRATE;
	UCSR0B=0b10011000;//enable RXEN TXEN
	UCSR0C=0b00000110;// UCSZ1 UCSZ0
}
ISR(USART0_RX_vect)
{
	g=UDR0;
}
void drivewheel_r(long sp_vect, long l_lim, long h_lim)//red upar black niche
{
	sp_vect=map_value(sp_vect,l_lim,h_lim,-120,120);
	sp_vect=limit_var(sp_vect,-120,120);
	if (sp_vect<(-5))
	{
		PORTA&=(~(1<<PA2));
		PORTA|=(1<<PA3);
		sp_vect=(-sp_vect);
	}
	else if (sp_vect>5)
	{
		PORTA&=(~(1<<PA3));
		PORTA|=(1<<PA2);
	}
	else
	{
	   PORTA&=(~(1<<PA3));
	   PORTA&=(~(1<<PA2));
	}
	OCR4A=sp_vect;
}
void drivewheel_rextra(long sp_vect, long l_lim, long h_lim)//red upar black niche
{
	sp_vect=map_value(sp_vect,l_lim,h_lim,-120,120);
	sp_vect=limit_var(sp_vect,-120,120);
	if (sp_vect<(-5))
	{
		PORTA&=(~(1<<PA0));
		PORTA|=(1<<PA1);
		sp_vect=(-sp_vect);
	}
	else if (sp_vect>5)
	{
		PORTA&=(~(1<<PA1));
		PORTA|=(1<<PA0);
	}
	else
	{
			PORTA&=(~(1<<PA1));
			PORTA&=(~(1<<PA0));
	}
	OCR4C=sp_vect;
}
void gyro()
{
	  if(gyro_rot == 1)
	  {
		  jg = 50;
	  }
	  else if(gyro_rot == 2)
	  {
		  jg = 150;
	  }
	  else
	  {
		  jg = 100;
	  }
	  errg=jg-g;
	  PORTK = g;
	  errg = map_value(errg,-100,100,-255,255);
	  pwmg = errg;
	  pwmg = pwmg*kpg_rot;
	  pwmg = map_value(pwmg,-255,255,-65,65);
	  if(pwmg >= -18 && pwmg <=-1)
	  {
		  pwmg = -18;
	  }
	  else if(pwmg<=18 && pwmg >=1)
	  {
		  pwmg = 18;
	  }
	  errg = errg*kpg;
  if(yj1 == 0 && xj1 ==0 && g!=jg)
  {
	  drivecross(0,0,pwmg,0,0);
  }
  else if(yj1 == 0 && xj1 ==0 && g==jg)
  {
	  drivecross(0,0,0,0,0);
  }
  else if(xj1 != 0 && g!=jg)
  {
	  drivecross(xj1,yj1,xj2,errg,errg);
  }
  else
  {
	  drivecross(xj1,yj1,xj2,errg,0);
  }
}
void drivewheel_grip(long sp_vect, long l_lim, long h_lim)
{
	sp_vect=map_value(sp_vect,l_lim,h_lim,-120,120);
	sp_vect=limit_var(sp_vect,-120,120);
	if (sp_vect>5)
	{
		PORTB&=(~(1<<PB0));
		PORTB|=(1<<PB1);
	}
	else if (sp_vect<(-5))
	{
		PORTB&=(~(1<<PB1));
		PORTB|=(1<<PB0);
	}
	else
	{
	PORTB&=(~(1<<PB1));
	PORTB&=(~(1<<PB0));
	}
}
