//
//  main.c
//  PS2 code
//
//  Created by Soofiyan Atar on 04/12/18.
//
#define F_CPU 16000000UL
#define BAUD 115200
#define MYUBRR    F_CPU/8/BAUD-1

#include <avr/io.h>
#include <util/delay.h>

uint8_t value = 0,value1 = 0;

long map_value(long in_value, long in_min, long in_max, long out_min, long out_max);
long limit_var(long in_var, long l_limit, long h_limit);
void ADC_Init(void);
int ADC_Read(char channel);
void init_uart(unsigned short uValue);
void USART_Transmit( unsigned char data);
void keypad_init(void);
void button_pressed(void);
void check1p(void);
void check2p(void);
void check3p(void);
void combine_port(void);

int main(void)
{
    keypad_init();
    init_uart(MYUBRR);
    ADC_Init();
    
    int X_v_l,Y_v_l,X_v_r,Y_v_r;
    uint8_t xlout,xrout,yrout,ylout;
    
    while (1)
    {
        button_pressed();
        _delay_ms(10);
        X_v_l = ADC_Read(3);                         /* Read X, Y, Z axis ADC value */
        Y_v_l = ADC_Read(2);
        X_v_r = ADC_Read(1);
        Y_v_r = ADC_Read(0);
        xlout = map_value(X_v_l,0,1024,0,200);
        ylout = map_value(X_v_r,0,1024,0,200);
        xrout = map_value(Y_v_l,0,1024,0,200);
        yrout = map_value(Y_v_r,0,1024,0,200);
        USART_Transmit(201);
        USART_Transmit(xlout);
        _delay_ms(20);
        USART_Transmit(202);
        USART_Transmit(ylout);
        _delay_ms(20);
        USART_Transmit(203);
        USART_Transmit(xrout);
        _delay_ms(20);
        USART_Transmit(204);
        USART_Transmit(yrout);
        _delay_ms(20);
    }
    return 0;
}

long map_value(long in_value, long in_min, long in_max, long out_min, long out_max)
{
    return (in_value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
long limit_var(long in_var, long l_limit, long h_limit)
{
    if (in_var>h_limit)
    {in_var=h_limit;}
    else if (in_var<l_limit)
    {in_var=l_limit;}
    return in_var;
}

void ADC_Init(void)                                           /* ADC Initialization function */
{
    DDRC = 0x00;                                      /* Make ADC port as input */
    ADCSRA = 0x87;                                      /* Enable ADC, with freq/128  */
    ADMUX = 0x40;                                      /* Vref: Avcc, ADC channel: 0 */
}

int ADC_Read(char channel)                              /* ADC Read function */
{
    ADMUX = 0x40 | (channel & 0x07);                  /* set input channel to read */
    ADCSRA |= (1<<ADSC);                              /* Start ADC conversion */
    while (!(ADCSRA & (1<<ADIF)));                /* Wait until end of conversion by polling interrupt flag */
    ADCSRA |= (1<<ADIF);                                 /* Clear interrupt flag */
    _delay_ms(1);                                      /* Wait a little bit */
    return ADCW;                                      /* Return ADC word */
}

void init_uart(unsigned short uValue)
{                                                     // setting the baud rate  based on the datasheet
    UBRR0H =0x00;                                     //(unsigned char)  ( uValue>> 8);  // 0x00
    UBRR0L =0x0C;                                     //(unsigned char) uValue;  // 0x0// enabling TX & RX
    UCSR0B = (1<<RXEN0)|(1<<TXEN0);
    UCSR0A = (1<<UDRE0)|(1<<U2X0);
    UCSR0C =  (1 << UCSZ01) | (1 << UCSZ00);         // Set frame: 8data, 1 stop
}

void USART_Transmit( unsigned char data)
{                                                    /* Wait for empty transmit buffer */
    while (!(UCSR0A & (1<<UDRE0)));               /* Put data into buffer, sends the data */
    UDR0 = data;
}

void keypad_init(void)
{
    DDRD |= 0xE0;
    DDRD &= ~(0x1C);
    PORTD |= 0xFC;
}

void button_pressed(void)
{
    PORTD |= 0x7C;
    PORTD &= ~(0x80);
    combine_port();
    if((value & 0x7C) != 0x7C)
    {
        check1p();
    }
    
    PORTD |= 0xBC;
    PORTD &= ~(0x40);
    combine_port();
    if((value & 0xBC) != 0xBC)
    {
        check2p();
    }
    
    PORTD |= 0xDC;
    PORTD &= ~(0x20);
    combine_port();
    if((value & 0xDC) != 0xDC)
    {
        check3p();
    }
}

void check1p(void)
{
    if((value & 0xFD) == 0x78)
    {
        value1 = 234;
        USART_Transmit(value1);
    }
    if((value & 0xFD) == 0x74)
    {
        value1 = 232;
        USART_Transmit(value1);
    }
    if((value & 0xFD) == 0x6C)
    {
        value1 = 245;
        USART_Transmit(value1);
    }
}

void check2p(void)
{
    if((value & 0xFD) == 0xB8)
    {
        value1 = 235;
        USART_Transmit(value1);
    }
    if((value & 0xFD) == 0xB4)
    {
        value1 = 238;
        USART_Transmit(value1);
    }
    if((value & 0xFD) == 0xAC)
    {
        value1 = 243;
        USART_Transmit(value1);
    }
}

void check3p(void)
{
    if((value & 0xFD) == 0xD8)
    {
        value1 = 236;
        USART_Transmit(value1);
    }
    if((value & 0xFD) == 0xD4)
    {
        value1 = 231;
        USART_Transmit(value1);
    }
    if((value & 0xFD) == 0xCC)
    {
        value1 = 244;
        USART_Transmit(value1);
    }
}

void combine_port(void)
{
    value = 0x00;
    value |= (PIND & 0xFC);
}
