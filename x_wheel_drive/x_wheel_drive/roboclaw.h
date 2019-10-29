#include <stdarg.h>
#include <stdbool.h>
#include <stddef.h>
#define rc_baudrate (((F_CPU / (rc_baud * 16UL))) - 1)
#define SetWORDval(arg) (uint8_t)(((uint16_t)arg)>>8),(uint8_t)arg
#define MAXRETRY 2
#define GETCURRENTS 49
#define rc_baud 115200
uint16_t crc = 0;
volatile uint16_t recvd_data = 0;			//changing to 8 bit
uint8_t *status1 = 0, *status2 = 0;
bool *valid1 = NULL, *valid2 = NULL;
int16_t current1,current2;
void drive_m1(uint8_t address, int sp_vect);
void drive_m2(uint8_t address, int sp_vect);



void ResetEncoders(uint8_t address);
int32_t read_enc1(uint8_t address);
uint32_t readenc1_raw(uint8_t address);
int32_t read_enc2(uint8_t address);
uint32_t readenc2_raw(uint8_t address);
void SetConfig(uint8_t address, uint16_t config);
void readcurrents(uint8_t address);

void write_n(uint8_t cnt, ... );
uint32_t Read4_1(uint8_t address, uint8_t cmd, uint8_t *status, bool *valid);
uint32_t Read4(uint8_t address, uint8_t cmd, bool *valid);

void crc_clear();
void crc_update (uint8_t data);
uint16_t crc_get();

void rc_serialstart_3();
void transmit(uint8_t data);
void usart_flush();
long limit_var(long in_var, long l_limit, long h_limit);
uint32_t read_time();
void set_baudrate(int baud)
{
	SetConfig()
}
void SetConfig(uint8_t address, uint16_t config)
{
	write_n(4,address,98,SetWORDval(config));
}
void rc_serialstart_3()
{
	UBRR3H = rc_baudrate>>8;
	UBRR3L = rc_baudrate;
	UCSR3B=0b00011000;//enable RXEN TXEN
	UCSR3C=0b00000110;// UCSZ1 UCSZ0
}
void transmit(unsigned char data)
{
	while(!(UCSR3A & (1<<UDRE3)));
	UDR3 = data;
}
void usart_flush()
{
	unsigned char dummy;
	while ( UCSR3A & (1<<RXC3) )
	 dummy = UDR3;
}
uint32_t read_time()
{
	while ( !((UCSR3A) & (1<<RXC3)))
	{}
	return UDR3;
}
void drive_m1(uint8_t address, int sp_vect)
{
	sp_vect = limit_var(sp_vect,-127,127);
	if(sp_vect == 0 || sp_vect > 0)
	{
		write_n(3,address,0,sp_vect);
	}
	else if(sp_vect < 0)
	{
		write_n(3,address,1,(-sp_vect));
	}
}
void drive_m2(uint8_t address, int sp_vect)
{
	sp_vect = limit_var(sp_vect,-127,127);
	if(sp_vect == 0 || sp_vect > 0)
	{
		write_n(3,address,4,sp_vect);
	}
	else if(sp_vect < 0)
	{
		write_n(3,address,5,(-sp_vect));
	}
}
void reset_enc(uint8_t address)
{
	write_n(2,address,20);
}
int32_t read_enc1(uint8_t address)
{
	int32_t enc1_val = readenc1_raw(address);
	if(valid1)
	{
		if(!(status1 && 0x02))
		{
			return enc1_val;
		}
		else
		{
			return (-enc1_val);
		}
	}	
}
void readcurrents(uint8_t address)
{
	bool valid;
	uint32_t value = Read4(address,GETCURRENTS,&valid);
	if(valid)
	{
		current1 = value>>16;
		current2 = value&0xFFFF;
	}
}
uint32_t readenc1_raw(uint8_t address)
{
	return Read4_1(address,16,&status1,&valid1);
}
int32_t read_enc2(uint8_t address)
{
	int32_t enc2_val = readenc2_raw(address);
	if(valid1)
	{
		if(!(status2 && 0x02))
		{
			return enc2_val;
		}
		else
		{
			return (-enc2_val);
		}
	}
}
uint32_t readenc2_raw(uint8_t address)
{
	return Read4_1(address,17,status2,valid2);
}
uint32_t Read4(uint8_t address, uint8_t cmd, bool *valid)
{
	if(valid)
	*valid = false;
	uint32_t value=0;
	uint8_t trys=MAXRETRY;
	int16_t data;
	do{
		usart_flush();
		crc_clear();
		transmit(address);
		crc_update(address);
		transmit(cmd);
		crc_update(cmd);
		data = read_time(timeout);
		crc_update(data);
		value=(uint32_t)data<<24;
		if(data!=-1)
		{
			data = read_time(timeout);
			crc_update(data);
			value|=(uint32_t)data<<16;
		}
		if(data!=-1)
		{
			data = read_time(timeout);
			crc_update(data);
			value|=(uint32_t)data<<8;
		}
		if(data!=-1)
		{
			data = read_time(timeout);
			crc_update(data);
			value|=(uint32_t)data;
		}
		if(data!=-1)
		{
			uint16_t ccrc;
			data = read_time(timeout);
			if(data!=-1)
			{
				ccrc = data << 8;
				data = read_time(timeout);
				if(data!=-1)
				{
					ccrc |= data;
					if(crc_get()==ccrc)
					{
						*valid = true;
						return value;
					}
				}
			}
		}
	}
	while(trys--);
	return false;
}
void write_n(uint8_t cnt, ... )
{
	uint8_t trys=MAXRETRY;
	do{
		crc_clear();
		va_list marker;
		va_start( marker, cnt );     /* Initialize variable arguments. */
		for(uint8_t index=0;index<cnt;index++)
		{
			uint8_t data = va_arg(marker, int);
			crc_update(data);
			transmit(data);
		}
		va_end( marker );              /* Reset variable arguments.      */
		uint16_t crc = crc_get();
		transmit(crc>>8);
		transmit(crc);
	}
	while(trys--);
}
uint32_t Read4_1(uint8_t address, uint8_t cmd, uint8_t *status, bool *valid)
{
	if(valid)
	*valid = false;
	uint32_t value = 0;
	uint8_t trys=MAXRETRY;
	int16_t data;
	do
	{
		usart_flush();
		crc_clear();
		transmit(address);
		crc_update(address);
		transmit(cmd);
		crc_update(cmd);
		while ( !((UCSR3A) & (1<<RXC3)));
		data = UDR3;
		crc_update(data);
		value=(uint32_t)data<<24;
		while ( !((UCSR3A) & (1<<RXC3)));
		data = UDR3;
		crc_update(data);		 
		value|=(uint32_t)data<<16;
		while ( !((UCSR3A) & (1<<RXC3)));
		data = UDR3;
		crc_update(data);		 		
		value|=(uint32_t)data<<8;	
		while ( !((UCSR3A) & (1<<RXC3)));
		data = UDR3;
		crc_update(data);
		value|=(uint32_t)data;	
		while ( !((UCSR3A) & (1<<RXC3)));
		data = UDR3;
		crc_update(data);
		if(status)
		*status = data;
		uint16_t ccrc;		
		while ( !((UCSR3A) & (1<<RXC3)));
		data = UDR3;		
		ccrc = data << 8;
		while ( !((UCSR3A) & (1<<RXC3)));
		data = UDR3;
		ccrc |= data;				
		if(crc_get() == ccrc)
		{
			*valid = true;
			return value;
		}
	}
	while(trys--);
	return false;
}
void crc_clear()
{
	crc = 0;
}
void crc_update (uint8_t data)
{
	int i;
	crc = crc ^ ((uint16_t)data << 8);
	for (i=0; i<8; i++)
	{
		if (crc & 0x8000)
		crc = (crc << 1) ^ 0x1021;
		else
		crc <<= 1;
	}
}
uint16_t crc_get()
{
	return crc;
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