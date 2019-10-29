/*
 * fence following.c
 *
 * Created: 22-07-2018 04:54:54 PM
 * Author : soofi
 */ 
#define F_CPU 14745600UL
#define BAUDRATE ((F_CPU/(BAUD*16UL)-1))
#define BAUD 9600
#define kp 1.5
#define ki 0.0005


#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
#include "lcd.h"


int distancef = 0,distanceb = 0,target = 25;
int x,y,z;
int butt[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int RX_raw=-1,RX_ad1=-1,RX_range=200,pwm_range = 200,rot_range = 100;
int xj1=0,yj1=0,xj2=0,yj2=0,x_vect=0,y_vect =0;
uint8_t RX[16]={100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100};
uint8_t on = 0;
int errorf = 0, errorb = 0, P1 = 0,I1 = 0,P2 = 0,I2 = 0,error1 = 0,error2 = 0,error3 = 0;

uint16_t ADC_read(uint8_t ch);
void ADC_initiate();
void init_ports1();
long limit_var(long in_var, long l_limit, long h_limit);
void drivecross(int x_vect,int y_vect,int m_vect ,int errpid_1,int errpid_2,int errpid_3);
void drivewheel_1(long sp_vect, long l_lim, long h_lim);
void drivewheel_2(long sp_vect, long l_lim, long h_lim);
void drivewheel_3(long sp_vect, long l_lim, long h_lim);
void usart_init();
void ps2_values();
void distance_values();
void error_distance();
void receive();


int main(void)
{
    
	init_ports1();
	ADC_initiate();
	init_ports();
	lcd_init();
	usart_init();
	TCCR4A |= (1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1)|(1<<WGM40);
	TCCR4B |= (1<<WGM42)|(1<<CS41)|(1<<CS40);
	sei();
	
	
    while (1) 
    {
		distance_values();
		error_distance();
		ps2_values();
// 		lcd_print(1,1,distancef,4);
// 		lcd_print(1,6,distanceb,4);
// 		lcd_print(2,1,(distancef - target),5);
// 		lcd_print(2,6,(distanceb - target),5);
		if(on == 1)
		{
			if((distancef > target && distanceb < target) || (distancef < target && distanceb > target))  //1
			{
				error1 = -errorf + errorb;
				error2 = errorf*2;
				error3 = -errorb*2;
			}
			else if((distancef < target && distanceb < target) || (distancef > target && distanceb > target))  //1
			{
				error1 = (errorf + errorb);
				error2 = -errorf*2;
				error3 = -errorb*2;
			}
            drivecross(200,0,0,error1,error2,error3);
		}
		else
		{
			drivecross(0,0,0,0,0,0);
			PORTA = 0x00;
		}
    }
}


void error_distance()
{
	P1 = (distancef - target)*kp;
	I1 += (distancef - target)*ki;
	P2 = (distanceb - target)*kp;
	I2 += (distanceb - target)*ki;
	errorf = P1 + I1;
	errorb = P2 + I2;
}


void drivecross(int x_vect,int y_vect,int m_vect ,int errpid_1,int errpid_2,int errpid_3)
{
	x = (m_vect/2) + (x_vect) + (errpid_1*0.45);				//horizontal(1)  -+-
	y = (m_vect/2) - (y_vect) - (x_vect*0.8) - (errpid_2*0.8);		//left(2) - +++(y,x,v,h)
	z = (m_vect/2) + (y_vect) - (x_vect*0.8) - (errpid_3*0.8);		//right(3) + +++
	drivewheel_1(z,-255,255);
	drivewheel_2(y,-255,255);
	drivewheel_3(x,-255,255);
}


void drivewheel_1(long sp_vect, long l_lim, long h_lim)//black uper red niche
{
	sp_vect=limit_var(sp_vect,l_lim,h_lim);
	if (sp_vect<(-1))
	{
		PORTA&=(~(1<<PA0));
		PORTA|=(1<<PA1);
		sp_vect=(-sp_vect);
	}
	else if (sp_vect>1)
	{
		PORTA&=(~(1<<PA1));
		PORTA|=(1<<PA0);
	}
	else
	{
		PORTA &= ~(1<<PA0);
		PORTA &= ~(1<<PA1);
		sp_vect=0;
	}
	OCR4A = sp_vect;
	
}


void drivewheel_2(long sp_vect, long l_lim, long h_lim)//red upar black niche
{
	sp_vect=limit_var(sp_vect,l_lim,h_lim);
    sp_vect=limit_var(sp_vect,-255,255);
	if (sp_vect<(-1))
	{
		PORTA &= (~(1<<PA2));
		PORTA |= (1<<PA3);
		sp_vect = (-sp_vect);
	}
	else if (sp_vect>1)
	{
		PORTA&=(~(1<<PA3));
		PORTA|=(1<<PA2);
	}
	else
	{
		PORTA &= ~(1<<PA2);
		PORTA &= ~(1<<PA3);
		sp_vect=0;
	}
	OCR4B = sp_vect;
}


void drivewheel_3(long sp_vect, long l_lim, long h_lim)//red upar black niche
{
	sp_vect=limit_var(sp_vect,l_lim,h_lim);
	if (sp_vect<(-1))
	{
		PORTA&=(~(1<<PA4));
		PORTA|=(1<<PA5);
		sp_vect = -sp_vect;
	}
	else if (sp_vect>1)
	{
		PORTA&=(~(1<<PA5));
		PORTA|=(1<<PA4);
	}
	else
	{
		PORTA &= ~(1<<PA4);
		PORTA &= ~(1<<PA5);
		sp_vect=0;
	}
	OCR4C=sp_vect;
}


void distance_values()
{
	uint16_t d = ADC_read(0);
    distancef = (6762/(d-9))-4;
	d = ADC_read(1);
	distanceb = (6762/(d-9))-4;	
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


void ADC_initiate()
{
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


void init_ports1()
{
	DDRF = 0x00;
    DDRH = 0xFF;
	PORTH = 0xFF;
	DDRA = 0xFF;
}


void receive()
{
	if ((RX_raw>200) && (RX_raw<255))
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
	DDRJ = 0xFF;
	PORTJ = 0xFF;
}

void ps2_values()
{
	if (butt[0]==1)//l1
	{
		on ^= 1;
		butt[0]=0;
	}
	else if (butt[1]==1)//r1
	{
		on ^= 1;
		butt[1]=0;
	}
	else if (butt[2]==1)//l2
	{
		on ^= 1;
		butt[2]=0;
	}
	else if (butt[3]==1)//r2
	{
		on ^= 1;
		butt[3]=0;
	}
	else if (butt[4]==1)//l3
	{
		on ^= 1;
		butt[4]=0;
	}
	else if (butt[5]==1)//r3
	{
		on ^= 1;
		butt[5]=0;
	}
	else if (butt[6]==1)//triangle
	{
		on ^= 1;
		butt[6]=0;
	}
	else if (butt[7]==1)//square
	{
		on ^= 1;
		butt[7]=0;
	}
	else if (butt[8]==1)//cross
	{
		on ^= 1;
		butt[8] = 0;
	}
	else if (butt[9]==1)//circle
	{
		on ^= 1;
		butt[9]=0;
	}
	else if (butt[10]==1)//up
	{
		on ^= 1;
		butt[10]=0;
	}
	else if (butt[11]==1)//left
	{
		on ^= 1;
		butt[11]=0;
	}
	else if (butt[12]==1)//down
	{
		on ^= 1;
		butt[12] = 0;
	}
	else if (butt[13]==1)//right
	{
		on ^= 1;
		butt[13]=0;
	}
	else if (butt[14]==1)//start
	{
		on ^= 1;
		butt[14]=0;
	}
	else if (butt[15]==1)//select
	{
		on ^= 1;
		butt[15] = 0;
	}
}
