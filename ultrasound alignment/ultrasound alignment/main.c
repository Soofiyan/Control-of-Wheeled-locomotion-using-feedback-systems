#define F_CPU 14745600UL
#define BAUD 9600
#define kp	1
#define kpg 3.335
#define kig 0.0015
#define kp_us 15
#define BAUDRATE ((F_CPU/(BAUD*16UL)-1))
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <math.h>
void usart_init();
long map_value(long in_value, long in_min, long in_max, long out_min, long out_max);
void drivecross(int x_vect, int y_vect , int m_vect , int errg_vect , int errg_side);
void gyro();
void receive();
void gripka();
void ultra_init();
void main1();
void gyro1();
void rack();
void ultra();
void ultrasound_trig();
void timer_init1();
void timer_init3();
void while_wala();
void drivewheel_1(long sp_vect, long l_lim, long h_lim);
void drivewheel_2(long sp_vect, long l_lim, long h_lim);
void drivewheel_3(long sp_vect, long l_lim, long h_lim);
long limit_var(long in_var, long l_limit, long h_limit);
int butt[16] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
int x=0,y=0,z=0;
int RX_raw=-1,RX_ad=-1,m=0,RX_ad1=-1,RX_range=200,pwm_range = 200,rot_range = 100;
int xj1=0,yj1=0,xj2=0,yj2=0,x_vect=0,y_vect =0,us = 0,targ_ush = 0,targ_usv = 0,us_flag = 0,us_error = 0;
uint8_t RX[16]={100,100,100,100,100,100,100,100,100,100,100,100,100,100,100,100};
int errg=0,errg1 = 0,g=0,countd = 0,curr_timer = 0,prev_timer = 0;
int ip=0,ip1=0,ip2 = 0,gyromode = 1000,gyro_rot = 0,Ig_rot =0,rackmode=0,rack_niche =0,rack_upar = 0,jg = 100,grip_piche =0,g_flag=0,grip_aage =0;
int en_flag =0,k=0,kg=0,kg1=0,kg2=0,kg3=0,kg4=0,kg5 =0,kg6=0;
int countg =0,countg1=0,g1=0,errg1_p = 0,errg1_i = 0,slow =0,pi=0,pi3 = 0,pi4 = 0,pi2 = 0, pi1 = 50;
uint32_t r,result,i,i1,result1,r1;
int d,d1;
void naya();
int flag = 0,flag1 = 0,e = 0,c=0,u = 0;
int main(void)
{
	sei();
	DDRJ |= (1<<PJ0)|(1<<PJ1);
	PORTJ |= (1<<PJ1)|(1<<PJ0);//bluetooth
	DDRE = 0x00;
	DDRC = 0xFF;//direction drive
	DDRH = 0xFF;//pwm drive
	DDRL = 0xFF;//pwm drive
	DDRA = 0x00;
	DDRK = 0xFF;
	DDRF = 0xFF;
	TCCR4A |= (1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1)|(1<<WGM40);
	TCCR4B |= (1<<WGM42)|(1<<CS42)|(1<<CS40);
		EICRB |= (1<<ISC50)|(1<<ISC51);
		EIMSK |= (1<<INT5);
	PORTH = 0xFF;//high pwm output
	usart_init();
	timer_init1();
	while(1)
{
	main1();
/*	ultra();*/
ultrasound_trig();
if(c == 1)
{
	naya();
	c = 0;
}
	PORTK = d;
	PORTF = d1;
}
}
	void main1()
	{
		xj1=map_value(RX[0],0,RX_range,(pwm_range),(-pwm_range));
		yj1=map_value(RX[1],0,RX_range,(pwm_range),(-pwm_range));
		xj2=map_value(RX[2],0,RX_range,(-rot_range),(rot_range));
		yj2=map_value(RX[3],0,RX_range,(pwm_range),(-pwm_range));
		if (butt[0]==1)
		{
			ip2 ^=1;
			us_flag = 1;
			u = 0;
			butt[0]=0;
		}
		else if (butt[1]==1)
		{
			ip1 ^=1;
			butt[1]=0;
		}
		else if (butt[2]==1)
		{
			grip_aage ^= 1;
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
			g_flag = 1;
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
			g_flag = 1;
			butt[4]=0;
		}
		else if (butt[5]==1)
		{
			grip_piche ^= 1;
			butt[5]=0;
		}
		else if (butt[6]==1)
		{
			us++;
			switch(us)
			{
				case 1:
				break;
				case 2:
				break;
				case 3 :
				break;
				case 4 :
				us = 0;
				break;
			}
			us_flag = 1;
			butt[6]=0;
		}
		else if (butt[7]==1)
		{
			us = 0;
			us_flag = 0;
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
			ip ^= 1;
			butt[10]=0;
		}
		else if (butt[11]==1)
		{
			butt[11]=0;
		}
		else if (butt[12]==1)
		{
			slow ^= 1;
			butt[12] = 0;
		}
		else if (butt[13]==1)//circle
		{
			rack_upar ^=1;
			pi2 = 0;
			butt[13]=0;
		}
		else if (butt[14]==1)//square
		{
			rack_niche^= 1;
			pi2 = 0;
			butt[14]=0;
		}
		else if (butt[15]==1)
		{
			butt[15] =0;
		}
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
	x=-(-(m_vect/2)+(x_vect*0.9/1.05) + (errg_side));			//horizontal wheel
	y=(-(m_vect/2.5) + (y_vect)-(x_vect*0.9/2)+(errg_vect) + (errg_side/1.78));
	z=((m_vect/2.5)+(y_vect/1.19)+(x_vect/2.32) - (errg_vect/1.19) - (errg_side/2.12));
	drivewheel_1(x,-200,200);
	drivewheel_2(y,-200,200);
	drivewheel_3(z,-200,200);
}
void drivewheel_1(long sp_vect, long l_lim, long h_lim)//black uper red niche
{
	sp_vect=limit_var(sp_vect,-255,255);
	if (sp_vect<(-45))
	{
		PORTC&=(~(1<<PC0));
		PORTC|=(1<<PC1);
		sp_vect=(-sp_vect);
	}
	else if (sp_vect>45)
	{
		PORTC&=(~(1<<PC1));
		PORTC|=(1<<PC0);
	}
	else
	{
		PORTC|=(1<<PC0);
		PORTC|=(1<<PC1);
		sp_vect=0;
	}
	OCR4C=sp_vect;
}
void drivewheel_2(long sp_vect, long l_lim, long h_lim)//red upar black niche
{
	sp_vect=limit_var(sp_vect,-255,255);
    sp_vect=limit_var(sp_vect,-255,255);
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
		PORTC|=(1<<PC2);
		PORTC|=(1<<PC3);
		sp_vect=0;
	}
	OCR4A=sp_vect;
}
void drivewheel_3(long sp_vect, long l_lim, long h_lim)//red upar black niche
{
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
		if(xj1>=20 && xj1<=200)
		{
			sp_vect = sp_vect*0.965;
		}
		if(yj1>=20 && yj1<=200)
		{
			sp_vect = sp_vect*0.985;
		}
		sp_vect = sp_vect*0.954;
	}
	else
	{
		PORTC|=(1<<PC4);
		PORTC|=(1<<PC5);
		sp_vect=0;
	}
	OCR4B=sp_vect;
}

void ultra()
{
	if(u ==0)
	{
		us_error = (20 - d)*kp_us;
		drivecross(us_error,0,0,errg1,errg1);
		if(us_error ==0)
		{
			drivecross(0,0,0,errg1,errg1);
			u = 1;
			PORTK = 0xAA;
		}
	}
	else if(u == 1)
	{
		us_error = (20-d1)*kp_us;
		drivecross(us_error,0,0,errg1,errg1);
		if(us_error == 0)
		{
		drivecross(0,0,0,errg1,errg1);
		u = 2;
		}
	}
}
void ultrasound_trig()
{
		if(e==0)
		{
			DDRE|=(1<<PE4);
			_delay_us(10);
			PORTE |=(1<<PE4);
			_delay_us(15);
			PORTE&=(~(1<<PE4));//Low
			_delay_us(20);
			DDRE&=(~(1<<PE4));
			e=1;
	}
// 	else	if(c%2 ==1)
// 	{
// 		if(e==0)
// 		{
// 			DDRE|=(1<<PE6);
// 			_delay_us(10);
// 			PORTE |=(1<<PE6);
// 			_delay_us(15);
// 			PORTE&=(~(1<<PE6));//Low
// 			_delay_us(20);
// 			DDRE&=(~(1<<PE6));
// 			e=1;
// 		}
// 	}
}
// ISR(INT5_vect)
// {
// 	flag1 ^= 1;
// 	if(flag1 ==1)
// 	{
// 		TCCR3A=0X00;
// 		TCCR3B=(1<<CS31); //Prescaler = Fcpu/8
// 		TCNT3=0x00;       //Init counter
// 	}
// 	else
// 	{
// 		result1 = TCNT3;
// 		TCCR3B=0x00;
// 		r1= (result1>>1);
// 		d1=(r1/48.738);
// 		e = 0;
// 		c++;
// 	}
// }
// ISR(INT7_vect)
// {
// 	flag ^= 1;
// 	if(flag ==1)
// 	{
// 		TCCR5A=0X00;
// 		TCCR5B=(1<<CS51); //Prescaler = Fcpu/8
// 		TCNT5=0x00;       //Init counter
// 	}
// 	else
// 	{
// 		result = TCNT5;
// 		TCCR5B=0x00;
// 		r = (result>>1);
// 		d=(r/48.738);
// 		e = 0;
// 		c++;
// 	}
// }
ISR(INT5_vect)
{
	//Setup Timer1
	TCCR3A=0x00;
	TCCR3B=(1<<CS31); //Prescaler = Fcpu/8
	TCNT3=0x00;       //Init counter
	//Now wait for the falling edge
	c = 1;
}
void naya()
{
	for(i=0;i<50000;i++)
	{
		if((PINE & (1<<PE5)) ==1 )
		{
			if(TCNT3 > 30000) break; else continue;
		}
		else
		break;
	}
	//Falling edge found
	result = TCNT3;
	//Stop Timer
	TCCR3B=0x00;
	r = (result>>1);
	//Handle Errors
	d=(r/48.738); //Convert to cm
	e = 0;
}
void timer_init1()
{
	TCCR1A |= 0x00;
	TCCR1B |= (1<<CS12)|(1<<CS10);
	TIMSK1 |= (1<<TOIE1);
}
ISR(TIMER1_OVF_vect)
{
	TCNT1 = 64671;
	 	curr_timer = RX_raw;
	 	if(curr_timer == prev_timer)
	 	{
	 		PORTC = 0x00;
	}
	prev_timer = curr_timer;
}
