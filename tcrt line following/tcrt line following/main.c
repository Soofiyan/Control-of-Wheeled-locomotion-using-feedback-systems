/*
 * tcrt line following.c
 *
 * Created: 16-06-2018 12:30:32 PM
 * Author : soofi
 */ 
#define F_CPU 14745600UL
#include <avr/io.h>
#include <avr/interrupt.h>
#include <util/delay.h>


void PID_linefollowing();
void calculate();
void drivecross(int x_vect,int y_vect,int m_vect ,int errpid);
void drivewheel_1(long sp_vect, long l_lim, long h_lim);
void drivewheel_2(long sp_vect, long l_lim, long h_lim);
void drivewheel_3(long sp_vect, long l_lim, long h_lim);
void timer4();
long limit_var(long in_var, long l_limit, long h_limit);


int sen_array[8]  = {0,0,0,0,0,0,0,0};
int i = 0,sensor_count = 0,sensor_whiteline = 0,target = 4.5;
int temp_variable = 0x01;
int error = 0,kp = 8,ki = 0.005,I = 0,P = 0;
int x = 0,y = 0,z = 0;
int left_pwm = 0,right_pwm = 0,total_error = 0;


int main(void)
{
	DDRK = 0x00;                  //TCRT sensors
	DDRB = 0xFF;				  //led
	DDRC = 0xFF;
	DDRH = 0xFF;
	PORTH = 0xFF;
	
	timer4();
    while (1) 
    {
	  calculate();
	  PID_linefollowing();
	  drivecross(0,100,0,total_error);
    }
}


void PID_linefollowing()
{
  	error = (sensor_whiteline/sensor_count) - target;
	P = error*kp;
	I += error*ki;
	total_error = P + I ;
}


void calculate()
{
    sensor_whiteline = 0;
    sensor_count = 0;
	for(i =0;i<=7 ;i++)
	{
		if(i == 0)
		{
		  temp_variable = 0x01;
		}
		sen_array[i] = PIND & temp_variable;
		if(sen_array[i] != 0x00)
		{
		sensor_whiteline += (i+1);
		sensor_count ++;
		}
	    temp_variable = temp_variable << 1;
	}
}


void drivecross(int x_vect,int y_vect,int m_vect ,int errpid)
{
	x = (m_vect/2) + (x_vect) + errpid;				//horizontal(1)
	y = (m_vect/2) - (y_vect) + (x_vect) + errpid;		//left(2)
	z = -(m_vect/2) + (y_vect) + (x_vect) + errpid;		//right(3)
	drivewheel_1(x,-255,255);
	drivewheel_2(y,-255,255);
	drivewheel_3(z,-255,255);
}


void drivewheel_1(long sp_vect, long l_lim, long h_lim)//black uper red niche
{
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
		PORTC &= ~(1<<PC0);
		PORTC &= ~(1<<PC1);
		sp_vect=0;
	}
	OCR4A = sp_vect;
}


void drivewheel_2(long sp_vect, long l_lim, long h_lim)//red upar black niche
{
	sp_vect=limit_var(sp_vect,-255,255);
	if (sp_vect<(-15))
	{
		PORTC &= (~(1<<PC2));
		PORTC |= (1<<PC3);
		sp_vect = (-sp_vect);
	}
	else if (sp_vect>15)
	{
		PORTC&=(~(1<<PC3));
		PORTC|=(1<<PC2);
	}
	else
	{
		PORTC &= ~(1<<PC2);
		PORTC &= ~(1<<PC3);
		sp_vect=0;
	}
	OCR4B=sp_vect;
}


void drivewheel_3(long sp_vect, long l_lim, long h_lim)//red upar black niche
{
	sp_vect=limit_var(sp_vect,-255,255);
	if (sp_vect<(-15))
	{
		PORTC&=(~(1<<PC4));
		PORTC|=(1<<PC5);
		sp_vect = -sp_vect;
	}
	else if (sp_vect>15)
	{
		PORTC&=(~(1<<PC5));
		PORTC|=(1<<PC4);
	}
	else
	{
		PORTC &= ~(1<<PC4);
		PORTC &= ~(1<<PC5);
		sp_vect=0;
	}
	OCR4C=sp_vect;
}


void timer4()
{
	TCCR4A |= (1<<COM4A1)|(1<<COM4B1)|(1<<COM4C1)|(1<<WGM40);
	TCCR4B |= (1<<WGM42)|(1<<CS42)|(1<<CS40);
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
