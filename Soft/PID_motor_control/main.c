/*
 * PID_motor_control.c
 *
 * Created: 20.10.2021 17:06:38
 * Author : User
 */ 

/*
Таймер 1 (16 бит) используется для захвата тахо сигнала ICP1 на ноге PB0
Таймер 2 (8 бит) используется для генерации ШИМ сигнала OC2B на ноге PD3
Аналоговый сигнал захватывается ADC0 на ноге PC0 - отключен. Установка производится энкодером
Таймер 0 (8 бит) отсчитывает период расчета ПИД
*/
#define F_CPU 16000000UL //16MHz
#define BAUD 9600
#define MYUBRR F_CPU/16/BAUD-1

#define save          (!(PINC&(1<<PC0)))  //сигналы от кнопок
#define off           (!(PINC&(1<<PC1)))
#define on            (!(PINC&(1<<PC2)))
#define enc_butt      (!(PINC&(1<<PC3)))

#include <util/delay.h>
#include <avr/io.h>
#include <avr/interrupt.h>
#include <stdio.h>
#include <avr/eeprom.h>
#include "uart328.h"
#include "LCD.h"
#include "ENCODER.h"

uint16_t periodTime=18750;
uint8_t delay_show;
uint8_t overflow_T1;
uint8_t flag_show, show_counter, flag_start, flag_save, save_ok, stop_flag, flag_calc, flag_taho;
uint16_t set_speed, real_speed;
uint8_t schet;
char string[20];
uint16_t save_speed EEMEM = 50;  //минимальная скорость. но ерр все равно не шью 


 /************************************************************************/
 /*       Масштабирование переменной Х                                   */
 /************************************************************************/
 long map(long x, long in_min, long in_max, long out_min, long out_max) 
 {
 	return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
 }
  
 /************************************************************************/
 /*       ПИД регулятор                                                  */
 /************************************************************************/
 int computePID(float input, float setpoint, float kp, float ki, float kd) 
 {
	  float err = setpoint - input;
	  static float integral = 0, prevErr = 0;
	  
	  integral = integral + (float)err * ki ; //*dt;
	  if (integral <= 0) { integral=0; }
	  if(integral >= 255) { integral=255; }
		  
	  float D = (err - prevErr);// / dt;
	  prevErr = err;
		
	  if ((err * kp + integral + D * kd)<=0)  { return 0; }		//остановка 
	  if ((err * kp + integral + D * kd)>=255) { return 255; }	//запуска
	  return err * kp + integral + D * kd;	 
  }
  
  /************************************************************************/
  /*       Вывод графиков в плоттер по последовательному соединению       */
  /************************************************************************/
 void debug_serial()
 {
	  sprintf(string,"%d", real_speed);
	  Serial_print(string);
	  Serial_print(",");
	  sprintf(string,"%d", set_speed);
	  Serial_print(string);
	  Serial_print(",");
	  sprintf(string,"%d", OCR2B);
	  Serial_println(string);
 }
   
 //---Таймер считает время периода от датчика
 ISR (TIMER1_CAPT_vect)  
 {
	static uint16_t dump;
	periodTime = ICR1-dump;
	dump=ICR1; 
	overflow_T1=0;
	stop_flag=0;
	real_speed=1100000/periodTime;	
 }
 
 //---Определение переполнения от таймера датчика - двигатель остановился
 ISR (TIMER1_OVF_vect)
 {
	 overflow_T1++;
	 if (overflow_T1==2) {stop_flag=1; overflow_T1=1;}  //если переполнение случилось 2 раза, датчик не насчитал ни одного оборота, то флаг остановки = 1
 }
 
 //---Таймер задает флаг с частотой дескритизации ПИД регулятора
 ISR(TIMER0_COMPA_vect){

	asm("cli");
  	static uint8_t count;
  	count++;
  
	count=0;
	flag_calc=1;
	

	//счетчик периода отображения информации на дисплее (200мс)
	show_counter++;  
	if (show_counter==40)
	{
		show_counter=0;
		flag_show=1;
	}

	//опрос кнопок и энкодера
	if (on) 
	{
		flag_start=1;
		TCCR2B=2;
		TCCR2A|=(1<<COM2B1)|(1<<WGM21)|(1<<WGM20); //fast PWM, OCR2, TOP=0xFF
	}
	if (off)
	{
		flag_start=0;
		TCCR2B=0; // остановка таймера
		TCCR2A=0; //fast PWM, OCR2, TOP=0xFF
		OCR2B=0;
		PORTD&=~(1<<PD3);
	}
	if (save){flag_save=1;}
		
	set_speed+=(signed char)encoder_proc();
	
	if (set_speed<=80)
	{
		set_speed=80;
	}
	if (set_speed>=600)
	{
		set_speed=600;
	}
	asm("sei");
 }



int main(void) {
	
    _delay_ms(100);

 	USART_Init(MYUBRR);
	encoder_init();
	 
	//на вход с подтягиванием
	PORTC|=(1<<PC0)|(1<<PC1)|(1<<PC2)|(1<<PC3); //кнопки

 	//Настройка ШИМ
  	DDRD|=(1<<PD3); //выход ШИМ
	//TCCR2A|=(1<<COM2B1)|(1<<WGM21)|(1<<WGM20); //fast PWM, OCR2, TOP=0xFF
  	//TCCR2B=3; // 7812 Гц
 	//OCR2B=50;
	PORTD&=~(1<<PD3); //остановка двигателя
 	
    //Настройка таймера для счета оборотов
 	PORTB|=(1<<PB0);// вход сигнала захвата ICP, с подтяжкой
  	TCCR1A = 0; // нормальный режим работы таймера 1
  	TCCR1B = (1 << ICNC1)| (1 << CS11) | (1 << CS10); // включение подавления шума входного сигнала, делитель 64, выбор отрицательного фронта входного сигнала
  	TCNT1 = 0; // сброс счетчика
  	TIMSK1 = (1 << ICIE1)|(1 << TOIE1); //разрешения прерываний таймера 1 по захвату и переполнению
	
 	//настройка периода расчета ПИД
   	TCCR0A|=(1<<WGM01); //сброс при совпадении
   	TCCR0B|=(1<<CS02)|(1<<CS00); //деление на 1024
   	OCR0A=156; //234 - 15мс, 156-период 10мс
  	TIMSK0|=(1<<OCIE0A); //прерывание по переполнен	 
		
  	LCDinit();
  	LCDclear();
  	
    set_speed = eeprom_read_word(& save_speed);

	asm("sei");
	
	while(1) 
	{ 	
		if (stop_flag)
		{
			real_speed=0;
		}
		
     	if (flag_calc && flag_start) //флаг устнавливается с частотой дескритизации ПИД регулятора таймером 0
     	{
   			flag_calc=0;
			
			OCR2B = computePID(real_speed, set_speed, 1, 0.02, 0);  //управление ШИМ    1  0.1   0
 		    debug_serial();
 		}
		 
  		if (flag_show)
  		{
 			flag_show=0;

			if (flag_save)
			{
				LCDstring("Set:  SAVE      ",0,0);

 				delay_show++;
				if (delay_show==7)
 				{
	 				delay_show=0;
	 				flag_save = 0;
 				}	
			} 
			else
			{
				//LCDclear();
				save_ok=1;
				sprintf(string, "Set:  %d.%02d m/s  ", set_speed/100, set_speed%100);
				LCDstring(string,0,0);
			} 					
 			sprintf(string, "Real: %d.%02d m/s  ", real_speed/100, real_speed%100);
 			LCDstring(string,0,1);
 		}		
		
		if (flag_save&&save_ok)  //если нажали кнопку и если это первое сохранение 
		{	
			eeprom_write_word(& save_speed, set_speed);
			save_ok=0;		
		}
	}
}



