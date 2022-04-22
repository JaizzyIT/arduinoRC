#include <directADC.h>
#include <EEPROM.h>
#include <GyverPower.h>
#include <SPI.h>
#include <directADC.h>
#include <bits_macros.h>

#include "nRF24L01.h"
#include "RF24.h"

#define MOTORS_PIN 3
#define MOTORS_PWM OCR0B

#define SERVO_1_ROLL_PWM OCR1A					//left servo
#define SERVO_2_ROLL_PWM OCR1B					//right servo
#define SERVO_3_PITCH_PWM OCR3A

#define SERVO_1_ROLL_PIN 9
#define SERVO_2_ROLL_PIN 10 
#define SERVO_3_PITCH_PIN 5

/*
values in discretes
32-160 discretes = 0-180 degrees, PWM frequency = 61 Hz, duty cycle = 0.5us - 2.5us 
96 discretets - middle position
*/
#define SERVO_1_ROLL_DEF_POS 96
#define SERVO_2_ROLL_DEF_POS 96
#define SERVO_3_PITCH_DEF_POS 96

#define SERVO_ROLL_FLAPS_1_POS 16
#define SERVO_ROLL_FLAPS_2_POS 32
#define SERVO_PITCH_FLAPS_1_POS 96
#define SERVO_PITCH_FLAPS_2_POS 96

#define SERVO_ROLL_MIN_POS 64
#define SERVO_ROLL_MAX_POS 128
#define SERVO_PITCH_MIN_POS 64
#define SERVO_PITCH_MAX_POS 128
    
#define VCC_INT_REF 1080         
#define BAT_EMPTY 3000     
#define BAT_CRITICAL 2800
#define BAT_FULL 4050   
 
#define THROTTLE_MIN_SENSE 40

#define ON 1
#define OFF 0

#define SYSTEM_DATA_ADDR 5 
#define CONFIG_DATA_ADDR 10  

#define PACKAGE_SIZE 6
#define NO_DATA_MAX 2500

#define BUTT_1_BIT 0					//left button
#define BUTT_2_BIT 1  					//right button

#define FLAPS_FULL_UP 0
#define FLAPS_POS_1 1
#define FLAPS_POS_2 2

#define SELECTOR_LEFT_BIT 0				//1:1 control sensetivity scale
#define SELECTOR_CENTER_BIT 1			//1:2 control sensetivity scale	
#define SELECTOR_RIGHT_BIT 2			//1:4 control sensetivity scale

RF24 radio(19,18); 																		                            // "создать" модуль на пинах 19 и 18 Для Уно

//--------------------0-------1--------2-------3------4---------5--------6------
enum TX_data_pack {battery, spare1, spare2, spare3, spare4, rxEndFlag};
//--------------------0-------1------2-------3--------4---------5--------6------
enum RX_data_pack {throttle, roll, pitch, buttons, switcher, rxFlag};

/* структура для системных данных */
/*struct {
  uint16_t internal_ref;																// 1080 - value
} systemData;*/

bool rx_now;
uint8_t address[][2] = {"1Node","2Node"};//,"3Node","4Node","5Node","6Node"};			//возможные номера труб
uint8_t rx_data[PACKAGE_SIZE];
uint8_t tx_data[PACKAGE_SIZE];
uint8_t received_packs, received_packs_count;
uint16_t no_data_packs_count;
uint16_t charge;
uint8_t tx_tries;
uint8_t flaps;
uint8_t roll_2x_min, roll_2x_max, pitch_2x_min, pitch_2x_max;
uint8_t roll_4x_min, roll_4x_max, pitch_4x_min, pitch_4x_max;

void received_data_processing(void){
	static uint8_t buttons_prev_state;

//--- buttons parameters
	if (buttons_prev_state != rx_data[buttons]){
		 if (BitIsSet(rx_data[buttons], BUTT_2_BIT)){
		 	 if (flaps < FLAPS_POS_2) flaps++;
		 	}
		 if (BitIsSet(rx_data[buttons], BUTT_1_BIT)){
		 	 if (flaps > FLAPS_FULL_UP) flaps--;
		 	}
		}
	buttons_prev_state = rx_data[buttons];

//--- throttle parameters
	if ((rx_data[throttle] > THROTTLE_MIN_SENSE) && (charge > BAT_CRITICAL)) {
     TCCR0A = 0b00100011;
     MOTORS_PWM = rx_data[throttle];
    }
	else {
     MOTORS_PWM = 0;
     TCCR0A = 0b00000011;
    }

//--- flapperons parameters 
	if (flaps) {												//if flaps extnded
		if (flaps == FLAPS_POS_1){												
			if(BitIsSet(rx_data[switcher], SELECTOR_LEFT_BIT)){
			 SERVO_1_ROLL_PWM = constrain((SERVO_ROLL_FLAPS_1_POS + map(rx_data[roll], 0, 255, SERVO_ROLL_MIN_POS, SERVO_ROLL_MAX_POS)), SERVO_ROLL_MIN_POS, SERVO_ROLL_MAX_POS);
			 SERVO_2_ROLL_PWM = constrain((map(rx_data[roll], 0, 255, SERVO_ROLL_MIN_POS, SERVO_ROLL_MAX_POS) - SERVO_ROLL_FLAPS_1_POS), SERVO_ROLL_MIN_POS, SERVO_ROLL_MAX_POS);
			 SERVO_3_PITCH_PWM = map(rx_data[pitch], 0, 255, SERVO_PITCH_MIN_POS, SERVO_PITCH_MAX_POS);
			}

			if(BitIsSet(rx_data[switcher], SELECTOR_CENTER_BIT)){
			 SERVO_1_ROLL_PWM = constrain((SERVO_ROLL_FLAPS_1_POS + map(rx_data[roll], 0, 255, roll_2x_min, roll_2x_max)), SERVO_ROLL_MIN_POS, SERVO_ROLL_MAX_POS);
			 SERVO_2_ROLL_PWM = constrain((map(rx_data[roll], 0, 255, roll_2x_min, roll_2x_max) - SERVO_ROLL_FLAPS_1_POS), SERVO_ROLL_MIN_POS, SERVO_ROLL_MAX_POS);
			 SERVO_3_PITCH_PWM = map(rx_data[pitch], 0, 255, pitch_2x_min, pitch_2x_max);	 
			}	

			if(BitIsSet(rx_data[switcher], SELECTOR_RIGHT_BIT)){
			 SERVO_1_ROLL_PWM = constrain((SERVO_ROLL_FLAPS_1_POS + map(rx_data[roll], 0, 255, roll_4x_min, roll_4x_max)), SERVO_ROLL_MIN_POS, SERVO_ROLL_MAX_POS);
			 SERVO_2_ROLL_PWM = constrain((map(rx_data[roll], 0, 255, roll_4x_min, roll_4x_max) - SERVO_ROLL_FLAPS_1_POS), SERVO_ROLL_MIN_POS, SERVO_ROLL_MAX_POS);
			 SERVO_3_PITCH_PWM = map(rx_data[pitch], 0, 255, pitch_4x_min, pitch_4x_max);	 
			}		
		}
		else if (flaps == 2){
			if(BitIsSet(rx_data[switcher], SELECTOR_LEFT_BIT)){
			 SERVO_1_ROLL_PWM = constrain((SERVO_ROLL_FLAPS_2_POS + map(rx_data[roll], 0, 255, SERVO_ROLL_MIN_POS, SERVO_ROLL_MAX_POS)), SERVO_ROLL_MIN_POS, SERVO_ROLL_MAX_POS);
			 SERVO_2_ROLL_PWM = constrain((map(rx_data[roll], 0, 255, SERVO_ROLL_MIN_POS, SERVO_ROLL_MAX_POS) - SERVO_ROLL_FLAPS_2_POS), SERVO_ROLL_MIN_POS, SERVO_ROLL_MAX_POS);
			 SERVO_3_PITCH_PWM = map(rx_data[pitch], 0, 255, SERVO_PITCH_MIN_POS, SERVO_PITCH_MAX_POS);
			}

			if(BitIsSet(rx_data[switcher], SELECTOR_CENTER_BIT)){
			 SERVO_1_ROLL_PWM = constrain((SERVO_ROLL_FLAPS_2_POS + map(rx_data[roll], 0, 255, roll_2x_min, roll_2x_max)), SERVO_ROLL_MIN_POS, SERVO_ROLL_MAX_POS);
			 SERVO_2_ROLL_PWM = constrain((map(rx_data[roll], 0, 255, roll_2x_min, roll_2x_max) - SERVO_ROLL_FLAPS_2_POS), SERVO_ROLL_MIN_POS, SERVO_ROLL_MAX_POS);
			 SERVO_3_PITCH_PWM = map(rx_data[pitch], 0, 255, pitch_2x_min, pitch_2x_max);	 
			}	

			if(BitIsSet(rx_data[switcher], SELECTOR_RIGHT_BIT)){
			 SERVO_1_ROLL_PWM = constrain((SERVO_ROLL_FLAPS_2_POS + map(rx_data[roll], 0, 255, roll_4x_min, roll_4x_max)), SERVO_ROLL_MIN_POS, SERVO_ROLL_MAX_POS);
			 SERVO_2_ROLL_PWM = constrain((map(rx_data[roll], 0, 255, roll_4x_min, roll_4x_max) - SERVO_ROLL_FLAPS_2_POS), SERVO_ROLL_MIN_POS, SERVO_ROLL_MAX_POS);
			 SERVO_3_PITCH_PWM = map(rx_data[pitch], 0, 255, pitch_4x_min, pitch_4x_max);	 
			}				
		}
	} 	
	else{														//if no flaps
		if(BitIsSet(rx_data[switcher], SELECTOR_LEFT_BIT)){
		 SERVO_1_ROLL_PWM = map(rx_data[roll], 0, 255, SERVO_ROLL_MIN_POS, SERVO_ROLL_MAX_POS);
		 SERVO_2_ROLL_PWM = map(rx_data[roll], 0, 255, SERVO_ROLL_MIN_POS, SERVO_ROLL_MAX_POS);
		 SERVO_3_PITCH_PWM = map(rx_data[pitch], 0, 255, SERVO_PITCH_MIN_POS, SERVO_PITCH_MAX_POS);
		}

		if(BitIsSet(rx_data[switcher], SELECTOR_CENTER_BIT)){
		 SERVO_1_ROLL_PWM = map(rx_data[roll], 0, 255, roll_2x_min, roll_2x_max);
		 SERVO_2_ROLL_PWM = map(rx_data[roll], 0, 255, roll_2x_min, roll_2x_max);
		 SERVO_3_PITCH_PWM = map(rx_data[pitch], 0, 255, pitch_2x_min, pitch_2x_max);	 
		}	

		if(BitIsSet(rx_data[switcher], SELECTOR_RIGHT_BIT)){
		 SERVO_1_ROLL_PWM = map(rx_data[roll], 0, 255, roll_4x_min, roll_4x_max);
		 SERVO_2_ROLL_PWM = map(rx_data[roll], 0, 255, roll_4x_min, roll_4x_max);
		 SERVO_3_PITCH_PWM = map(rx_data[pitch], 0, 255, pitch_4x_min, pitch_4x_max);	 
		}
	}	
}	

void idle_output_values(void){ 
	 MOTORS_PWM = 0;
	 SERVO_1_ROLL_PWM = SERVO_1_ROLL_DEF_POS;
	 SERVO_2_ROLL_PWM = SERVO_2_ROLL_DEF_POS;
	 SERVO_3_PITCH_PWM = SERVO_3_PITCH_DEF_POS;	 
	 flaps = FLAPS_FULL_UP;
	}	
	
/*void standBy(void){
	 
	}*/

uint16_t getVcc(){             																	// чтение напряжения питания
	 static uint16_t buf[4];
	 static uint8_t i = 0;
	 
	 if (ADC_available()){
		 buf[i] = ADC_read();
		 Serial.println(buf[i]);
		 Serial.print("\n");		 
		 if (i < 3) i++;
		 else i = 0;
		 ADC_startConvert();			 
		}
	 else return;
	 
	 return (uint32_t)((VCC_INT_REF * 1024UL) / ((buf[0] + buf[1] + buf[2] + buf[3]) >> 2)); 	// считаем напряжение питания
	}	
	
uint16_t getVref(uint16_t vcc) {    															// чтение внуреннего опорного
	 uint16_t buf = 0;                 															// буфер для усредняющего фильтра
	 for (uint8_t i = 0; i < 8; i++) { 															// цикл усредняющего фильтра
		 ADC_startConvert();               														// запустить преобразование ацп
		 while (!ADC_available());          													// дождаться окончания
		 buf += ADC_read();             														// прочитать ацп
		}																
	 buf >>= 3;                        															// поделить на кол во итераций
	 return (uint32_t)((uint32_t) vcc * buf) >> 10;  											// рассчитать опорное
	}	

void serviceMode(){    																			// сервис режим (калибровка опорного)
	 uint16_t vcc = 4940;  																		// переменная для хранения реального напряжения питания
	 uint16_t vref;
	 
	 while (1) {           																		// бесконечный цикл									
		 vref = getVref(vcc);       											// рассчитать значение опорного из напряжения питания
			
			
		 Serial.print(F("Vcc: "));	
		 Serial.println(vcc);	
		 Serial.print(F("Ref: "));	
		 Serial.println(vref);	
		 Serial.print("\n");	
			
		 delay(1000);	
		}	
	}		
	
void _ADCInit(void){                  															// инициализация ацп
	 ADC_enable();			
	 ADC_setPrescaler(8);			
	 ADC_setReference(ADC_VCC);			
	
     ADMUX &= ~(1 << MUX0);																		//ADC_1V1 - внутреннее опорное 1.1В 
     ADMUX |= ((1 << MUX4) | (1 << MUX3) | (1 << MUX2) | (1 << MUX1));	
		
	 ADC_setResolution(10);			
	 delayMicroseconds(100);         															// даем паузу
	 ADC_startConvert();		
	}		
	
void NRF24_init (void){
	 radio.begin(); 																	      //активировать модуль
	 radio.setAutoAck(1);        														//режим подтверждения приёма, 1 вкл 0 выкл
	 radio.setRetries(0,3);     														//(время между попыткой достучаться, число попыток)
	 radio.enableAckPayload();   														//разрешить отсылку данных в ответ на входящий сигнал
	 radio.setPayloadSize(PACKAGE_SIZE);     										//размер пакета, в байтах

	 radio.openReadingPipe(1,address[0]);      											//хотим слушать трубу 0
	 radio.openWritingPipe(address[0]); 
	 
	 radio.setChannel(0x00);  															//выбираем канал (в котором нет шумов!)

	 radio.setPALevel (RF24_PA_MAX); 													//уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
	 radio.setDataRate (RF24_250KBPS); 													//скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
	 
	 radio.powerUp(); 																	//начать работу
	 radio.startListening();  															//начинаем слушать эфир, мы приёмный модуль
	}			

ISR (TIMER4_OVF_vect){
//  static uint16_t counter;
//  counter++;
//  if (counter > 1000){
	   digitalWrite(21, !digitalRead(21));
//     counter = 0;
//    }
}

void setup(){			
//	 Serial.begin(9600); 																		//открываем порт для связи с ПК
	 
	 _ADCInit();
	 
	 NRF24_init();
	 rx_now = true;
	 
	 pinMode(MOTORS_PIN, OUTPUT);	 
	 pinMode(SERVO_1_ROLL_PIN, OUTPUT);
	 pinMode(SERVO_2_ROLL_PIN, OUTPUT); 
	 pinMode(SERVO_3_PITCH_PIN, OUTPUT);
	 pinMode(21, OUTPUT);

	 TCCR0A = 0b00000011;																		        //timer 0 - fast PWM, 8 bit, prescaler = 256 (~305 Hz)
	 TCCR0B = 0b00000011;	
	 MOTORS_PWM = 0;
	 
	 SERVO_1_ROLL_PWM = SERVO_1_ROLL_DEF_POS;
	 SERVO_2_ROLL_PWM = SERVO_2_ROLL_DEF_POS;
	 SERVO_3_PITCH_PWM = SERVO_3_PITCH_DEF_POS;

	 TCCR1A = 0b10100011;                                           //timer 1 - fast PWM, 10 bit, prescaler = 256 (~61 Hz) - servo 1, servo 2
	 TCCR1B = 0b00001100;
  
   	 TCCR3A = 0b10000011;                                           //timer 3 - fast PWM, 10 bit, prescaler = 256 (~61 Hz) - servo 3
	 TCCR3B = 0b00001100;

	 TCCR4A = 0b00000000;                                           //timer 4 - fast PWM, 10 bit, prescaler = 256 (~61 Hz)
	 TCCR4B = 0b00001000;
	// TCCR4C = 0b00000000; 
	// TCCR4D = 0b00000000;
	// TCCR4E = 0b00000000;
	 TIMSK4 = 0b00000100;
	 OCR4C = 0xff;
//	 0CR4CL = 0xB7;

	 roll_2x_min = SERVO_ROLL_MIN_POS + ((SERVO_ROLL_MAX_POS - SERVO_ROLL_MIN_POS) >> 3);
	 roll_2x_max = SERVO_ROLL_MAX_POS - ((SERVO_ROLL_MAX_POS - SERVO_ROLL_MIN_POS) >> 3);
	 roll_4x_min = SERVO_ROLL_MIN_POS + ((SERVO_ROLL_MAX_POS - SERVO_ROLL_MIN_POS) >> 2);
	 roll_4x_max = SERVO_ROLL_MAX_POS - ((SERVO_ROLL_MAX_POS - SERVO_ROLL_MIN_POS) >> 2);
	 
	 pitch_2x_min = SERVO_PITCH_MIN_POS + ((SERVO_PITCH_MAX_POS - SERVO_PITCH_MIN_POS) >> 3);
	 pitch_2x_max = SERVO_PITCH_MAX_POS - ((SERVO_PITCH_MAX_POS - SERVO_PITCH_MIN_POS) >> 3);
	 pitch_4x_min = SERVO_PITCH_MIN_POS + ((SERVO_PITCH_MAX_POS - SERVO_PITCH_MIN_POS) >> 2);
	 pitch_4x_max = SERVO_PITCH_MAX_POS - ((SERVO_PITCH_MAX_POS - SERVO_PITCH_MIN_POS) >> 2);	 
//	 serviceMode();
	 sei();
	}

void loop(void) {
//	 if (charge < BAT_CRITICAL) standBy();
	
	 if (rx_now){
		 if(radio.available()){			 
			 radio.read(&rx_data, PACKAGE_SIZE);          								// чиатем входящий сигнал 
			 received_packs_count++;
			 no_data_packs_count = 0;
			}
		 else{
			 no_data_packs_count++;
			 if (no_data_packs_count > NO_DATA_MAX) received_packs = 0;
			}
			
		 if (rx_data[rxFlag]){
			 radio.stopListening();
			 rx_now = false;
			 received_packs = received_packs_count;
			 received_packs_count = 0;
			}	
		 
		}
	 else{
		 radio.write(&tx_data, PACKAGE_SIZE);
		 tx_tries++;
		 if (tx_tries > 11) {
			 radio.startListening(); 
			 tx_tries = 0;
			 rx_data[rxFlag] = false;
			 rx_now = true;
			 charge = getVcc();
			 tx_data[battery] = constrain(map(charge, BAT_EMPTY, BAT_FULL, 0, 255), 0, 255);		 
			}		 
		}

	 if (received_packs > 0) received_data_processing();
	 else idle_output_values();

//	 Serial.println(rx_data[0]);
//	 Serial.print("\n");		
	}

