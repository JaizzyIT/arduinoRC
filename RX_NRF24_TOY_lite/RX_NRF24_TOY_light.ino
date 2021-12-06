#include <directADC.h>
#include <EEPROM.h>
#include <GyverPower.h>
#include <TimerMs.h>
#include <SPI.h>
#include <Servo.h>
#include <directADC.h>
#include <bits_macros.h>

#include "nRF24L01.h"
#include "RF24.h"

#define MOTORS_PIN 3
#define MOTORS_PWM OCR0B
#define ROLL_SERVO1_PIN 5
//#define ROLL_SERVO2_PIN 6 
#define PITCH_SERVO_PIN 6
#define LED_PIN 2
#define ROLL_SERVO1_DEF_POS 102
//#define ROLL_SERVO2_DEF_POS 100
#define PITCH_SERVO_DEF_POS 70

#define ROLL_SERVO_MIN_POS 72
#define ROLL_SERVO_MAX_POS 132
#define PITCH_SERVO_MIN_POS 40
#define PITCH_SERVO_MAX_POS 145
    
#define VCC_INT_REF 1157         
#define BAT_EMPTY 3000     
#define BAT_CRITICAL 2800  
#define BAT_FULL 4000   
 
#define THROTTLE_MIN_SENSE 40

#define ON 1
#define OFF 0

#define SYSTEM_DATA_ADDR 5 
#define CONFIG_DATA_ADDR 10  

#define PACKAGE_SIZE 6
#define NO_DATA_MAX 2500

#define LEFT_BUTT_BIT 0
#define RIGHT_BUTT_BIT 1 

#define SELECTOR_LEFT_BIT 0
#define SELECTOR_CENTER_BIT 1
#define SELECTOR_RIGHT_BIT 2

//TimerMs vccTimer(VCC_INT, 1, 1);														// (период, мс), (0 не запущен / 1 запущен), (режим: 0 период / 1 таймер)
Servo servo_roll1;
//Servo servo_roll2;
Servo servo_pitch;

//RF24 radio(9,10);
RF24 radio(19,18); 																		// "создать" модуль на пинах 9 и 10 Для Уно

//--------------------0---------1------2-------3--------4-------5--------6------
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
uint8_t roll_2x_min, roll_2x_max, pitch_2x_min, pitch_2x_max;
uint8_t roll_4x_min, roll_4x_max, pitch_4x_min, pitch_4x_max;

void received_data_processing(void){
	 if ((rx_data[throttle] > THROTTLE_MIN_SENSE) && (charge > BAT_CRITICAL)){
		 TCCR0A = 0b00100011;	
		 MOTORS_PWM = rx_data[throttle];
		}
	 else {
		 MOTORS_PWM = 0;
		 TCCR0A = 0b00000011;	
		}	
	 
	 if(BitIsSet(rx_data[switcher], SELECTOR_LEFT_BIT)){
		 servo_roll1.write(map((255 - rx_data[roll]), 0, 255, ROLL_SERVO_MIN_POS, ROLL_SERVO_MAX_POS));
//		 servo_roll2.write(map(rx_data[roll], 0, 255, ROLL_SERVO_MIN_POS, ROLL_SERVO_MAX_POS));
		 servo_pitch.write(constrain(map(rx_data[pitch], 0, 255, PITCH_SERVO_MIN_POS, PITCH_SERVO_MAX_POS), 40, 140));		 
		}
		
	 if(BitIsSet(rx_data[switcher], SELECTOR_CENTER_BIT)){
		 servo_roll1.write(map((255 - rx_data[roll]), 0, 255, roll_2x_min, roll_2x_max));
//		 servo_roll2.write(map(rx_data[roll], 0, 255, roll_2x_min, roll_2x_max));
		 servo_pitch.write(map(rx_data[pitch], 0, 255, pitch_2x_min, pitch_2x_max));	 
		}	
		
	 if(BitIsSet(rx_data[switcher], SELECTOR_RIGHT_BIT)){
		 servo_roll1.write(map((255 - rx_data[roll]), 0, 255, roll_4x_min, roll_4x_max));
//		 servo_roll2.write(map(rx_data[roll], 0, 255, roll_4x_min, roll_4x_max));
		 servo_pitch.write(map(rx_data[pitch], 0, 255, pitch_4x_min, pitch_4x_max));	 
		}	
	 if (BitIsSet(rx_data[buttons], LEFT_BUTT_BIT)) {
		 digitalWrite(LED_PIN, LOW);
		 delay(20);																			//----------REDO!!!
		}
	 else digitalWrite(LED_PIN, HIGH);
	}	

void idle_output_values(void){ 
	 MOTORS_PWM = 0;
	 TCCR0A = 0b00000011;	
	 
	 servo_roll1.write(ROLL_SERVO1_DEF_POS);
//	 servo_roll2.write(ROLL_SERVO2_DEF_POS);
	 servo_pitch.write(PITCH_SERVO_DEF_POS);	
	 digitalWrite(LED_PIN, HIGH);
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
		 vref = getVref(vcc);       															// рассчитать значение опорного из напряжения питания
			
			
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
	 Serial.begin(9600);
	 
	 radio.begin(); 																	//активировать модуль
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

void setup(){			
//	 Serial.begin(9600); 																		//открываем порт для связи с ПК
	 
	 _ADCInit();
	 
	 NRF24_init();
	 rx_now = true;
	 
	 pinMode(MOTORS_PIN, OUTPUT);	 
	 pinMode(LED_PIN, OUTPUT);	
	 digitalWrite(LED_PIN, HIGH);
	 TCCR0A = 0b00000011;																		//timer 0 - fast PWM, 8 bit, prescaler = 256 (~305 Hz)
	 TCCR0B = 0b00000011;	
	 MOTORS_PWM = 0;
	 
	 servo_roll1.attach(ROLL_SERVO1_PIN);	 	 
//	 servo_roll2.attach(ROLL_SERVO2_PIN);	
	 servo_pitch.attach(PITCH_SERVO_PIN);

	 roll_2x_min = ROLL_SERVO_MIN_POS + ((ROLL_SERVO_MAX_POS - ROLL_SERVO_MIN_POS) >> 3);
	 roll_2x_max = ROLL_SERVO_MAX_POS - ((ROLL_SERVO_MAX_POS - ROLL_SERVO_MIN_POS) >> 3);
	 roll_4x_min = ROLL_SERVO_MIN_POS + ((ROLL_SERVO_MAX_POS - ROLL_SERVO_MIN_POS) >> 2);
	 roll_4x_max = ROLL_SERVO_MAX_POS - ((ROLL_SERVO_MAX_POS - ROLL_SERVO_MIN_POS) >> 2);
	 
	 pitch_2x_min = PITCH_SERVO_MIN_POS + ((PITCH_SERVO_MAX_POS - PITCH_SERVO_MIN_POS) >> 3);
	 pitch_2x_max = PITCH_SERVO_MAX_POS - ((PITCH_SERVO_MAX_POS - PITCH_SERVO_MIN_POS) >> 3);
	 pitch_4x_min = PITCH_SERVO_MIN_POS + ((PITCH_SERVO_MAX_POS - PITCH_SERVO_MIN_POS) >> 2);
	 pitch_4x_max = PITCH_SERVO_MAX_POS - ((PITCH_SERVO_MAX_POS - PITCH_SERVO_MIN_POS) >> 2);	 
//	 serviceMode();
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
		 radio.write(&tx_data,PACKAGE_SIZE);
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

