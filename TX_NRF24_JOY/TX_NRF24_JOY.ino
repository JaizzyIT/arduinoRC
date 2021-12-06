#define USE_MICRO_WIRE

#include <directADC.h>
#include <EEPROM.h>
#include <GyverPower.h>
#include <TimerMs.h>
#include <GyverButton.h>
#include <GyverOLED.h>
#include <SPI.h>
#include "nRF24L01.h"
#include "RF24.h"
#include <bits_macros.h>

#define PACKAGE_SIZE 6
#define RX_INT 1000
#define CONNECT_OK_LED 2
#define OLED_PWR1 3
#define OLED_PWR1 4

#define LEFT_BUTT_PIN 7
#define RIGHT_BUTT_PIN 8 
#define SELECTOR_LEFT_PIN 6 
#define SELECTOR_RIGHT_PIN 5

#define THROTTLE_STICK_AI ADC_A0
#define ROLL_STICK_AI ADC_A6
#define PITCH_STICK_AI ADC_A7

#define CONTR_INC 5         
#define VCC_INC 5           
#define VCC_INT 500    
#define TIM_INT 500       
#define BAT_EMPTY 3000      
#define BAT_FULL 4000    

#define SYSTEM_DATA_ADDR 5 
#define CONFIG_DATA_ADDR 10  

#define BUTT_LEFT_IND_ON oled.circle(70, 25, 4, OLED_FILL)
#define BUTT_LEFT_IND_OFF oled.circle(70, 25, 4, OLED_STROKE)
#define BUTT_RIGHT_IND_ON oled.circle(90, 25, 4, OLED_FILL)
#define BUTT_RIGHT_IND_OFF oled.circle(90, 25, 4, OLED_STROKE)

#define ON 1
#define OFF 0

#define LEFT_BUTT_BIT 0
#define RIGHT_BUTT_BIT 1 

#define SELECTOR_LEFT_BIT 0
#define SELECTOR_CENTER_BIT 1
#define SELECTOR_RIGHT_BIT 2

#define SEL_LEFT_IND_ON oled.rect(65, 40, 75, 50, OLED_FILL)
#define SEL_LEFT_IND_OFF oled.rect(65, 40, 75, 50, OLED_STROKE)
#define SEL_CENTER_IND_ON oled.rect(75, 40, 85, 50, OLED_FILL)
#define SEL_CENTER_IND_OFF oled.rect(75, 40, 85, 50, OLED_STROKE)
#define SEL_RIGHT_IND_ON oled.rect(85, 40, 95, 50, OLED_FILL)
#define SEL_RIGHT_IND_OFF oled.rect(85, 40, 95, 50, OLED_STROKE)

/* обьекты библиотек */
GyverOLED<SSD1306_128x64, OLED_BUFFER> oled;
GButton left_butt(LEFT_BUTT_PIN, HIGH_PULL);
GButton right_butt(RIGHT_BUTT_PIN, HIGH_PULL);
GButton selector_left(SELECTOR_LEFT_PIN, HIGH_PULL);
GButton selector_right(SELECTOR_RIGHT_PIN, HIGH_PULL);

TimerMs timer(TIM_INT, 1, 1);															// (период, мс), (0 не запущен / 1 запущен), (режим: 0 период / 1 таймер)
	
RF24 radio(9,10); 																			// "создать" модуль на пинах 9 и 10 Для Уно

//--------------------0-------1------2-------3--------4---------5--------6------
enum TX_data_pack {throttle, roll, pitch, buttons, switcher, rxFlag};

//--------------------0-------1-------2-------3--------4--------5--------6------
enum RX_data_pack {battery, spare1, spare2, spare3, spare4, rxEndFlag};

bool tx_now, get_vcc;
uint8_t address[][2] = {"1Node","2Node"};//,"3Node","4Node","5Node","6Node"};  					//возможные номера труб
uint8_t tx_data[PACKAGE_SIZE];
uint8_t rx_data[PACKAGE_SIZE];
uint8_t charge_indicator_local;// charge_val;
uint8_t charge_indicator_remote, charge_val_remote;
uint8_t throttle_val;
uint8_t roll_val;
uint8_t pitch_val;
uint8_t ADC_mux_switch = 0;
uint8_t buttons_state, switcher_state;
uint8_t received_packs, rx_tries, received_packs_avr;
uint8_t received_packs_arr[4];

uint16_t charge;

/* структура для системных данных */
struct {
  uint8_t contrast;
  uint16_t internal_ref;																	// 1049 - value
} systemData;

struct {
  uint8_t thr_max;
  uint8_t thr_min;
  uint8_t roll_max;
  uint8_t roll_min;
  uint8_t pitch_max;
  uint8_t pitch_min;  
} stickLimits;

void dataTransmitPrepare(void){
	 tx_data[throttle] = constrain(map(throttle_val, stickLimits.thr_min, stickLimits.thr_max, 0, 255), 0, 255);
	 tx_data[roll] = roll_val;
	 tx_data[pitch] = pitch_val;
	 tx_data[buttons] = buttons_state;
	 tx_data[switcher] = switcher_state;
	}

void NRF24_init(void){
	 radio.begin(); 																//активировать модуль
	 radio.setAutoAck(1);         													//режим подтверждения приёма, 1 вкл 0 выкл
	 radio.setRetries(0,15);     													//(время между попыткой достучаться, число попыток)
	 radio.enableAckPayload();    													//разрешить отсылку данных в ответ на входящий сигнал
	 radio.setPayloadSize(PACKAGE_SIZE);     										//размер пакета, в байтах

	 radio.openWritingPipe(address[0]);   											//мы - труба 0, открываем канал для передачи данных
	 radio.openReadingPipe(1,address[0]); 
	 
	 radio.setChannel(0x00);  														//выбираем канал (в котором нет шумов!)
	 
	 radio.setPALevel (RF24_PA_MAX); 												//уровень мощности передатчика. На выбор RF24_PA_MIN, RF24_PA_LOW, RF24_PA_HIGH, RF24_PA_MAX
	 radio.setDataRate (RF24_250KBPS); 												//скорость обмена. На выбор RF24_2MBPS, RF24_1MBPS, RF24_250KBPS
	 
	 radio.powerUp(); 																//начать работу
	 radio.stopListening();  														//не слушаем радиоэфир, мы передатчик
	 
	 tx_now = true;
	}

void _ADCInit(void){                  															// инициализация ацп
	 ADC_enable();		
	 ADC_setPrescaler(8);		
	 ADC_setReference(ADC_VCC);		
	 setAnalogMux(ADC_1V1);		
	 ADC_setResolution(8);		
	 delayMicroseconds(100);         														// даем паузу
	 ADC_startConvert();		
	}		
	
void oledEnable(void){                        													// 
//	 pinMode(OLED_PWR1, OUTPUT); 		
//	 pinMode(OLED_PWR2, OUTPUT); 		
	 oled.init();              																// инициализировать дисплей+
	 oled.clear();   																		// очистить дисплей (или буфер)
	 oled.setContrast(systemData.contrast);   												// отправить ему актуальную яркость
	}		

void buttonsTick(void){
	 left_butt.tick();
	 right_butt.tick();
	}
	
void buttonsProcessing(void){  																		// опрос всех кнопок
	 if (left_butt.isClick()) SetBit(buttons_state, LEFT_BUTT_BIT);
	 else ClearBit(buttons_state, LEFT_BUTT_BIT);
	 
	 if (right_butt.isClick()) SetBit(buttons_state, RIGHT_BUTT_BIT);
	 else ClearBit(buttons_state, RIGHT_BUTT_BIT);

	 if (!digitalRead(SELECTOR_LEFT_PIN)) SetBit(switcher_state, SELECTOR_LEFT_BIT);
	 else ClearBit(switcher_state, SELECTOR_LEFT_BIT);
	 
	 if (digitalRead(SELECTOR_RIGHT_PIN) and digitalRead(SELECTOR_LEFT_PIN)) SetBit(switcher_state, SELECTOR_CENTER_BIT);
	 else ClearBit(switcher_state, SELECTOR_CENTER_BIT);
	 
	 if (!digitalRead(SELECTOR_RIGHT_PIN)) SetBit(switcher_state, SELECTOR_RIGHT_BIT);
	 else ClearBit(switcher_state, SELECTOR_RIGHT_BIT);
	}

void dataSend(void){
	 radio.write(&tx_data,PACKAGE_SIZE);
	}

void displayUpdate (void){
	 uint8_t throttle_percent_val;//, roll_percent_val, pitch_percent_val;

	 throttle_percent_val = map(throttle_val, stickLimits.thr_min, stickLimits.thr_max, 0, 100);
	 
/*	 roll_percent_val = map(roll_val, 0, 255, 0, 100);
	 pitch_percent_val = map(pitch_val, 0, 255, 0, 100);*/
	
	 oled.clear();

//--- remote battery indicator	 
	 charge_val_remote = map(rx_data[battery], 0, 255, 0, 100);
	 charge_indicator_remote = constrain(map(rx_data[battery], 0, 255, 0, 30), 0, 30);

	 oled.rect(49, 0, 80, 6, OLED_STROKE);  													// тело индикатора
	 oled.fastLineV(81, 1, 5);  																// полосочка для красоты

	 for (uint8_t i = 0; i < charge_indicator_remote; i++) oled.line(50 + i, 0, 50 + i, 6);		// заполнение индикатора полосками  
	 
	 oled.setScale(1);
	 
	 if (charge_val_remote >= 100) oled.setCursor(57,1);
	 else if (charge_val_remote > 9) oled.setCursor(60,1);
		 else oled.setCursor(63,1);	 
	 oled.print(charge_val_remote);

//--- local battery indicator	 
//	 charge_indicator_local = constrain(map(charge, BAT_EMPTY, BAT_FULL, 0, 10), 0, 10);
//	 charge_val = constrain(map(charge, BAT_EMPTY, BAT_FULL, 0, 100), 0, 100);

	 oled.rect(0, 0, 11, 6, OLED_STROKE);  														// тело индикатора
	 oled.fastLineV(12, 1, 5);  																// полосочка для красоты
	 for (uint8_t j = 0; j < charge_indicator_local; j++) oled.line(1 + j, 0, 1 + j, 6);		// заполнение индикатора полосками
//	 oled.setCursor(3,1);
//	 oled.print("J");

//--- signal quality indicator
/*	 oled.setCursor(106,0);
	 oled.print(received_packs_avr);*/

	 do {																				
		 if(received_packs_avr > 0){
//			 oled.circle(118, 7, 6, OLED_CLEAR);
			 oled.fastLineV(111, 15, 15);
			 oled.fastLineV(112, 15, 15);
			 digitalWrite(CONNECT_OK_LED, LOW);
			}
		 else {
			 digitalWrite(CONNECT_OK_LED, HIGH);
			 oled.circle(118, 7, 6, OLED_STROKE);
			 oled.line(115, 4, 122, 11);
			 oled.setCursor(58,1);	 
			 oled.print(" ? ");			 
			 break;
			}
			
		 if(received_packs_avr > 2){	
			 oled.fastLineV(114, 12, 15);
			 oled.fastLineV(115, 12, 15);
			}
		 else break;
		 
		 if(received_packs_avr > 4){	
			 oled.fastLineV(117, 9, 15);
			 oled.fastLineV(118, 9, 15);
			}
		 else break;
		 
		 if(received_packs_avr > 6){	
			 oled.fastLineV(120, 6, 15);
			 oled.fastLineV(121, 6, 15);
			}
		 else break;
		 	
		 if(received_packs_avr > 8){	
			 oled.fastLineV(123, 3, 15);
			 oled.fastLineV(124, 3, 15);
			}
		 else break;
		 
		 if(received_packs_avr > 10){	
			 oled.fastLineV(126, 0, 15);
			 oled.fastLineV(127, 0, 15);
			}
		} while (false);
	 
//--- sticks indicators
	 oled.setScale(1);
	 oled.setCursor(10,3);
	 oled.print("THR"); 
	 oled.setScale(2);
	 if (throttle_percent_val >= 100) oled.setCursor(1,5);
	 else if (throttle_percent_val > 9) oled.setCursor(7,5);
		 else oled.setCursor(13,5);
	 oled.print(throttle_percent_val); 
//	 oled.print(throttle_val); 
//	 oled.fastLineV(1, 52, (52-(throttle_percent_val >> 2)));       						// 
/*
	 oled.setCursor(24,2);
	 oled.print("RL"); 
	 if (roll_percent_val >= 100)  oled.setCursor(21,7);
	 else if (roll_percent_val > 0) oled.setCursor(24,7);
		 else oled.setCursor(27,7);	 
	 oled.print(roll_percent_val); 
//	 oled.print(roll_val);
	 oled.fastLineV(29, 52, (52-(roll_percent_val >> 2)));       							// 

	 oled.setCursor(43,2);
	 oled.print("PT"); 	
	 if (pitch_percent_val >= 100) oled.setCursor(40,7);
	 else if (pitch_percent_val > 0) oled.setCursor(43,7);
		 else oled.setCursor(46,7);	 
	 oled.print(pitch_percent_val);
//	 oled.print(pitch_val);	 
	 oled.fastLineV(48, 52, (52-(pitch_percent_val >> 2)));       							// 
*/
//--- button indicators	 
/*	 if (!digitalRead(LEFT_BUTT_PIN)) BUTT_LEFT_IND_ON;
	 else BUTT_LEFT_IND_OFF;
	 
	 if (!digitalRead(RIGHT_BUTT_PIN)) BUTT_RIGHT_IND_ON;
	 else BUTT_RIGHT_IND_OFF;*/

//--- selector indicators	 
	 oled.setScale(1);
	 oled.setCursor(50,3);
	 oled.print("SENCE"); 
	 oled.setCursor(55,5);	
	 oled.setScale(2);
	 
	 if (!digitalRead(SELECTOR_LEFT_PIN)) oled.print("x1");
	 if (!digitalRead(SELECTOR_RIGHT_PIN)) oled.print("x4");
	 if (digitalRead(SELECTOR_RIGHT_PIN) and digitalRead(SELECTOR_LEFT_PIN)) oled.print("x2");

//--- trim indicator
	 oled.setScale(1);
	 oled.setCursor(100,3);
	 oled.print("TRIM"); 
	 oled.setCursor(100,5);	
	 oled.setScale(2);
	 
	 oled.print("+0");
	 
	 oled.update();
	}

void ADC_read_data(void){
	 
	 if(ADC_available()){
		 switch (ADC_mux_switch)
			{
			 case 0:
				 throttle_val = 255-(ADC_read8());
				 setAnalogMux(ROLL_STICK_AI);
				 ADC_mux_switch = 1;
				 delayMicroseconds(5);
				 break;
			 case 1:
				 roll_val = 255-(ADC_read8());				 
				 setAnalogMux(PITCH_STICK_AI);
				 ADC_mux_switch = 2;
				 delayMicroseconds(5);
				 break;
			 case 2:
				 pitch_val = ADC_read8();
				 setAnalogMux(THROTTLE_STICK_AI);
				 ADC_mux_switch = 3;
				 delay(1);
				 if (get_vcc) ADC_mux_switch = 3;   
				 break;
			 case 3:
				 throttle_val = 255-(ADC_read8());
				 charge = getVcc();
				 setAnalogMux(THROTTLE_STICK_AI);
				 ADC_mux_switch = 0;
				 get_vcc = false;
				 charge_indicator_local = constrain(map(charge, BAT_EMPTY, BAT_FULL, 0, 10), 0, 10);
				 delayMicroseconds(5);
				 break;
			}
		 ADC_startConvert();
		}
	}

uint16_t getVref(uint16_t vcc){    														// чтение внуреннего опорного
	 uint16_t buf = 0;                 														// буфер для усредняющего фильтра
	
	 for (uint8_t i = 0; i < 8; i++) { 														// цикл усредняющего фильтра
		 ADC_startConvert(); 	              												// запустить преобразование ацп
		 while (!ADC_available());
		 buf += ADC_read8();             													// прочитать ацп
		}
		
	 buf >>= 3;                        														// поделить на кол во итераций
	 return ((uint32_t) vcc * buf) >> 8;  													// рассчитать опорное
	}

uint16_t getVcc(){             															// чтение напряжения питания
	 static uint16_t buf[4];
	 static uint8_t i = 0;
	 
	 setAnalogMux(ADC_1V1);
	 delay(1);
	 ADC_startConvert();
	 while (!ADC_available());
	 buf[i] = ADC_read8();
	 if (i < 3) i++;
	 else i = 0;
	 
	 return (uint32_t)((systemData.internal_ref * 256UL) / ((buf[0] + buf[1] + buf[2] + buf[3]) >> 2)); 					// считаем напряжение питания
	}

void serviceMode(){    																	// сервис режим (калибровка опорного)
	 uint16_t vcc = 4200;  																	// переменная для хранения реального напряжения питания
	 
	 stickLimits.thr_max = 127;																//start values before adjustment
	 stickLimits.thr_min = 127;
	 roll_val

	 while (1) {           																	// бесконечный цикл
		 buttonsTick();      																// опросить кнопки
		 
		 if (!digitalRead(SELECTOR_LEFT_PIN)){
			 setAnalogMux(ADC_1V1);		
			 delayMicroseconds (50);
			 
			 if (left_butt.isClick() or left_butt.isHold()) {  									// нажатием или удержанием устанавливаем vcc
				 vcc = constrain(vcc + VCC_INC, 2000, 5500);								
				 } 
			 if (right_butt.isClick() or right_butt.isHold()) {								
				 vcc = constrain(vcc - VCC_INC, 2000, 5500);								
				 } 
			 if (left_butt.isHold() and right_butt.isHold()) {                         			// нажатие на ок
				 EEPROM.put(SYSTEM_DATA_ADDR, systemData);   									// сохраняет настройки
				 digitalWrite(CONNECT_OK_LED, LOW);
				 delay(100);
				 digitalWrite(CONNECT_OK_LED, HIGH);
				 }								
	 
			 systemData.internal_ref = getVref(vcc);       										// рассчитать значение опорного из напряжения питания
	 
			 oled.clear();                                 										// вывод значений
			 oled.home();
			 oled.setScale(2);
			 oled.print(F("Vcc: "));
			 oled.println(vcc);
			 oled.print(F("Ref: "));
			 oled.println(systemData.internal_ref);
			 oled.setScale(1);
			 oled.print
			 (F(
			 "\n"
			 "Press OK to save"
			 ));
			 oled.setCursor(0,7);
			 oled.print("Press LEFT/RIGHT to adj\n");
			}
			
		 if (digitalRead(SELECTOR_RIGHT_PIN) and digitalRead(SELECTOR_LEFT_PIN)){						//center position fo switcher
		 	 setAnalogMux(THROTTLE_STICK_AI);		
		 	 delayMicroseconds (50);
		 	 ADC_startConvert();  
			 
		 	 while (!ADC_available());
		 	 
		 	 if ((255 - ADC_read8()) > stickLimits.thr_max) stickLimits.thr_max = 255 - ADC_read8();
		 	 if ((255 - ADC_read8()) < stickLimits.thr_min) stickLimits.thr_min = 255 - ADC_read8();
		 	 
		 	 if (left_butt.isHold() and right_butt.isHold()){
		 		 EEPROM.put(CONFIG_DATA_ADDR, stickLimits); 	
		 		 digitalWrite(CONNECT_OK_LED, LOW);
		 		 delay(100);
		 		 digitalWrite(CONNECT_OK_LED, HIGH);
		 		}
		 	 if (left_butt.isClick()){ 
				 stickLimits.thr_max = 127;																//start values before adjustment
				 stickLimits.thr_min = 127;
				}
				
		 	 oled.clear();                                 												// вывод значений
		 	 oled.home();
		 	 oled.setScale(2);
			 oled.print(F("CURR:"));
		 	 oled.println(255 - ADC_read8());
		 	 oled.print(F("TH_MAX:"));
		 	 oled.println(stickLimits.thr_max);
		 	 oled.print(F("TH_MIN:"));
		 	 oled.println(stickLimits.thr_min);
		 	 oled.setScale(1);
		 	 oled.print
		 	 (F(
		 	 "Move throttle stick"
			 ));
			 oled.setCursor(0,7);
			 oled.print("Left butt - reset");
		 	}
			
		 if (digitalRead(SELECTOR_RIGHT_PIN)){						//center position fo switcher
		 	 setAnalogMux(THROTTLE_STICK_AI);		
		 	 delayMicroseconds (50);
		 	 ADC_startConvert();  
			 
		 	 while (!ADC_available());
		 	 
			 
			 
		 	 if ((255 - ADC_read8()) > stickLimits.thr_max) stickLimits.thr_max = 255 - ADC_read8();
		 	 if ((255 - ADC_read8()) < stickLimits.thr_min) stickLimits.thr_min = 255 - ADC_read8();
		 	 
		 	 if (left_butt.isHold() and right_butt.isHold()){
		 		 EEPROM.put(CONFIG_DATA_ADDR, stickLimits); 	
		 		 digitalWrite(CONNECT_OK_LED, LOW);
		 		 delay(100);
		 		 digitalWrite(CONNECT_OK_LED, HIGH);
		 		}
		 	 if (left_butt.isClick()){ 
				 stickLimits.thr_max = 127;																//start values before adjustment
				 stickLimits.thr_min = 127;
				}
				
		 	 oled.clear();                                 												// вывод значений
		 	 oled.home();
		 	 oled.setScale(2);
			 oled.print(F("CURR:"));
		 	 oled.println(255 - ADC_read8());
		 	 oled.print(F("TH_MAX:"));
		 	 oled.println(stickLimits.thr_max);
		 	 oled.print(F("TH_MIN:"));
		 	 oled.println(stickLimits.thr_min);
		 	 oled.setScale(1);
		 	 oled.print
		 	 (F(
		 	 "Move throttle stick"
			 ));
			 oled.setCursor(0,7);
			 oled.print("Left butt - reset");
		 	}
		 oled.update();
		}
	}	 

void setup(){
//	 Serial.begin(9600); 																//открываем порт для связи с ПК 
	 timer.setTimerMode();	
		
	 get_vcc = false;	
	 tx_now = true;	
		
	 EEPROM.get(SYSTEM_DATA_ADDR, systemData);  										// Читаем системные настройки из EEPROM
	 EEPROM.get(CONFIG_DATA_ADDR, stickLimits);  	
		
	 NRF24_init();	
	 _ADCInit();	
	 oledEnable();   									                            	// Включаем и инициализируем дисплей
	 pinMode(CONNECT_OK_LED, OUTPUT);
	 digitalWrite(CONNECT_OK_LED, HIGH);
	 if (!digitalRead(LEFT_BUTT_PIN)) serviceMode();         							// Если при старте кнопка LEFT нажата	
	 
	}

void loop(void) {
	 ADC_read_data();
	 buttonsTick(); 
	 buttonsProcessing();	 
	 dataTransmitPrepare();
	 
	 if (tx_now) {
		 dataSend();
		 if (tx_data[rxFlag]){
			 tx_now = false;
			 received_packs = 0;
			 radio.startListening(); 
			}
		}
	 else {
		 while(radio.available()){		 
			 radio.read(&rx_data, PACKAGE_SIZE);   
			 received_packs++;
			}
			
		 rx_tries++;
		 
		 if (rx_tries > 100){
			 static uint8_t i, buf;	
			 
			 received_packs_arr[i] = received_packs;
			 i++;
			 if (i > 3) i = 0;
			 
			 received_packs_avr = (received_packs_arr[0] + received_packs_arr[1] +received_packs_arr[2] +received_packs_arr[3]) >> 2;
				
/*			 Serial.print("Receive count: "); 
			 Serial.println(received_packs_avr); 
			 Serial.print("\n"); 
			 Serial.println(received_packs_arr[0]); 
			 Serial.println(received_packs_arr[1]); 
			 Serial.println(received_packs_arr[2]); 
			 Serial.println(received_packs_arr[3]); 
			 Serial.print("\n"); 
			 Serial.print("\n"); */
			 
			 rx_tries = 0;
			 tx_now = true;
			 tx_data[rxFlag] = false;
			 
			 radio.stopListening(); 
			 displayUpdate();
			}
		}
		
	 if (timer.tick()) {
		 tx_data[rxFlag] = true;
		 get_vcc = true;
		 timer.start();
		}	 	
	 
	}
