/* mbed Microcontroller Library
 * Copyright (c) 2019 ARM Limited
 * SPDX-License-Identifier: Apache-2.0
 */
#include "mbed.h"
//#include "platform/mbed_thread.h"
#include "Sht31.h"
#define MAXIMUM_BUFFER_SIZE
using namespace std;

///////////////

LowPowerTicker lpTicker;

DigitalOut  myled(LED1);
//static BufferedSerial pc(USBTX, USBRX, 9600);
//AnalogIn    a_in(A0);///////////////////

AnalogIn my_adc(ARDUINO_UNO_A0); //D11 on board
int flag_warning_datchik;

//  Function prototypes --------------------------------------------------------
void sw_irq(void);
void LowPowerConfiguration(void);

void tickerIRQ (void)
{
         system_reset();
}

void sw_irq(void)
{
      system_reset();
}
///////////////////////

//sda, scl
Sht31   temp_sensor(I2C_SDA, I2C_SCL);
    float t, h, fire_float;
   
Thread thread1;
Thread thread2;
DigitalOut led1(LED1);

static UnbufferedSerial pc(USBTX, USBRX);
static UnbufferedSerial  dev(D8, D2);

int dev_RxLen=0;
int pc_RxLen=0;
uint8_t hh,tt,fire;
    
 char dev_RxBuf[64] = {0};
char pc_RxBuf[64] = {0};
  char buf4[64] = {0};
char buf_datchik[64] = {0};

  

// обработчики прерываний по приему байта с устройства и с компа- просто заполняют буферы свои- при достижении 64 байта в буфере -начинают заполнять снова с нуля
// нужны только для отладки - в рабочем режиме- тключить для экономии энергии
void dev_recv()
{
if (dev_RxLen<63){

    dev.read(&dev_RxBuf[dev_RxLen], sizeof(dev_RxBuf[dev_RxLen]));    //  Got 1 char
         
    dev_RxLen++;
}
else dev_RxLen=0;
    }

 
void pc_recv()
{
    if (pc_RxLen<63){
    pc.read(&pc_RxBuf[pc_RxLen], sizeof(pc_RxBuf[pc_RxLen]));  //  Got 1 char
    pc_RxLen++;
    }
else pc_RxLen=0;
}

void recieve_otvety()
{                   
   
     
        if (dev_RxLen>0)
        {    
    //нужно только для отладки -можно убрать
    //распечатать на консоль то что пришло  с рак811
    for (uint8_t i = 0; i < dev_RxLen; i++) {  
   pc.write(&dev_RxBuf[i], sizeof(dev_RxBuf[i]));}
 
     //распечатать на консоль то что пришло с компа    
 //   for (uint8_t i = 0; i < pc_RxLen; i++) {  
 //   pc.write(&pc_RxBuf[i], sizeof(pc_RxBuf[i]));}
    
        dev_RxLen=0;
        ThisThread::sleep_for(1s);
        }
   }
   

void read_datchik_and_send_to_server_lora()
{
   //  while (true) {  //УДАЛЯЕМ БЕСКОНЕЧНЫЙ ЦИКЛ
        t = temp_sensor.readTemperature();
        h = temp_sensor.readHumidity();
        fire_float = my_adc.read()*100;
        fire = round(fire_float);
        printf("fire= %d\n\r", fire);
        tt=round(t); //из-за того что %f не работает пришлось посылать округленные показания датчиков
        sprintf(buf_datchik,"temp= %d",tt); 
        for (uint8_t i = 0; i < 8; i++) {  
    pc.write(&buf_datchik[i], sizeof(buf_datchik[i]));}
     int hh=round(h);
        sprintf(buf_datchik,"himi= %d",hh); 
        for (uint8_t i = 0; i < 8; i++) {  
    pc.write(&buf_datchik[i], sizeof(buf_datchik[i]));}
    
    
    
          //послать округленные до целого данные с датчика темп-ры и без пробела с датчика влажности на  рак811 в hex-формате(посылаются 3байта=6символов)
   //  sprintf(buf4,"at+send=lora:1:%d %d\r\n",tt,hh);
      sprintf(buf4,"at+send=lora:1:%x%x%x\r\n",tt,hh,fire);    //15+6+2=23
    for (uint8_t i = 0; i < 23; i++) {  
    dev.write(&buf4[i], sizeof(buf4[i]));
    //продублировать это в консоль
    pc.write(&buf4[i], sizeof(buf4[i]));} 


          //послать округленные до целого данные с датчика влажности на  рак811 - попытка послать наканал 0 - так не работает 
          //это можно убрать
 //    sprintf(buf4,"at+send=lora:0:%d\r\n",hh);
//    for (uint8_t i = 0; i < 19; i++) {  
 //   dev.write(&buf4[i], sizeof(buf4[i]));
    //продублировать это в консоль
 //   pc.write(&buf4[i], sizeof(buf4[i]));} 
    
           
       ThisThread::sleep_for(2s);// ждем ответа сервера 2сек что данные дошли
//}
}



int main()
{
    pc.attach(&pc_recv, UnbufferedSerial::RxIrq);
    dev.attach(&dev_recv, UnbufferedSerial::RxIrq);
        
    pc.baud(115200);
    dev.baud(115200);
    
//фориат передачи -по умолчанию - поэтому не нужно
  //     pc.format(8, BufferedSerial::None,  1    );
   //     dev.format(8, BufferedSerial::None, 1    );
   //
   // подключиться к серверу (параметры подключения уже установлены везде -настройка передатчика, базовой станции, сервера )
   //и ждать 10сек успешного подключения -так как сервер отвечает не сразу
   sprintf(buf4,"at+join\r\n"); 
  for (uint8_t i = 0; i < 9; i++) {  
    dev.write(&buf4[i], sizeof(buf4[i]));
//распечатать на консоль то что послано на  рак811
pc.write(&buf4[i], sizeof(buf4[i]));
        }  
         ThisThread::sleep_for(2s); // ждем ответа сервера 2сек что подключился успешно
         
//потоки не запускаем, просто одноразово посылаем показания 
//    thread1.start(read_datchik_and_send_to_server_lora);
//    thread2.start(recieve_otvety);
    
read_datchik_and_send_to_server_lora();
recieve_otvety();

   lpTicker.attach(tickerIRQ, 10); // every 10 second


// уходит в глубокий сон
     printf("\r\n Deep-sleep mode...to exit deep sleep wait 10s or push User button\r\n");
            LowPowerConfiguration();

//прерывания от входа А1 - подключаем туда же на аналоговый датчик протечки 
//(А0- вход АЦП, А1-вход прерывания ). 0-нет воды, 1-есть вода
            InterruptIn my_irq(A1);
     while (my_irq.read() == 1) {;} //ждет пока сбросится влажность на датчике чтобы начать сначала
            ThisThread::sleep_for(100ms);
            my_irq.rise(sw_irq); // назначаем обработчик - работает по front импульса

            //------------ IMPORTANT-------------------
            // 1) removes the receive interrupt handler
//прерыванеие от консоли удаляем
            pc.enable_input(false);
//удаляем также прерывание от приемопердатчика!!!!!!
            dev.enable_input(false);
            // 2) and releases the deep sleep lock
            sleep_manager_can_deep_sleep();
            // 3) then enter Deep Sleep mode

while(1)
    {
      ThisThread::sleep_for(10s); // уйти в сон бесконечный 
    }



}
void LowPowerConfiguration(void)
{
#if defined(TARGET_NUCLEO_L152RE)
    RCC->AHBENR |=
        (RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN | RCC_AHBENR_GPIOCEN |
         RCC_AHBENR_GPIODEN | RCC_AHBENR_GPIOHEN);
#elif defined(TARGET_NUCLEO_L476RG)

#elif defined(TARGET_NUCLEO_F446RE)
    RCC->AHB1ENR |=
        (RCC_AHB1ENR_GPIOAEN | RCC_AHB1ENR_GPIOBEN | RCC_AHB1ENR_GPIOCEN |
         RCC_AHB1ENR_GPIODEN | RCC_AHB1ENR_GPIOHEN);
#endif
    GPIO_InitTypeDef GPIO_InitStruct;
    // All other ports are analog input mode
    GPIO_InitStruct.Pin = GPIO_PIN_All;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
    HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);
#if defined(TARGET_NUCLEO_L152RE)
    RCC->AHBENR &=
        ~(RCC_AHBENR_GPIOAEN | RCC_AHBENR_GPIOBEN |RCC_AHBENR_GPIOCEN |
          RCC_AHBENR_GPIODEN | RCC_AHBENR_GPIOHEN);
#elif defined(TARGET_NUCLEO_L476RG)
    RCC->AHB1ENR = 0;
    RCC->AHB2ENR = 0;
    RCC->AHB3ENR = 0;
    RCC->APB1ENR2 = 0;
    RCC->APB2ENR = 0;
#elif defined(TARGET_NUCLEO_F446RE)
    RCC->AHB1ENR = 0;
    RCC->AHB2ENR = 0;
    RCC->AHB3ENR = 0;
    RCC->APB1ENR = 0x8;     // alive TIM5
    RCC->APB2ENR = 0;
    RCC->AHB1LPENR = 0;
    RCC->AHB2LPENR = 0;
    RCC->APB1LPENR = 0x8;   // alive TIM5
    RCC->APB2LPENR = 0;
#endif
}


