/*
 * APP_MANAGER.cpp
 *
 *  Created on: Nov 28, 2024
 *      Author: Daniel
 */


#include "app_manager.hpp"
#include "tim.h"
#include "usart.h"

#define BUFFER_LENGHT 50
#define FREQUENCY 25000
#define CPU_fREQUENCY 170000000
#define MAX_STEPS CPU_fREQUENCY/FREQUENCY
#define STEP MAX_STEPS / 50
#define TIMES 1

#define MEASURE_PERIOD 30 //Time in ms
#define STEP_PERIOD 1000 * 60 * 1 // Time in ms

float setpoint = 40;
uint16_t count = 0;
uint16_t last_count = 0;
uint16_t incremented = 0;
float average = 0;
uint32_t cooler_duty = 0, temperature_duty = 0;

uint32_t adc_value = 0;
float temperature = 0;

driverUARTConfig_t uart_cfg =
{
  .uartNumber = DRV_USART_PERIPHERAL_1,
  .handler = (void *) &huart1,
};

char buffer[BUFFER_LENGHT] = "";

DrvPIT m_tach_Timer;
DrvPIT m_pwm_Timer;
DrvPIT m_temperature_Timer;
DrvPIT m_resistor_Timer;

DrvADC * m_adc;
DrvPWM m_cooler(DRV_PWM_3, DRV_PWM_CH_2);
DrvPWM m_resistor(DRV_PWM_4, DRV_PWM_CH_1);
DrvGPIO m_tach(DRV_IO_PORT_A, (drvIoPin_t) DRV_IO_PIN_10);
DrvUART m_uart(&uart_cfg);

uint8_t random_vector[50] = {
    12, 34, 7, 45, 23, 5, 50, 29, 41, 8,
    3, 17, 39, 11, 6, 48, 25, 31, 20, 14,
    37, 4, 27, 9, 1, 40, 16, 32, 44, 18,
    2, 36, 10, 46, 21, 15, 28, 42, 0, 19,
    38, 35, 30, 43, 33, 26, 13, 24, 22, 47
};


uint32_t last_tick = 0, now = 0;
uint32_t pulse_period = 0;

void tachCallback(void *self, void * args)
{
//	count++;
  now = HAL_GetTick();
  pulse_period = now - last_tick;
  last_tick = now;
}


void loopPWMxVelocidade()
{
  if(m_tach_Timer.read())
  {
//    incremented += count;
    average = (float) ((float) count /  (float) TIMES) * (float) (1000 / MEASURE_PERIOD);
    count = 0;

    sprintf(buffer, "%d, %.2f\r\n", cooler_duty, average);
    m_uart.write((uint8_t *) buffer, strlen(buffer));

    m_tach_Timer.write(MEASURE_PERIOD, MilliSec);
  }

  if(m_pwm_Timer.read())
  {
    cooler_duty = m_cooler.getDuty();

    if(cooler_duty + STEP> MAX_STEPS) while(1);
    else m_cooler.setDuty(cooler_duty+STEP);

    m_pwm_Timer.write(STEP_PERIOD, MilliSec);
  }

}


void loopResistorxPWM()
{
  if(m_temperature_Timer.read()){
    m_adc->read(&adc_value);
    temperature = (3.3 * ((float)adc_value / (float)((2<<11)))) / 0.01;

    sprintf(buffer, "%d, %.2f\r\n", temperature_duty, temperature);
    m_uart.write((uint8_t *) buffer, strlen(buffer));

    m_temperature_Timer.write(MEASURE_PERIOD, MilliSec);
  }

  if(m_resistor_Timer.read()){
    temperature_duty = m_resistor.getDuty();
    if(temperature_duty + STEP> MAX_STEPS)  while(1){}
    else m_resistor.setDuty(temperature_duty+STEP);
    m_resistor_Timer.write(STEP_PERIOD, MilliSec);
  }
}


void loopResistorCooler()
{

  m_resistor.setDuty(MAX_STEPS);

  while(temperature < 40)
  {
    m_adc->read(&adc_value);
    temperature = (3.3 * ((float)adc_value / (float)((2<<11)))) / 0.01;
  }

  for(;;)
  {
    if(m_temperature_Timer.read()){
      m_adc->read(&adc_value);
      temperature = (3.3 * ((float)adc_value / (float)((2<<11)))) / 0.01;

      average = (float) ((float) count /  (float) TIMES) * (float) (1000 / MEASURE_PERIOD);
      count = 0;

      sprintf(buffer, "%d, %.2f, %.2f\r\n", m_cooler.getDuty(), temperature, average);
      m_uart.write((uint8_t *) buffer, strlen(buffer));

      m_temperature_Timer.write(MEASURE_PERIOD, MilliSec);
    }

    if(m_pwm_Timer.read())
    {
      cooler_duty = m_cooler.getDuty();

      if(cooler_duty + STEP> MAX_STEPS) while(1);
      else m_cooler.setDuty(cooler_duty+STEP);

      m_pwm_Timer.write(STEP_PERIOD, MilliSec);
    }
  }
}


float error_vector[10] = {0};
float cooler_pwm[10] = {0};
float resistor_pwm[10] = {0};
float rps = 0;

void velocityController()
{

  if(m_tach_Timer.read())
  {
//    rps = (float) ((float) count /  (float) TIMES) * (float) (1000 / (MEASURE_PERIOD));
    rps = 1000/pulse_period;
    float error = setpoint - rps;

    error_vector[1] = error_vector[0];
    error_vector[0] = error;
    count = 0;

    sprintf(buffer, "RPS:%.2f, Setpoint:%.2f, Zero:0.00\r\n", rps, setpoint);
    m_uart.write((uint8_t *) buffer, strlen(buffer));

    cooler_pwm[1] = cooler_pwm[0];
    cooler_pwm[0] = (6.237 * error_vector[0] - 6.22 * error_vector[1] + 0.9998 * cooler_pwm[1]);

    if(cooler_pwm[0] >= 0) m_cooler.setDuty((uint32_t) (cooler_pwm[0] * STEP + 1496));
    else m_cooler.setDuty(0);

    m_tach_Timer.write(MEASURE_PERIOD, MilliSec);
  }
}


void resistorController()
{
  if(m_temperature_Timer.read())
  {
    m_adc->read(&adc_value);
    temperature = (3.3 * ((float)adc_value / (float)((2<<11)))) / 0.01;

    error_vector[1] = error_vector[0];
    error_vector[0] = setpoint - temperature;

    resistor_pwm[1] = resistor_pwm[0];
    resistor_pwm[0] = 32.25 * error_vector[0] - 28.88 * error_vector[1] + 0.9383 * resistor_pwm[1];
    if(resistor_pwm[0] >= 0)  m_resistor.setDuty((uint32_t) (resistor_pwm[0] * STEP));
    else m_resistor.setDuty(0);


    sprintf(buffer, "Temperature:%.2f, Setpoint:%.2f, Zero:0.00\r\n", temperature, setpoint);
    m_uart.write((uint8_t *) buffer, strlen(buffer));

    m_temperature_Timer.write(MEASURE_PERIOD, MilliSec);
  }
}

int main(void)
{

  hardwareConfiguration();

  m_adc = new DrvADC (&hadc1, DRV_ADC_CHANNEL_1);

  m_resistor.start();
  m_resistor.setDuty(0);

  m_cooler.start();
  m_cooler.setDuty(0);

  DrvPIT::delay(2500);
  m_tach.setInterrupt(DRV_IO_IT_RISING, true, tachCallback, nullptr);

  m_tach_Timer.write(MEASURE_PERIOD, MilliSec);
  m_pwm_Timer.write(STEP_PERIOD, MilliSec);


  m_temperature_Timer.write(MEASURE_PERIOD, MilliSec);
  m_resistor_Timer.write(STEP_PERIOD, MilliSec);

  for(;;)
  {
//    loopPWMxVelocidade();
//    loopResistorxPWM();
//    loopResistorCooler();
    velocityController();
//    resistorController();
  }

  return 0;
}
