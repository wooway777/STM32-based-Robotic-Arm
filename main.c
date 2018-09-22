/* Includes ------------------------------------------------------------------*/
#include "stm32f10x_conf.h"
#include "stm32f10x.h"
#include "stm32f10x_tim.h"
#include "lcd.h"
#include <math.h>

/** @addtogroup STM32F10x_StdPeriph_Examples
  * @{
  */

/** @addtogroup ADC_ADC1_DMA
  * @{
  */ 


/* Private typedef -----------------------------------------------------------*/
GPIO_InitTypeDef GPIO_InitStructure;
TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
TIM_OCInitTypeDef  TIM_OCInitStructure;

/* Private define ------------------------------------------------------------*/
#define ADC1_DR_Address    ((uint32_t)0x4001244C)

/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
u8 *adcstr = ADCString;
ADC_InitTypeDef ADC_InitStructure;
DMA_InitTypeDef DMA_InitStructure;
__IO uint16_t ADCConvertedValue;
ErrorStatus HSEStartUpStatus;

/* Global Constants ----------------------------------------------------------*/
float ht = 87;
float preht = 0;
float ht0 = 87;//41-134
float ht1 = 50;
float ht2 = 50;
float ht3 = 50;
float ht4 = 50;
float ht5 = 50;
u8 prsdly = 10;
u32 flag = 500;
u32 stepper = 0;
float step = 1;
float rspd = 100;
float dtcc;
float scale = 1000;
//float period = ?/1000;
float freq = 64;
float count = 0;//<65535
float prescal = 1440;
float tempper = 0;

/* Computation Variables -----------------------------------------------------*/
float x1 = 10;
float y1 = 10;
float z1 = 10;
float r1;
float x = 0;
float y = 0;
float r = 0;
float theta = 0;

float theta0 = 0;
float theta1 = 0;
float theta2 = 0;
float theta3 = 0;

float dtheta = 0;
float hdtheta = 0;

float t0 = 0;
float t1 = 0;
float t2 = 0;
float t3 = 0;
float t4 = 0;
float t5 = 0;

float prex1 = 0;
float prey1 = 0;
float prez1 = 0;
float pretheta0 = 0;
float pretheta1 = 0;
float pretheta2 = 0;
float pretheta3 = 0;
float preht4 = 0;
float preht5 = 0;
float predtheta = 0;
float prehdtheta = 0;

float safex1 = 0;
float safey1 = 0;
float safez1 = 0;
float safetheta = 0;
float safedtheta = 0;
float safehdtheta = 0;
float safeht4 = 0;
float safeht5 = 0;

/* Important Constants to be Determined --------------------------------------*/
float pi = 3.1415926535;
float l1 = 10.4;
float l2 = 9.75;
float l3 = 16.5;
float ht0_90 = 41;//41-134
float ht0_n90 = 134;
float ht1_0 = 130;
float ht1_180 = 39;
float ht2_0 = 135;
float ht2_n90 = 89;
float ht3_n90 = 127;
float ht3_90 = 39;
float ht4_n90 = 39;
float ht4_90 = 127;
float ht5_close = 40;
float ht5_open = 66;

float CCR0_Val = 0;
float CCR1_Val = 0;
float CCR2_Val = 0;
float CCR3_Val = 0;
float CCR4_Val = 0;
float CCR5_Val = 0;

u8 hold = 0;
u8 prehold = 0;
u8 controlState = 0;
u8 testingServo = 0;

/* Private function prototypes -----------------------------------------------*/
void RCC_Configuration(void);
void GPIO_Configuration(void);

void LongDelay(u32 nCount);
void Delayms(u32 m);
void Display(u32 num, u32 row);
void DisplayAll();
void buttonsPWM();
void computations();
void servoSet(u8 servoNum, float ht);
void check();
void pwmInit();
void variablesInit();
void showThetas();
void buttons();
void buttonsWelcome();
void buttonsX1Z1();
void buttonsY1Wrist();
void buttonsPawHold();
void buttonsFBOrth();
void servoMap(u8 servoNum);
void act();
void save();
void load();
void safety();
  
/* Private functions ---------------------------------------------------------*/

/**
  * @brief   Main program
  * @param  None
  * @retval None
  */
int main(void)
{
  SystemInit();
 
  /* Enable FSMC, GPIOA, GPIOD, GPIOE, GPIOF, GPIOG and AFIO clocks */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_FSMC, ENABLE);

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOD | 
                         RCC_APB2Periph_GPIOE | RCC_APB2Periph_GPIOF | 
                         RCC_APB2Periph_GPIOG | RCC_APB2Periph_AFIO, ENABLE);
  
  //Jotstick Enable
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  GPIO_Init(GPIOE, &GPIO_InitStructure);
  GPIO_Init(GPIOF, &GPIO_InitStructure);
  GPIO_Init(GPIOG, &GPIO_InitStructure); 
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11|GPIO_Pin_12|GPIO_Pin_13|GPIO_Pin_14|GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  //End of Joystick Enable
  
  STM3210E_LCD_Init(); 

  /* System clocks configuration ---------------------------------------------*/
  RCC_Configuration();

  /* GPIO configuration ------------------------------------------------------*/
  GPIO_Configuration(); 
  
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = DISABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel14 configuration */ 
  
  ADC_RegularChannelConfig(ADC1, ADC_Channel_14, 1, ADC_SampleTime_55Cycles5);

  /* Enable ADC1 */
  
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibaration register */   
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibaration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));

  /* Start ADC1 Software Conversion */ 
  
  /*Initializations*/
  //pwm
  pwmInit();
  //variables;
  variablesInit();
  computations();
  
  //test phase
  controlState = 0;
  testingServo = 0;
  
  while (1)
  {
    if(controlState!=5){
      save();
      buttons();
      safety();
      DisplayAll();
      act();
      stepper ++;
    }else{
      buttonsPWM();
      check();
      if(ht != preht){
        servoSet(testingServo,ht);
        preht = ht;
      }
      LCD_DrawString(0, 0, "pwm:", 6);    
      Display(ht, 0);
    }
  }
}

/**
  * @brief  Configures the different system clocks.
  * @param  None
  * @retval None
  */
void RCC_Configuration(void)
{
    /* RCC system reset(for debug purpose) */
  RCC_DeInit();

  /* Enable HSE */
  RCC_HSEConfig(RCC_HSE_ON);

  /* Wait till HSE is ready */
  HSEStartUpStatus = RCC_WaitForHSEStartUp();

  if(HSEStartUpStatus == SUCCESS)
  { 
    /* HCLK = SYSCLK */
    RCC_HCLKConfig(RCC_SYSCLK_Div1); 
  
    /* PCLK2 = HCLK */
    RCC_PCLK2Config(RCC_HCLK_Div1); 

    /* PCLK1 = HCLK/2 */
    RCC_PCLK1Config(RCC_HCLK_Div2);

    /* ADCCLK = PCLK2/4 */
    RCC_ADCCLKConfig(RCC_PCLK2_Div4); 
  
#ifndef STM32F10X_CL  
    /* PLLCLK = 8MHz * 7 = 56 MHz */
    RCC_PLLConfig(RCC_PLLSource_HSE_Div1, RCC_PLLMul_7);

#else
    /* Configure PLLs *********************************************************/
    /* PLL2 configuration: PLL2CLK = (HSE / 5) * 8 = 40 MHz */
    RCC_PREDIV2Config(RCC_PREDIV2_Div5);
    RCC_PLL2Config(RCC_PLL2Mul_8);

    /* Enable PLL2 */
    RCC_PLL2Cmd(ENABLE);

    /* Wait till PLL2 is ready */
    while (RCC_GetFlagStatus(RCC_FLAG_PLL2RDY) == RESET)
    {}

    /* PLL configuration: PLLCLK = (PLL2 / 5) * 7 = 56 MHz */ 
    RCC_PREDIV1Config(RCC_PREDIV1_Source_PLL2, RCC_PREDIV1_Div5);
    RCC_PLLConfig(RCC_PLLSource_PREDIV1, RCC_PLLMul_7);
#endif

    /* Enable PLL */ 
    RCC_PLLCmd(ENABLE);

    /* Wait till PLL is ready */
    while(RCC_GetFlagStatus(RCC_FLAG_PLLRDY) == RESET)
    {
    }

    /* Select PLL as system clock source */
    RCC_SYSCLKConfig(RCC_SYSCLKSource_PLLCLK);

    /* Wait till PLL is used as system clock source */
    while(RCC_GetSYSCLKSource() != 0x08)
    {
    }
  }

/* Enable peripheral clocks --------------------------------------------------*/
  /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* Enable ADC1 and GPIOC clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOC, ENABLE);
}

/**
  * @brief  Configures the different GPIO ports.
  * @param  None
  * @retval None
  */
void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure the INPUTS */

  /* Configure PC.04 (ADC Channel14) as analog input ------------*/
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_4;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);

  /* Configure PA.0 Wakeup Key as input -------------------------*/

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{ 
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


void LongDelay(u32 nCount)
{
  for(; nCount != 0; nCount--);
}


void Delayms(u32 m)
{
  u32 i;
  
  for(; m != 0; m--)	
       for (i=0; i<50000; i++);
}


void Display(u32 imputNum, u32 row){
  
  u8 char4=(imputNum/1)%10;
  u8 char3=(imputNum/10)%10;
  u8 char2=(imputNum/100)%10;
  u8 char1=(imputNum/1000)%10;
  
  
  LCD_DrawChar(row, 32, HexValueOffset[char1]);
  LCD_DrawChar(row, 40, HexValueOffset[char2]);
  LCD_DrawChar(row, 48, HexValueOffset[char3]);
  LCD_DrawChar(row, 56, HexValueOffset[char4]);
  
}

void DisplayAll(){
  if(stepper==flag){
    LCD_Clear();
  }
  switch(controlState){
  case 0:
    //Welcome
    LCD_DrawString(0, 0, "Welcome.", 8);
    LCD_DrawString(2, 0, "Press Wakeup Key", 16);
    LCD_DrawString(4, 0, "to continue.    ", 16);
  break;
  case 1:
    //x1, z1 control
    if(stepper < flag){
      LCD_DrawString(0, 0, "Up/Down:", 8);
      LCD_DrawString(2, 0, "Front/Back      ", 16);
      LCD_DrawString(4, 0, "Left/Right: ", 12);
      LCD_DrawString(6, 0, "Rise/Fall       ", 16);
    }else{
      showThetas();
    }
  break;
  case 2:
    //y1, wrist control
    if(stepper < flag){
      LCD_DrawString(0, 0, "Up/Down:", 8);
      LCD_DrawString(2, 0, "Anti/Clockwise  ", 16);
      LCD_DrawString(4, 0, "Left/Right: ", 12);
      LCD_DrawString(6, 0, "Left/Right      ", 16);
    }else{
      showThetas();
    }
  break;
  case 3:
    //paw, hold control
    if(stepper < flag){
      LCD_DrawString(0, 0, "Up/Down:", 8);
      LCD_DrawString(2, 0, "Hand Up/Down    ", 16);
      LCD_DrawString(4, 0, "Left/Right: ", 12);
      LCD_DrawString(6, 0, "Grab/Release    ", 16);
    }else{
      if(t3>=0){
        LCD_DrawString(0, 0, "t3:+", 6);    
        Display(t3, 0);
      }else{
        LCD_DrawString(0, 0, "t3:-", 6);    
        Display(-t3, 0);
      }
      if(t5>=0){
        LCD_DrawString(2, 0, "t5:+", 6);    
        Display(t5, 2);
      }else{
        LCD_DrawString(2, 0, "t5:-", 6);    
        Display(-t5, 2);
      }
      LCD_DrawString(4, 0, "hold:", 6);
      Display(hold, 6);
    }
  break;
  case 4:
    //test phase
      LCD_DrawString(0, 0, "Up/Down:", 8);
      LCD_DrawString(2, 0, "Forward/Backward", 16);
      LCD_DrawString(4, 0, "Left/Right: ", 12);
      LCD_DrawString(6, 0, "OrtLeft/OrtRight", 16);
  break;
  case 5:
    if(stepper < flag){
      
    }else{
      
    }
  break;
  }
  
  if(stepper>=(2*flag)){
    stepper = 0;
  }
}

void computations(){
  //theta0
  if(x1!=0){
    theta0 = atan2(y1,x1);
  }else{
    theta0 = 0;
  }
  //x,y,r
  if(x1>=0){
    x = sqrt(x1*x1 + y1*y1);
  }else{
    x = -sqrt(x1*x1 + y1*y1);
  }
  y = z1;
  r = sqrt(x*x + y*y);
  //theta
  if(hold == 0){
    if(x != 0){
      theta = atan2(y, x) - dtheta;
    }else{
      theta = pi/2 - dtheta;
    }
  }else{
    theta = theta - hdtheta;
    hdtheta = 0;
    prehdtheta = 0;
    if(x != 0){
      dtheta = atan2(y, x) - theta;
    }else{
      dtheta = pi/2 - theta;
    }
  }
  //theta1,theta2,theta3
  if(x != 0){
    theta1 = atan2(y, x) + acos((l1*l1 - l2*l2 + r*r)/(2*l1*r));
  }else{
    theta1 = pi/2 + acos((l1*l1 - l2*l2 + r*r)/(2*l1*r));
  }
  theta2 = - pi + acos((l1*l1 + l2*l2 - r*r)/(2*l1*l2));
  theta3 = theta - theta1 - theta2;
  
  //angle conversions
  t0 = theta0*180/pi;
  t1 = theta1*180/pi;
  t2 = theta2*180/pi;
  t3 = theta3*180/pi;
  t4 = ht4;
  t5 = ht5;
}

void check(){
  if(ht1<0){
    ht1=0;
  }
  if(ht1>scale){
    ht1=scale;
  }
}

void pwmInit(){
  tempper = 72000000/prescal;
  count = tempper/freq;
  
  CCR0_Val = count*ht0/scale;
  CCR1_Val = count*ht1/scale;
  CCR2_Val = count*ht2/scale;
  CCR3_Val = count*ht3/scale;
  CCR4_Val = count*ht4/scale;
  CCR5_Val = count*ht5/scale;
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE);
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_3 | GPIO_Pin_6 | GPIO_Pin_7;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  TIM_TimeBaseStructure.TIM_Period = (count-1);
  TIM_TimeBaseStructure.TIM_Prescaler = (prescal-1);
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure);
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR0_Val;//zzzzzzz
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM3, &TIM_OCInitStructure);
  
  TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
  TIM_OC2Init(TIM3, &TIM_OCInitStructure);
  
  TIM_Cmd(TIM3, ENABLE);
  
  //=======================================================================//
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM5, ENABLE);
  
  TIM_TimeBaseStructure.TIM_Period = (count-1);
  TIM_TimeBaseStructure.TIM_Prescaler = (prescal-1);
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM5, &TIM_TimeBaseStructure);
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR2_Val;//zzzzzzz
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC4Init(TIM5, &TIM_OCInitStructure);
  
  TIM_Cmd(TIM5, ENABLE);
  
  //=======================================================================//
  
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6 | GPIO_Pin_7 | GPIO_Pin_8 | GPIO_Pin_9;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  TIM_TimeBaseStructure.TIM_Period = (count-1);
  TIM_TimeBaseStructure.TIM_Prescaler = (prescal-1);
  TIM_TimeBaseStructure.TIM_ClockDivision = 0;
  TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;
  TIM_TimeBaseInit(TIM4, &TIM_TimeBaseStructure);
  
  TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1;
  TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable;
  TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
  TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;
  TIM_OC1Init(TIM4, &TIM_OCInitStructure);
  
  TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
  TIM_OC3Init(TIM4, &TIM_OCInitStructure);
  
  TIM_OCInitStructure.TIM_Pulse = CCR5_Val;
  TIM_OC4Init(TIM4, &TIM_OCInitStructure);
  
  TIM_Cmd(TIM4, ENABLE);
}

void servoSet(u8 servoNum, float ht){
  switch(servoNum){
    case 0:
      CCR0_Val = count*ht/scale;      
      TIM_OCInitStructure.TIM_Pulse = CCR0_Val;
      TIM_OC1Init(TIM3, &TIM_OCInitStructure);
    break;
    case 1:
      CCR1_Val = count*ht/scale;
      TIM_OCInitStructure.TIM_Pulse = CCR1_Val;
      TIM_OC2Init(TIM3, &TIM_OCInitStructure);
    break;
    case 2:
      CCR2_Val = count*ht/scale;      
      TIM_OCInitStructure.TIM_Pulse = CCR2_Val;
      TIM_OC4Init(TIM5, &TIM_OCInitStructure);
    break;
    case 3:
      CCR3_Val = count*ht/scale;
      TIM_OCInitStructure.TIM_Pulse = CCR3_Val;
      TIM_OC1Init(TIM4, &TIM_OCInitStructure);
    break;
    case 4:
      CCR4_Val = count*ht/scale;
      TIM_OCInitStructure.TIM_Pulse = CCR4_Val;
      TIM_OC3Init(TIM4, &TIM_OCInitStructure);
    break;
    case 5:
      CCR5_Val = count*ht/scale;
      TIM_OCInitStructure.TIM_Pulse = CCR5_Val;
      TIM_OC4Init(TIM4, &TIM_OCInitStructure);
    break;
  }
}

void servoMap(u8 servoNum){
  switch(servoNum){
    case 0:
      ht0 = (ht0_90 + ht0_n90)/2 + (ht0_90 - ht0_n90)*theta0/pi;
    break;
    case 1:
      ht1 = ht1_0 + (ht1_180 - ht1_0)*theta1/pi;
    break;
    case 2:
      ht2 = ht2_0 + (ht2_0 - ht2_n90)*theta2/(pi/2);
    break;
    case 3:
      ht3 = (ht3_n90 + ht3_90)/2 + (ht3_90 - ht3_n90)*theta3/pi;
    break;
    case 4:
      ;
    break;
    case 5:
      ;
    break;
  }
}

void variablesInit(){
  
  /*
  float l1 = 15;
  float l2 = 15;
  float l3 = 10;
  */
  
  x1 = 0;
  y1 = 0;
  z1 = l1+l2-1;
  theta0 = 0;//theta0 = atan2(y1, x1);
  dtheta = 0;
  theta = pi/2;//theta = atan2(y, x) + dtheta;
  hdtheta = 0;
  prex1 = 0;
  prey1 = 0;
  prez1 = 30;
  pretheta0 = 0;
  pretheta1 = pi/2;
  pretheta2 = 0;
  pretheta3 = 0;
  hold = 0;
  prehold = 0;
  ht4=83;
  ht5=50;
  controlState = 0;
}

void showThetas(){
  if(t0>=0){
    LCD_DrawString(0, 0, "t0:+", 6);    
    Display(t0, 0);
  }else{
    LCD_DrawString(0, 0, "t0:-", 6);    
    Display(-t0, 0);
  }
  if(t1>=0){
    LCD_DrawString(2, 0, "t1:+", 6);    
    Display(t1, 2);
  }else{
    LCD_DrawString(2, 0, "t1:-", 6);    
    Display(-t1, 2);
  }
  if(t2>=0){
    LCD_DrawString(4, 0, "t2:+", 6);    
    Display(t2, 4);
  }else{
    LCD_DrawString(4, 0, "t2:-", 6);    
    Display(-t2, 4);
  }
  if(t3>=0){
    LCD_DrawString(6, 0, "t3:+", 6);    
    Display(t3, 6);
  }else{
    LCD_DrawString(6, 0, "t3:-", 6);    
    Display(-t3, 6);
  }
}

void buttons(){
  switch(controlState){
  case 0:
    //Welcome
    buttonsWelcome();
  break;
  case 1:
    //x1, z1 control
    buttonsX1Z1();
    if(prex1!=x1||prez1!=z1){
      computations();
      prex1 = x1;
      prez1 = z1;
    }
  break;
  case 2:
    //y1, wrist control
    buttonsY1Wrist();
    if(prey1!=y1){
      computations();
      prey1 = y1;
    }
  break;
  case 3:
    //paw, hold control
    buttonsPawHold();
    if(predtheta!=dtheta||prehdtheta!=hdtheta||preht5!=ht5){
      computations();
      predtheta = dtheta;
      prehdtheta = hdtheta;
      preht5 = ht5;
    }
  break;
  case 4:
    //x1, z1 control
    buttonsFBOrth();
    if(prex1!=x1||prey1!=y1){
      computations();
      prex1 = x1;
      prey1 = y1;
    }
  break;
  case 5:
    ;
  break;
  }
}

void buttonsPWM(){
    if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11))
    {
      stepper = flag;
    }
    if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12))
    {
      stepper = flag;
      Delayms(prsdly);
      ht = 134;
    }
     if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13))
    {
      stepper = flag;
      Delayms(prsdly);
      ht = 41;
    }
     if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14))
    {
      stepper = flag;
      Delayms(prsdly);
      ht = ht + step;
    }
     if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15))
    {
      stepper = flag;
      Delayms(prsdly);
      ht = ht - step;
    }
}

void buttonsWelcome(){
    if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11))
    {
      Delayms(prsdly);
      LCD_Clear();
      controlState = 1;
    }
    if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12))
    {
      Delayms(prsdly);
      LCD_Clear();
      controlState = 1;
    }
     if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13))
    {
      Delayms(prsdly);
      LCD_Clear();
      controlState = 1;
    }
     if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14))
    {
      Delayms(prsdly);
      LCD_Clear();
      controlState = 1;
    }
     if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15))
    {
      Delayms(prsdly);
      LCD_Clear();
      controlState = 1;
    }
    if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
    {
      while(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)){;}
      Delayms(prsdly);
      LCD_Clear();
      controlState = 1;
    }
}

void buttonsX1Z1(){
    if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11))
    {
      stepper = flag;
      Delayms(prsdly);
      variablesInit();
      computations();
    }
    if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12))
    {
      stepper = flag;
      Delayms(prsdly);
      z1 = z1 + step;
    }
     if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13))
    {
      stepper = flag;
      Delayms(prsdly);
      z1 = z1 - step;
    }
     if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14))
    {
      stepper = flag;
      Delayms(prsdly);
      x1 = x1 + step;
    }
     if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15))
    {
      stepper = flag;
      Delayms(prsdly);
      x1 = x1 - step;
    }
    if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
    {
      while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)){ ; }
      stepper = 0;
      Delayms(prsdly);
      LCD_Clear();
      controlState = 2;
    }
}

void buttonsY1Wrist(){
    if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11))
    {
      stepper = flag;
      Delayms(prsdly);
      variablesInit();
      computations();
    }
    if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12))
    {
      stepper = flag;
      Delayms(prsdly);
      y1 = y1 + step;
    }
     if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13))
    {
      stepper = flag;
      Delayms(prsdly);
      y1 = y1 - step;
    }
     if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14))
    {
      stepper = flag;
      Delayms(prsdly);
      ht4 = ht4 + step;
    }
     if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15))
    {
      stepper = flag;
      Delayms(prsdly);
      ht4 = ht4 - step;
    }
    if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
    {
      while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)){ ; }
      stepper = 0;
      Delayms(prsdly);
      LCD_Clear();
      controlState = 3;
    }
}

void buttonsPawHold(){
    if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11))
    {
      stepper = flag;
      Delayms(prsdly);
      variablesInit();
      computations();
    }
    if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12))
    {
      stepper = flag;
      Delayms(prsdly);
      ht5 = ht5 + step;
      hold = 0;
    }
     if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13))
    {
      stepper = flag;
      Delayms(prsdly);
      ht5 = ht5 - step;
      hold = 1;
    }
     if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14))
    {
      stepper = flag;
      Delayms(prsdly);
      if(hold==0){
        dtheta = dtheta + step/50;
      }else{
        hdtheta = step/50;
      }
    }
     if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15))
    {
      stepper = flag;
      Delayms(prsdly);
      if(hold==0){
        dtheta = dtheta - step/50;
      }else{
        hdtheta = -step/50;
      }
    }
    if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
    {
      while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)){ ; }
      stepper = 0;
      Delayms(prsdly);
      LCD_Clear();
      controlState = 4;
    }
}

void buttonsFBOrth(){
    if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_11))
    {
      stepper = flag;
      Delayms(prsdly);
      variablesInit();
      computations();
    }
    if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_12))
    {
      stepper = flag;
      Delayms(prsdly);
      x1 = x1 - step*sin(theta0)*sqrt(x1*x1+y1*y1)/rspd;
      y1 = y1 + step*cos(theta0)*sqrt(x1*x1+y1*y1)/rspd;
    }
     if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_13))
    {
      stepper = flag;
      Delayms(prsdly);
      x1 = x1 + step*sin(theta0)*sqrt(x1*x1+y1*y1)/rspd;
      y1 = y1 - step*cos(theta0)*sqrt(x1*x1+y1*y1)/rspd;
    }
     if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_14))
    {
      stepper = flag;
      Delayms(prsdly);
      x1 = x1 + step*cos(theta0);
      y1 = y1 + step*sin(theta0);
    }
     if(!GPIO_ReadInputDataBit(GPIOB, GPIO_Pin_15))
    {
      stepper = flag;
      Delayms(prsdly);
      x1 = x1 - step*cos(theta0);
      y1 = y1 - step*sin(theta0);
    }
    if(GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0))
    {
      while (GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0)){ ; }
      stepper = 0;
      Delayms(prsdly);
      LCD_Clear();
      controlState = 1;
    }
}

void act(){
    if(pretheta0!=theta0){
      servoMap(0);
      servoSet(0,ht0);
      pretheta0 = theta0;
    }
    if(pretheta1!=theta1){
      servoMap(1);
      servoSet(1,ht1);
      pretheta1 = theta1;
    }
    if(pretheta2!=theta2){
      servoMap(2);
      servoSet(2,ht2);
      pretheta2 = theta2;
    }
    if(pretheta3!=theta3){
      servoMap(3);
      servoSet(3,ht3);
      pretheta3 = theta3;
    }
    if(preht4!=ht4){
      //servoMap(4);
      servoSet(4,ht4);
      preht4 = ht4;
    }
    //if(preht5!=ht5){
      //servoMap(5);
      servoSet(5,ht5);
      preht5 = ht5;
    //}
}

void save(){
  safex1 = x1;
  safey1 = y1;
  safez1 = z1;
  safetheta = theta;
  safedtheta = dtheta;
  safehdtheta = hdtheta;
  safeht4 = ht4;
  safeht5 = ht5;
}

void load(){
  x1 = safex1;
  y1 = safey1;
  z1 = safez1;
  theta = safetheta;
  dtheta = safedtheta;
  hdtheta = safehdtheta;
  ht4 = safeht4;
  ht5 = safeht5;
}

void safety(){
  r1=sqrt(x1*x1+y1*y1+z1*z1);
  if(r1>l1+l2-1){
    load();
    computations();
  }
  computations();
  if(theta0<-pi/2||theta0>pi/2||theta1<0||theta1>pi||theta2>0||theta2<-5*pi/6||theta3<-pi/2||theta3>pi/2||ht4<ht4_n90||ht4>ht4_90||ht5<ht5_close||ht5>ht5_open){
    load();
    computations();
  }
  if(t0<-90||t0>90||t1<0||t1>180||t2>0||t2<-150||t3<-90||t3>90){
    load();
    computations();
  }
}
/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/
