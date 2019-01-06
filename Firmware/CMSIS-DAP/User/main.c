//=====================================
//CMSIS-DAP v2.0 for ST-LinkV2.1/J-Link-OB board
//--------based on x893 source code
//--------2018-07-24 by RadioOperator
//--------2018-12-08 by HavenXie please use V2_1
//=====================================

#include <stdio.h>

#include <RTL.h>
#include <rl_usb.h>
#include <stm32f10x.h>

#define  __NO_USB_LIB_C
#include "usb_config.c"

#include "DAP_config.h"
#include "DAP.h"

#define USE_PWM_LED        0
#define USE_BOOTLOADER     0

#if (USE_BOOTLOADER == 1)
  #define APP_BASE 0x4000
#else
  #define APP_BASE 0x0
#endif

#if defined ( BLUEPILL )
#if defined ( SWD_REMAP )
#warning "BLUEPILL board: using Remapped SWD/SWC port, TDO-PB7, nRESET-PB6, TDI-PB5"
#else
#warning "BLUEPILL board: using SWD/TMS-PB9, SWC/TCK-PB8, TDO-PB7, nRESET-PB6, TDI-PB5"
#endif
#endif

#if  defined ( BOARD_STM32RF ) \
  || defined ( STLINK_V20 )    \
  || defined ( STLINK_V21 )    \
  || defined ( STLINK_V2A )
#warning "SWD mode only, JTAG mode disabled."
#else
#warning "SWD mode + JTAG mode"
#endif

void BoardInit(void);
uint8_t usbd_hid_process(void);
void Delayms(uint32_t delay);

extern void PIN_nRESET_OUT(uint8_t bit);

#if (USBD_CDC_ACM_ENABLE == 1)
extern int32_t USBD_CDC_ACM_PortInitialize(void);
extern void CDC_ACM_UART_to_USB(void);
#endif

uint8_t u8SysTick_Counter = 3;
uint8_t u8LedMode = 4;

#define LED_FLASH_ON()    SysTick->CTRL |= SysTick_CTRL_ENABLE_Msk  //turn-on SysTick, LED in flashing mode.
#define LED_FLASH_OFF()   SysTick->CTRL &= ~SysTick_CTRL_ENABLE_Msk //turn-off SysTick

#if defined ( BLUEPILL ) //Bluepill Board

void LedConnectedOn(void)     { LED_CONNECTED_PORT->BRR  = LED_CONNECTED_MASK;  } //Low active
void LedConnectedOff(void)    { LED_CONNECTED_PORT->BSRR = LED_CONNECTED_MASK;  }
void LedConnectedToggle(void) { LED_CONNECTED_PORT->ODR ^= LED_CONNECTED_MASK;  }

void LedRunningOn(void)       { LED_RUNNING_PORT->BRR    = LED_RUNNING_MASK;    } //Low active
void LedRunningOff(void)      { LED_RUNNING_PORT->BSRR   = LED_RUNNING_MASK;    }
void LedRunningToggle(void)   { LED_RUNNING_PORT->ODR   ^= LED_RUNNING_MASK;    }

#else //all other Board

void LedConnectedOn(void)     { LED_CONNECTED_PORT->BSRR = LED_CONNECTED_PIN;  } //High active
void LedConnectedOff(void)    { LED_CONNECTED_PORT->BRR  = LED_CONNECTED_PIN;  }
void LedConnectedToggle(void) { LED_CONNECTED_PORT->ODR ^= LED_CONNECTED_PIN;  }

void LedRunningOn(void)       { LED_RUNNING_PORT->BSRR   = LED_RUNNING_PIN;    } //High active
void LedRunningOff(void)      { LED_RUNNING_PORT->BRR    = LED_RUNNING_PIN;    }
void LedRunningToggle(void)   { LED_RUNNING_PORT->ODR   ^= LED_RUNNING_PIN;    }

#endif //#if defined ( BLUEPILL )

void usb_hdreset(void) 
{
    PORT_USB_CONNECT_SETUP();
}

void LEDS_Init(void)
{
#if defined LED_CONNECTED_RCC
    GPIO_InitTypeDef  ConnectedLED_InitStructure;
    ConnectedLED_InitStructure.GPIO_Pin = LED_CONNECTED_PIN;
    ConnectedLED_InitStructure.GPIO_Mode = LED_CONNECTED_MODE;
    ConnectedLED_InitStructure.GPIO_Speed = LED_CONNECTED_SPEED;
    GPIO_ResetBits(LED_CONNECTED_PORT,LED_CONNECTED_PIN);	
    GPIO_Init(LED_CONNECTED_PORT, &ConnectedLED_InitStructure);
#endif
 
#if defined LED_RUNNING_RCC    
    GPIO_InitTypeDef  RunningLED_InitStructure;
    RunningLED_InitStructure.GPIO_Pin = LED_RUNNING_PIN;
    RunningLED_InitStructure.GPIO_Mode = LED_RUNNING_MODE;
    RunningLED_InitStructure.GPIO_Speed = LED_RUNNING_SPEED;
    GPIO_ResetBits(LED_RUNNING_PORT,LED_RUNNING_PIN);	
    GPIO_Init(LED_RUNNING_PORT, &RunningLED_InitStructure);
#endif  
}

void LedConnectedOut(uint16_t bit)
{
    LedRunningOff();
    if(bit & 1)
        u8LedMode |= 0x01;
    else 
        u8LedMode &= ~0x01;
    LED_FLASH_ON();
}

void LedRunningOut(uint16_t bit)
{
    LedConnectedOff();
    if(bit & 1)
        u8LedMode |= 0x01;
    else 
        u8LedMode &= ~0x01;
    LED_FLASH_ON();
}

#if (USE_PWM_LED == 1)
uint8_t GetPinPos(uint16_t PINx)
{
    uint8_t pos = 0;
    assert_param(IS_GET_GPIO_PIN(PINx));
    if(PINx >= GPIO_Pin_8)
    {
        while((0x01<<(pos + 8)) != PINx) pos++;
        return pos + 8;  
    }
    else
    {
        while((0x01<<pos) != PINx) pos++;
        return pos; 
    }
}

void LedSwitchPWMMode(GPIO_TypeDef* GPIOx, uint16_t PINx, BOOL FLAG)    
{ 
    uint8_t mode = 0, pinpos = GetPinPos(PINx);
    if(pinpos >= 8)
    {
        GPIOx->CRH &= ~(0x0C << (4*(pinpos-8)));
        if(FLAG == __TRUE) GPIOx->CRH |= (0x0C << (4*(pinpos-8))); 
    }
    else
    {
        GPIOx->CRL &= ~(0x0C << (4*pinpos));
        if(FLAG == __FALSE) GPIOx->CRL |= (0x0C << (4*pinpos));  
    }
    GPIOx->BRR = PINx; 
} 

void TIM3_Int_Init(u16 arr,u16 psc)
{
    TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM3, ENABLE); //时钟使能

	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 计数到5000为500ms
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  10Khz的计数频率  
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM3, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
 
	TIM_ITConfig(TIM3,TIM_IT_Update,ENABLE ); //使能指定的TIM3中断,允许更新中断

	NVIC_InitStructure.NVIC_IRQChannel = TIM3_IRQn;  //TIM3中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;  //从优先级3级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);  //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器

	//TIM_Cmd(TIM3, ENABLE);  //使能TIMx外设					 
}
uint8_t cnt10ms = 8, flag = 1, cnt = 0, ledPwmOpen = 0;
//定时器3中断服务程序
void TIM3_IRQHandler(void)   //TIM3中断
{
	if (TIM_GetITStatus(TIM3, TIM_IT_Update) != RESET) //检查指定的TIM中断发生与否:TIM 中断源 
    {
		TIM_ClearITPendingBit(TIM3, TIM_IT_Update  );  //清除TIMx的中断待处理位:TIM 中断源 
        flag ? cnt++ : cnt--;
        if(cnt >= 99) flag = 0;
        if(cnt == 0)  flag = 1; 
        TIM_SetCompare2(TIM1, cnt); 
    }
}
//TIM1 PWM部分初始化 
//PWM输出初始化
//arr：自动重装值
//psc：时钟预分频数
void TIM1_PWM_Init(u16 arr,u16 psc)
{  
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);	//使能定时器1时钟
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);  //使能GPIO外设和AFIO复用功能模块时钟

//	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; 
//	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;  //复用推挽输出
//	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
//	GPIO_Init(GPIOA, &GPIO_InitStructure);//初始化GPIO
//  GPIO_ResetBits(GPIOA,GPIO_Pin_9);	
	
    //初始化TIM1
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值 
	TIM_TimeBaseStructure.TIM_ClockDivision = 0; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位
	
	//初始化TIM1 Channel2 PWM模式	 
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式2
 	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
    //TIM_OCInitStructure.TIM_Pulse = 5000;
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High; //输出极性:TIM输出比较极性高
    TIM_OCInitStructure.TIM_OCIdleState = TIM_OCIdleState_Set;
	TIM_OC2Init(TIM1, &TIM_OCInitStructure);  //根据T指定的参数初始化外设TIM3 OC2

	TIM_OC2PreloadConfig(TIM1, TIM_OCPreload_Enable);  //使能TIM1在CCR2上的预装载寄存器
 
	TIM_Cmd(TIM1, ENABLE);  //使能TIM1
    TIM_CtrlPWMOutputs(TIM1, DISABLE);
}
void LED_PWM_On(void)
{
    LedSwitchPWMMode(GPIOA, GPIO_Pin_9, __TRUE);
    TIM_Cmd(TIM3, ENABLE);
    TIM_CtrlPWMOutputs(TIM1, ENABLE);
}
void LED_PWM_Off(void)
{
    LedSwitchPWMMode(GPIOA, GPIO_Pin_9, __FALSE);
    TIM_Cmd(TIM3, DISABLE);
    TIM_CtrlPWMOutputs(TIM1, DISABLE);
}
#endif

void SysTick_Init(void)
{
  SysTick_Config(900000); //cale = 900000 * (1/9000000) = 100ms
  SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK_Div8); //Freq = 72/8 = 9MHz
  LED_FLASH_ON(); //turn-on SysTick
}

void SysTick_Handler(void)
{
    u8SysTick_Counter--;
    //***********IDLE Mode*********************************************************************************
    if(u8LedMode == 0)    
    {
        #if (USE_PWM_LED == 1)
        if(!ledPwmOpen) 
        {
            ledPwmOpen = 1;
            LED_PWM_On();                    //Connected LED: green led pwm,  for idle mode
        }
        #else
        LedConnectedOff();                   //Connected LED: green on red off,  for idle mode
        #endif
    }
    //***********NOT IDLE Mode******************************************************************************
    else                  
    {
        #if (USE_PWM_LED == 1)
        if(ledPwmOpen) 
        {
            ledPwmOpen = 0;
            LED_PWM_Off();
        }
        #endif
        if (u8LedMode & 0x01)     //Running LED Mode********************************************************
        {
            if (u8SysTick_Counter & 0x01)    //Running LED: 100ms on / 100ms off for debug or download, fast
                LedRunningToggle();
            else 
                LedRunningToggle();
        }
        else if (u8LedMode & 0x02)//Connected LED Mode******************************************************
        {
            u8LedMode &= ~0x02;
            if (u8SysTick_Counter & 0x01)    //Connected LED: 100ms on / 100ms off for CDC, fast
                LedConnectedToggle();
            else
                LedConnectedToggle();
        }
        else if(u8LedMode & 0x04) //Before connecte Mode****************************************************
        {
            if (u8SysTick_Counter & 0x01)    //Wait for USB Device to configure: 100ms on / 100ms off, fast
                LedConnectedToggle();
            else
                LedConnectedToggle();
        }
    }
}

CoreDescriptor_t * pCoreDescriptor;
const CoreDescriptor_t CoreDescriptor = {
    &LedConnectedOut,
    &LedRunningOut,
};

void UserAppInit(CoreDescriptor_t *core)
{
    pCoreDescriptor = core;
    DAP_Setup();
}

void UserAppAbort(void)
{
    DAP_TransferAbort = 1;
}

UserAppDescriptor_t * pUserAppDescriptor = NULL;
UserAppDescriptor_t UserAppDescriptor = {
    &UserAppInit,
    &DAP_ProcessCommand,
    &UserAppAbort
};

//==main=======================================================================
int main(void)
{
#if (USE_BOOTLOADER == 1)
    NVIC_SetVectorTable(FLASH_BASE, APP_BASE);//set NVIC_VectTab
#endif 

    SystemCoreClockUpdate();
    BoardInit();
    SysTick_Init(); //for LED flash

#if (USE_PWM_LED == 1)
    TIM1_PWM_Init(99, 71);     //PWM = 72000000/(99+1)/(71+1)=10Khz
    TIM3_Int_Init(9999,71);    //T = 10ms
#endif

    if (UserAppDescriptor.UserInit != NULL)
    {
        pUserAppDescriptor = &UserAppDescriptor;
        pUserAppDescriptor->UserInit((CoreDescriptor_t *)&CoreDescriptor);
    }
    //USB Device Initialization and connect
    usb_hdreset();
    usbd_init();
    usbd_connect(__TRUE);

    while (!usbd_configured());  // Wait for USB Device to configure
    Delayms(100);                // Wait for 100ms

    LedConnectedOff();
    u8LedMode = 0;

#if (USBD_CDC_ACM_ENABLE == 1)
    USBD_CDC_ACM_PortInitialize(); //initial CDC UART port
#endif

    while (1) //Main loop
    {
        usbd_hid_process(); //DAP process

    #if (USBD_CDC_ACM_ENABLE == 1)
        CDC_ACM_UART_to_USB(); //CDC, UART to USB
    #endif
    }
}

extern uint32_t __Vectors;

void HardFault_Handler(void);
void NMI_Handler(void)      __attribute((alias("HardFault_Handler")));
void MemManage_Handler(void)  __attribute((alias("HardFault_Handler")));
void BusFault_Handler(void)   __attribute((alias("HardFault_Handler")));
void UsageFault_Handler(void) __attribute((alias("HardFault_Handler")));
void SVC_Handler(void)      __attribute((alias("HardFault_Handler")));
void DebugMon_Handler(void)   __attribute((alias("HardFault_Handler")));
void PendSV_Handler(void)   __attribute((alias("HardFault_Handler")));

void HardFault_Handler(void)
{
    __disable_irq();
    __set_MSP(__Vectors);
    LEDS_Init();
    {
        register int count;
        for (count = 0; count < 5; count++)
        {
            LedRunningOn();
            Delayms(100);
            LedRunningOff();

            LedConnectedOn();
            Delayms(100);
            LedConnectedOff();

            Delayms(1000);
        }
    }
    NVIC_SystemReset();
}

/* Control USB connecting via SW  */
#ifdef PIN_USB_CONNECT_PORT
const GPIO_InitTypeDef INIT_PIN_USB_CONNECT = {
    PIN_USB_CONNECT_MASK,
    GPIO_Speed_50MHz,
    PIN_USB_MODE
};
#endif

void PORT_USB_CONNECT_SETUP(void)
{
#ifdef PIN_USB_CONNECT_PORT
    RCC->APB2ENR |= PIN_USB_CONNECT_RCC;
    PIN_USB_CONNECT_OFF();
    GPIO_INIT(PIN_USB_CONNECT_PORT, INIT_PIN_USB_CONNECT);
#endif
}

#if ( DAP_SWD != 0 )

#if ( DAP_JTAG != 0 )
const GPIO_InitTypeDef INIT_SWD_TDI = {
    PIN_TDI_MASK,
    (GPIOSpeed_TypeDef)0,
    GPIO_Mode_IN_FLOATING
};
#endif //#if ( DAP_JTAG != 0 )

const GPIO_InitTypeDef INIT_SWD_TDO = {
    PIN_TDO_MASK,
    (GPIOSpeed_TypeDef)0,
    GPIO_Mode_IN_FLOATING
};

const GPIO_InitTypeDef INIT_SWD_SWCLK_SWDIO = {
    PIN_SWCLK_TCK_MASK | PIN_SWDIO_TMS_MASK,
    GPIO_Speed_50MHz,
    GPIO_Mode_Out_PP
};

/** Setup SWD I/O pins: SWCLK, SWDIO, and nRESET.
  Configures the DAP Hardware I/O pins for Serial Wire Debug (SWD) mode:
  - SWCLK, SWDIO, nRESET to output mode and set to default high level.
  - TDI, TDO, nTRST to HighZ mode (pins are unused in SWD mode).
*/ 
void PORT_SWD_SETUP()
{
    PIN_SWCLK_TCK_PORT->BSRR = (PIN_SWCLK_TCK_MASK | PIN_SWDIO_TMS_MASK);
#if ( DAP_JTAG != 0 )
    GPIO_INIT(PIN_TDI_PORT,       INIT_SWD_TDI);
#endif  
    GPIO_INIT(PIN_TDO_PORT,       INIT_SWD_TDO);

#if defined ( BLUEPILL ) && defined ( SWD_REMAP )
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_DISABLE;
#endif

    GPIO_INIT(PIN_SWCLK_TCK_PORT, INIT_SWD_SWCLK_SWDIO);

    PIN_nRESET_OUT(0U);
    Delayms(100);
    PIN_nRESET_OUT(1U);
}

#endif //#if ( DAP_SWD != 0 )

#if ( DAP_JTAG != 0 )

const GPIO_InitTypeDef INIT_JTAG_TDO = {
    PIN_TDO_MASK,
    (GPIOSpeed_TypeDef)0,
    GPIO_Mode_IN_FLOATING
};

const GPIO_InitTypeDef INIT_JTAG_TCK_TMS = {
    PIN_SWCLK_TCK_MASK | PIN_SWDIO_TMS_MASK,
    GPIO_Speed_50MHz,
    GPIO_Mode_Out_PP
};

const GPIO_InitTypeDef INIT_JTAG_TDI = {
    PIN_TDI_MASK,
    GPIO_Speed_50MHz,
    GPIO_Mode_Out_PP
};

/** Setup JTAG I/O pins: TCK, TMS, TDI, TDO, nTRST, and nRESET.
  Configures the DAP Hardware I/O pins for JTAG mode:
  - TCK, TMS, TDI, nTRST, nRESET to output mode and set to high level.
  - TDO to input mode.
*/ 
void PORT_JTAG_SETUP()
{
    PIN_SWCLK_TCK_PORT->BSRR = PIN_SWCLK_TCK_MASK | PIN_SWDIO_TMS_MASK;
    PIN_TDI_PORT      ->BSRR = PIN_TDI_MASK;

    GPIO_INIT(PIN_TDO_PORT,       INIT_JTAG_TDO);
    GPIO_INIT(PIN_TDI_PORT,       INIT_JTAG_TDI);
  
#if defined ( BLUEPILL ) && defined ( SWD_REMAP )
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_DISABLE;
#endif
  
    GPIO_INIT(PIN_SWCLK_TCK_PORT, INIT_JTAG_TCK_TMS);

    PIN_nRESET_OUT(0U);
    Delayms(100);
    PIN_nRESET_OUT(1U);
}

const GPIO_InitTypeDef INIT_OFF_TDI = {
    PIN_TDI_MASK,
    (GPIOSpeed_TypeDef)0,
    GPIO_Mode_IN_FLOATING
};

#endif //#if ( DAP_JTAG != 0 )

const GPIO_InitTypeDef INIT_OFF_TCK_TMS = {
    PIN_SWCLK_TCK_MASK | PIN_SWDIO_TMS_MASK,
    (GPIOSpeed_TypeDef)0,
    GPIO_Mode_IN_FLOATING
};

const GPIO_InitTypeDef INIT_OFF_TDO = {
    PIN_TDO_MASK,
    (GPIOSpeed_TypeDef)0,
    GPIO_Mode_IN_FLOATING
};

const GPIO_InitTypeDef INIT_OFF_nRESET = {
    (PIN_nRESET_MASK),
    (GPIOSpeed_TypeDef)0,
    GPIO_Mode_IPU
};

/** Disable JTAG/SWD I/O Pins.
Disables the DAP Hardware I/O pins which configures:
- TCK/SWCLK, TMS/SWDIO, TDI, TDO, nTRST, nRESET to High-Z mode.
*/
void PORT_OFF()
{
#if defined ( BLUEPILL ) && defined ( SWD_REMAP )
    AFIO->MAPR |= AFIO_MAPR_SWJ_CFG_DISABLE;
#endif
  
    GPIO_INIT(PIN_SWCLK_TCK_PORT, INIT_OFF_TCK_TMS);
  
#if ( DAP_JTAG != 0 )
    GPIO_INIT(PIN_TDI_PORT,       INIT_OFF_TDI);
    GPIO_INIT(PIN_TDO_PORT,       INIT_OFF_TDO);
#endif
  
    GPIO_INIT(PIN_nRESET_PORT,    INIT_OFF_nRESET);
}

const GPIO_InitTypeDef INIT_PINS_A = {
    ( GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 |
    GPIO_Pin_4 | GPIO_Pin_5 | GPIO_Pin_6 | GPIO_Pin_7 |
    GPIO_Pin_8 | GPIO_Pin_9 | GPIO_Pin_10 
    //    GPIO_Pin_11 | GPIO_Pin_12 |   // USB pins
    //    GPIO_Pin_13 | GPIO_Pin_14 |   // SWD pins
    //GPIO_Pin_15
    ),
    (GPIOSpeed_TypeDef)0,
    GPIO_Mode_AIN
};

const GPIO_InitTypeDef INIT_PINS_B = {
    GPIO_Pin_All,
    (GPIOSpeed_TypeDef)0,
    GPIO_Mode_AIN
};

const GPIO_InitTypeDef INIT_PINS_C = {
    (GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15),
    (GPIOSpeed_TypeDef)0,
    GPIO_Mode_AIN
};

void BoardInit(void)
{
#if defined ( BLUEPILL ) && defined ( SWD_REMAP )
    //release JTAG-SWD Pins for GPIO
    RCC->APB2ENR |=  RCC_APB2ENR_AFIOEN;
    AFIO->MAPR   |=  AFIO_MAPR_SWJ_CFG_DISABLE;
#endif
    // Enable GPIO clock
    RCC->APB2ENR |= (RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN | RCC_APB2ENR_IOPCEN);
    // Reset all GPIO pins, except USB/SWD port
    GPIO_INIT(GPIOA, INIT_PINS_A);
    GPIO_INIT(GPIOB, INIT_PINS_B);
    GPIO_INIT(GPIOC, INIT_PINS_C);

    LEDS_Init();
}

void USBD_Error_Event(void)
{
    LedConnectedOn();
    LedRunningOn();

    usbd_connect(__FALSE);
    usbd_reset_core();

    HardFault_Handler();
}

//-----Soft reset + Hard reset-------------------------------------------------
#define PIN_SWCLK_SET PIN_SWCLK_TCK_SET
#define PIN_SWCLK_CLR PIN_SWCLK_TCK_CLR

#define RST_CLOCK_CYCLE()                  \
    PIN_SWCLK_CLR();                       \
    PIN_DELAY();                           \
    PIN_SWCLK_SET();                       \
    PIN_DELAY()

#define RST_WRITE_BIT(bit)                 \
    PIN_SWDIO_OUT(bit);                    \
    PIN_SWCLK_CLR();                       \
    PIN_DELAY();                           \
    PIN_SWCLK_SET();                       \
    PIN_DELAY()

#define RST_READ_BIT(bit)                  \
    PIN_SWCLK_CLR();                       \
    PIN_DELAY();                           \
    bit = PIN_SWDIO_IN();                  \
    PIN_SWCLK_SET();                       \
    PIN_DELAY()

#define PIN_DELAY() PIN_DELAY_SLOW(DAP_Data.clock_delay)

uint8_t RST_Transfer(uint32_t request, uint32_t data)
{
  uint32_t ack;                                                                 \
  uint32_t bit;                                                                 \
  uint32_t val;                                                                 \
  uint32_t parity;                                                              \
  uint32_t n;                                                                   \
  \
  /* Packet Request */                                                          \
  parity = 0U;                                                                  \
  RST_WRITE_BIT(1U);                     /* Start Bit */                        \
  bit = request >> 0;                                                           \
  RST_WRITE_BIT(bit);                    /* APnDP Bit */                        \
  parity += bit;                                                                \
  bit = request >> 1;                                                           \
  RST_WRITE_BIT(bit);                    /* RnW Bit */                          \
  parity += bit;                                                                \
  bit = request >> 2;                                                           \
  RST_WRITE_BIT(bit);                    /* A2 Bit */                           \
  parity += bit;                                                                \
  bit = request >> 3;                                                           \
  RST_WRITE_BIT(bit);                    /* A3 Bit */                           \
  parity += bit;                                                                \
  RST_WRITE_BIT(parity);                 /* Parity Bit */                       \
  RST_WRITE_BIT(0U);                     /* Stop Bit */                         \
  RST_WRITE_BIT(1U);                     /* Park Bit */                         \
  \
  /* Turnaround */                                                              \
  PIN_SWDIO_OUT_DISABLE();                                                      \
  for (n = DAP_Data.swd_conf.turnaround; n; n--) {                              \
    RST_CLOCK_CYCLE();                                                          \
  }                                                                             \
  \
  /* Acknowledge response */                                                    \
  RST_READ_BIT(bit);                                                            \
  ack  = bit << 0;                                                              \
  RST_READ_BIT(bit);                                                            \
  ack |= bit << 1;                                                              \
  RST_READ_BIT(bit);                                                            \
  ack |= bit << 2;                                                              \
  \
  /* Data transfer */                                                           \
  /* Turnaround */                                                              \
  for (n = DAP_Data.swd_conf.turnaround; n; n--) {                              \
    RST_CLOCK_CYCLE();                                                          \
  }                                                                             \
  PIN_SWDIO_OUT_ENABLE();                                                       \
  /* Write data */                                                              \
  val = data;                                                                   \
  parity = 0U;                                                                  \
  for (n = 32U; n; n--) {                                                       \
    RST_WRITE_BIT(val);              /* Write WDATA[0:31] */                    \
    parity += val;                                                              \
    val >>= 1;                                                                  \
  }                                                                             \
  RST_WRITE_BIT(parity);             /* Write Parity Bit */                     \
  PIN_SWDIO_OUT_ENABLE();                                                       \
  PIN_SWDIO_OUT(1U);                                                            \
  return ((uint8_t)ack);                                                        \
}

void vResetTarget(uint8_t bit)
{
  uint32_t i;
  //soft-reset for Cortex-M
  RST_Transfer(0x00000CC5, 0xE000ED0C); //set AIRCR address
  for (i=0; i<100; i++);
  RST_Transfer(0x00000CDD, 0x05FA0007); //set RESET data
  for (i=0; i<100; i++);
  RST_Transfer(0x00000CC5, 0xE000ED0C); //repeat
  for (i=0; i<100; i++);
  RST_Transfer(0x00000CDD, 0x05FA0007);
  
  if (bit & 1)  PIN_nRESET_HIGH();
  else          PIN_nRESET_LOW();
}


//=============END=====================
