/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "usbd_hid_core.h"
#include "usbd_usr.h"
#include "usbd_desc.h"

#define ALL_LED_PINS   (GPIO_Pin_12 | GPIO_Pin_13 | GPIO_Pin_14 | GPIO_Pin_15)
#define LEDS_N  4

static const uint16_t led[LEDS_N] = {
/*  LD4, green    LD3, orange    LD5, red      LD6, blue   */
    GPIO_Pin_12 , GPIO_Pin_13 , GPIO_Pin_14 , GPIO_Pin_15
};

static void initGPIO(void);
static void initTimer(void);
static void appInit(void);
static uint32_t configUSB(void);

static uint32_t delayTime;

static uint8_t button_state;
static uint8_t prev_button_state;
static uint8_t current_button;

__ALIGN_BEGIN USB_OTG_CORE_HANDLE  USB_OTG_dev __ALIGN_END; // The USB device

int main(void)
{

    button_state = 0;
    prev_button_state = 0;
    current_button = 0;
    delayTime = 10;
    appInit();

    while (1);

    return 0;
}

void appInit(void){
    initGPIO();
    initTimer();
    configUSB();
}


/******* The interrupt handler ***/
// As of now set to 1ms.. 
// Read the GPIO in to an 8 bit integer, send the report aka the 8 bit integer

void TIM7_IRQHandler(void)
{
    static uint32_t delayCounter = 0;
    static uint32_t logical_button_states; // This is used when sending the report.
    uint16_t inputRead; // Used as a temporary variable for the read

    if (delayCounter > 0) {
        delayCounter--;
    }
    else{

        /* Poll the controller for input */
        // use bitwise NOT for active-low buttons (i.e. push-to-ground)
        inputRead = GPIO_ReadInputData(GPIOA) & 0x01;
        button_state = (uint8_t)(inputRead);
        if (button_state != prev_button_state) {
            prev_button_state = button_state;
            // TODO use a define for number of buttons
            if (button_state) {
                if (++current_button > 31) {
                    current_button = 0;
                }
            }
        }
        logical_button_states = button_state << current_button;

        /* Send the input to USB */
        USBD_HID_SendReport (&USB_OTG_dev,
            &logical_button_states,
            4);
        if(logical_button_states){
            GPIO_SetBits(GPIOD, ALL_LED_PINS);
        } else {
            GPIO_ResetBits(GPIOD, ALL_LED_PINS);
        }

        delayCounter = delayTime;
    }
    
    /* Clear the TIM7 interrupt source or else the NVIC hardware
       sees the pending bit and thinks TIM7 is emiting the
       interupt again
       (mark this interrupt as handled) */
    TIM_ClearITPendingBit(TIM7, TIM_IT_Update);
}


void initGPIO(void){
    // Init the leds..
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOD, ENABLE);

    GPIO_InitTypeDef gpio_init;
    gpio_init.GPIO_Mode  = GPIO_Mode_OUT;
    gpio_init.GPIO_OType = GPIO_OType_PP;
    gpio_init.GPIO_Pin   = ALL_LED_PINS;
    gpio_init.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    gpio_init.GPIO_Speed = GPIO_Speed_100MHz;
    GPIO_Init(GPIOD, &gpio_init);

    // Init the GPIO for the sega controller
    // First configure the inputs
    RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);
    
    GPIO_InitTypeDef gpio_sega;
    gpio_sega.GPIO_Mode = GPIO_Mode_IN;
    gpio_sega.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 | GPIO_Pin_2 | GPIO_Pin_3 | GPIO_Pin_4 | GPIO_Pin_5; // Continue here..
    gpio_sega.GPIO_OType = GPIO_OType_PP;
    gpio_sega.GPIO_Speed = GPIO_Speed_100MHz;
    gpio_sega.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpio_sega);
    // Configure the output pin
    gpio_sega.GPIO_Mode = GPIO_Mode_OUT;
    gpio_sega.GPIO_Pin = GPIO_Pin_6; // Continue here..
    gpio_sega.GPIO_OType = GPIO_OType_PP;
    gpio_sega.GPIO_Speed = GPIO_Speed_100MHz;
    gpio_sega.GPIO_PuPd  = GPIO_PuPd_NOPULL;
    GPIO_Init(GPIOA, &gpio_sega);
}


// This timer is used when polling input from the GPIO

void initTimer(void){
    /* power on the TIM7 timer 
       (the microcontroller powers off every peripheral by default) */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM7, ENABLE);

    TIM_TimeBaseInitTypeDef timInitStruct;
    timInitStruct.TIM_CounterMode   = TIM_CounterMode_Up;
    
    /* Don't modify clock frequency, 
     * we do this by setting the prescaler value*/
    timInitStruct.TIM_ClockDivision = TIM_CKD_DIV1;
    
    /* According to the datasheet of the STM32F407xx
       (DM00037051.pdf, page 29, Table 3. Timer feature comparison)
       TIM7 runs at 84 MHz, that is it increments its value
       84000000 times in a second.
       By setting its prescaler to 1000 it increments only 
       84000 times in a second (1000 times slower).
       By setting its period (auto reload value)
       to 84, we make it increment 1000 times from 0 to 84 in a second.
       That is, it counts every millisecond from 0 to 84, then it interrupts.
       This way we get an interrupt every millisecond.
     */
    timInitStruct.TIM_Prescaler     = 1000;
    timInitStruct.TIM_Period        = 84;

    /* store this data into memory */
    TIM_TimeBaseInit(TIM7, &timInitStruct);
    /* enable the TIM7 interrupt source 
       this is not the same as enabling the actual interrupt routine,
       which we setup in init_interrupt(),
       it just enables TIM7 as interrupt source,
       that is it makes TIM7 emit interrupt signals
       when it overflows the TIM_Period value
     */
    TIM_ITConfig(TIM7, TIM_IT_Update, ENABLE);
    
    /* actually start the timer */
    TIM_Cmd(TIM7, ENABLE);
    
    /* From this point on, TIM7 runs and emits and interrupt signal
       every millisecond. Next step is to setup NVIC to actually
       catch the interrupt signal and branch to TIM7_IRQHandler. */
}


// This function enbles the interrupt generated from the timer..

void initInterrupt(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    /* which interupt number */
    NVIC_InitStructure.NVIC_IRQChannel    = TIM7_IRQn;
    /* enable or disable? */
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    /* lowest priority (highest is 0, lowest if 0x0F) */
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0x0F;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority        = 0x0F;
    
    /* Store this data into NVIC memory map
       This function, which is declared in misc.h,
       computes a few values and stores them at the address NVIC, 
       which is #defined in core_cm4.h.
       NVIC is just a memory address casted to an NVIC
       configurations structure (NVIC_Type). */
    NVIC_Init(&NVIC_InitStructure);
    
    /* From this point on and having setup TIM7 in init_timer, 
       the interrupt handler TIM7_IRQHandler is being called 
       every millisecond. */
}

// The set up method for USB!
static uint32_t configUSB(void)
{
  USBD_Init(&USB_OTG_dev,
            USB_OTG_FS_CORE_ID,
            &USR_desc, 
            &USBD_HID_cb, 
            &USR_cb);  
  return 0;
}


