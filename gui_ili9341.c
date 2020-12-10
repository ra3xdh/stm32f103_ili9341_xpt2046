#include <stdio.h>
#include <errno.h>
#include <stdint.h>
#include <stdbool.h>


#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/gpio.h"
#include "libopencm3/stm32/usart.h"
#include "libopencm3/stm32/timer.h"
#include "libopencm3/cm3/nvic.h"
#include "libopencm3/stm32/exti.h"

#include "ili9341.h"
#include "xpt2046.h"
#include "ugui.h"
#include "delay.h"


int _write(int file, char *ptr, int len);

void clock_setup(void);
void usart_setup(void);
void gpio_setup(void);
void nvic_setup(void);
void exti_setup(void);
void timer_setup(void);

void pixel_set(UG_S16 x, UG_S16 y, UG_COLOR rgb);
UG_RESULT _HW_DrawLine(UG_S16 x1, UG_S16 y1, UG_S16 x2, UG_S16 y2, UG_COLOR rgb);
UG_RESULT _HW_FillFrame(UG_S16 x1, UG_S16 y1, UG_S16 x2, UG_S16 y2, UG_COLOR rgb);

void window_1_callback( UG_MESSAGE* msg );

void clock_setup(void)
{
    /* We are running on MSI after boot. */
    /* Enable GPIOD clock for LED &amp; USARTs. */
    rcc_clock_setup_in_hse_8mhz_out_72mhz(); // F103
    rcc_periph_clock_enable(RCC_GPIOA);
    rcc_periph_clock_enable(RCC_GPIOB);
    rcc_periph_clock_enable(RCC_GPIOC);

    /* Enable clocks for USART2. */
    rcc_periph_clock_enable(RCC_USART1);
    rcc_periph_clock_enable(RCC_AFIO);
}

void usart_setup(void)
{
    /* Setup USART2 parameters. */
    usart_set_baudrate(USART1, 115200);
    usart_set_databits(USART1, 8);
    usart_set_stopbits(USART1, USART_STOPBITS_1);
    usart_set_mode(USART1, USART_MODE_TX);
    usart_set_parity(USART1, USART_PARITY_NONE);
    usart_set_flow_control(USART1, USART_FLOWCONTROL_NONE);

    /* Finally enable the USART. */
    usart_enable(USART1);
}

void gpio_setup(void)
{

    /* Setup GPIO pin GPIO13 on GPIO port C for Green LED. */
    gpio_set_mode(GPIOC, GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_PUSHPULL, GPIO13); //F103
    gpio_set_mode(DC_PORT, GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_PUSHPULL, DC_PIN); //F103
    gpio_set_mode(RESET_PORT, GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_PUSHPULL, RESET_PIN); //F103
    gpio_set_mode(CS_PORT, GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_PUSHPULL, CS_PIN); //F103
    gpio_set_mode(BL_PORT, GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_PUSHPULL, BL_PIN); //F103


    /* Setup GPIO pins for USART2 transmit. */

    gpio_set_mode(GPIOA, GPIO_MODE_OUTPUT_50_MHZ,GPIO_CNF_OUTPUT_ALTFN_PUSHPULL, GPIO_USART1_TX); //F103

}



int _write(int file, char *ptr, int len)
{
    int i;

    if (file == 1) {
        for (i = 0; i < len; i++)
            usart_send_blocking(USART1, ptr[i]);
        return i;
    } // if (file == 1)

    errno = EIO;
    return -1;
}


void nvic_setup(void)
{
	nvic_enable_irq(NVIC_EXTI0_IRQ);
	nvic_enable_irq(NVIC_TIM2_IRQ);
	nvic_set_priority(NVIC_EXTI0_IRQ,1);
}

void exti_setup(void)
{
	/* Set GPIO0 (in GPIO port A) to 'input float'. */
	gpio_set_mode(GPIOB, GPIO_MODE_INPUT, GPIO_CNF_INPUT_FLOAT, GPIO0);

	/* Configure the EXTI subsystem. */
	exti_select_source(EXTI0, GPIOB);
	exti_set_trigger(EXTI0, EXTI_TRIGGER_FALLING);
	exti_enable_request(EXTI0);
}

void exti0_isr(void)
{

    /*touch_state.pressed = true;
    touch_state.x = ts_get_x();
    touch_state.y = ts_get_y();*/
    //fillCircle(touch_state.x,touch_state.y,3,RED);

    //delay_tck(10000);
    //gpio_toggle(GPIOC,GPIO13);

    exti_reset_request(EXTI0);
}

void timer_setup(void)
{

    rcc_periph_clock_enable(RCC_TIM2);
	TIM_CNT(TIM2) = 1;
	/* Set timer prescaler. 72MHz/1440 => 50000 counts per second. */
	TIM_PSC(TIM2) = 1440;
	TIM_ARR(TIM2) = 500;
	TIM_DIER(TIM2) |= TIM_DIER_UIE;
	TIM_CR1(TIM2) |= TIM_CR1_CEN;

}

void tim2_isr(void)
{

    if (gpio_get(GPIOB,GPIO0) == 0) {
        uint16_t x = ts_get_x();
        uint16_t y = ts_get_y();
        //fillCircle(x, y, 3, RED);
        if ((x > 0 && x < 239) && (y > 0 && y < 319)) {
            UG_TouchUpdate(x, y, TOUCH_STATE_PRESSED);
        }
    } else {
        UG_TouchUpdate(-1, -1, TOUCH_STATE_RELEASED);
    }
    UG_Update();
	//gpio_toggle(GPIOC, GPIO13);   /* LED2 on/off. */
	TIM_SR(TIM2) &= ~TIM_SR_UIF; /* Clear interrrupt flag. */
}


static UG_GUI gui;
static UG_WINDOW window_1;
static UG_BUTTON button1_1;
static UG_BUTTON button1_2;

#define MAX_OBJECTS 10
UG_OBJECT obj_buff_wnd_1[MAX_OBJECTS];

void pixel_set(UG_S16 x, UG_S16 y, UG_COLOR rgb)
{
    uint16_t R = (rgb >> 16) & 0x0000FF;
    uint16_t G = (rgb >> 8) & 0x0000FF;
    uint16_t B = rgb & 0x0000FF;
    UG_COLOR RGB16 = RGBConv(R,G,B);
    setPixel(x,y,RGB16);
}

UG_RESULT _HW_DrawLine(UG_S16 x1, UG_S16 y1, UG_S16 x2, UG_S16 y2, UG_COLOR rgb)
{
    uint16_t R = (rgb >> 16) & 0x0000FF;
    uint16_t G = (rgb >> 8) & 0x0000FF;
    uint16_t B = rgb & 0x0000FF;
    UG_COLOR RGB16 = RGBConv(R,G,B);
    if (x1 == x2) {
        drawVerticalLine(x1,y1,y2-y1,RGB16);
    } else if (y1 == y2) {
        drawHorizontalLine(x1,y1,x2-x1,RGB16);
    } else {
        drawLine(x1,y1,x2,y2,RGB16);
    }
    return UG_RESULT_OK;
}

UG_RESULT _HW_FillFrame(UG_S16 x1, UG_S16 y1, UG_S16 x2, UG_S16 y2, UG_COLOR rgb)
{
    uint16_t R = (rgb >> 16) & 0x0000FF;
    uint16_t G = (rgb >> 8) & 0x0000FF;
    uint16_t B = rgb & 0x0000FF;
    UG_COLOR RGB16 = RGBConv(R,G,B);
    fillScreen(x1,x2,y1,y2,RGB16);
    return UG_RESULT_OK;
}

void window_1_callback( UG_MESSAGE* msg )
{
   if ( msg->type == MSG_TYPE_OBJECT )
   {
      if ( msg->id == OBJ_TYPE_BUTTON )
      {
         switch( msg->sub_id )
         {
            case BTN_ID_0: // Toggle green LED
            {
               gpio_toggle(GPIOC,GPIO13);
               break;
            }
            case BTN_ID_1: // Toggle red LED
            {
               gpio_toggle(GPIOC,GPIO13);
               break;
            }
            default : break;
         }
      }
   }
}


int main(void)
{
    clock_setup();
    nvic_setup();
    gpio_setup();
    //exti_setup();
    usart_setup();
    spi_setup();
    ts_spi_setup();


    TFT_BL_ON;
    delay_tck(100000);

    gpio_set(GPIOC, GPIO13); /* LED on/off */
    TFTinit();

    //fillScreenALL();

    UG_Init(&gui, pixel_set, 240, 320);

    UG_DriverRegister( DRIVER_DRAW_LINE, (void*)_HW_DrawLine );
    UG_DriverRegister( DRIVER_FILL_FRAME, (void*)_HW_FillFrame );
    UG_DriverEnable( DRIVER_DRAW_LINE );
    UG_DriverEnable( DRIVER_FILL_FRAME );

    UG_FillScreen(C_BLUE);

    UG_WindowCreate( &window_1, obj_buff_wnd_1, MAX_OBJECTS, window_1_callback );
    UG_WindowSetTitleText( &window_1, "uGUI @ STM32F103" );
    UG_WindowSetTitleTextFont( &window_1, &FONT_12X20 );

    UG_ButtonCreate( &window_1, &button1_1, BTN_ID_0, 10, 10, 110, 60 );
    UG_ButtonSetFont( &window_1, BTN_ID_0, &FONT_12X20 );
    UG_ButtonSetBackColor( &window_1, BTN_ID_0, C_LIME );
    UG_ButtonSetText( &window_1, BTN_ID_0, "Green\nLED" );

    UG_ButtonCreate( &window_1, &button1_2, BTN_ID_1, 10, 80, 110, 130 );
    UG_ButtonSetFont( &window_1, BTN_ID_1, &FONT_12X20 );
    UG_ButtonSetBackColor( &window_1, BTN_ID_1, C_RED );
    UG_ButtonSetText( &window_1, BTN_ID_1, "Red\nLED" );


    UG_WindowShow( &window_1 );

    timer_setup();

    //UG_Update();
    UG_WaitForUpdate();


    while (1) {
        //gpio_toggle(GPIOC, GPIO13);
        delay_tck(1600000);
    }

    return 0;
}
