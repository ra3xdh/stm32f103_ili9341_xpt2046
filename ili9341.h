#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/gpio.h"

#include "libopencm3/stm32/spi.h"


#define DC_PIN GPIO3
#define DC_PORT GPIOA
#define RESET_PIN GPIO2
#define RESET_PORT GPIOA
#define CS_PIN GPIO4
#define CS_PORT GPIOA
#define BL_PIN GPIO0
#define BL_PORT GPIOA

#define TFT_CS_LOW gpio_clear(CS_PORT, CS_PIN)
#define TFT_CS_HIGH gpio_set(CS_PORT, CS_PIN)
#define TFT_DC_LOW gpio_clear(DC_PORT, DC_PIN)
#define TFT_DC_HIGH gpio_set(DC_PORT, DC_PIN)
#define TFT_BL_OFF gpio_clear(BL_PORT, BL_PIN)
#define TFT_BL_ON gpio_set(BL_PORT, BL_PIN)
#define TFT_RST_ON gpio_clear(RESET_PORT, RESET_PIN)
#define TFT_RST_OFF gpio_set(RESET_PORT, RESET_PIN)


//Basic Colors
#define RED 0xf800
#define GREEN 0x07e0
#define BLUE 0x001f
#define BLACK 0x0000
#define YELLOW 0xffe0
#define WHITE 0xffff

//Other Colors
#define CYAN 0x07ff
#define BRIGHT_RED 0xf810
#define GRAY1 0x8410
#define GRAY2 0x4208

//TFT resolution 240*320
#define MIN_X 0
#define MIN_Y 0
#define MAX_X 239
#define MAX_Y 319

#define TS_MINX 116*2
#define TS_MAXX 890*2
#define TS_MINY 83*2
#define TS_MAXY 913*2

#define RGB(red, green, blue) ((unsigned int)( (( red >> 3 ) << 11 ) | (( green >> 2 ) << 5 ) | ( blue >> 3 )))

char readID(void);
void spi_setup(void);
char Read_Register(char Addr, char xParameter);
void sendCMD(char index);
void WRITE_DATA(char data);
void sendData(uint16_t data);
uint8_t spi_readwrite(uint32_t spi, uint8_t data);
void fillScreenALL(void);
void setCol(uint16_t StartCol,uint16_t EndCol);
void setPage(uint16_t StartPage,uint16_t EndPage);
void setXY(uint16_t poX, uint16_t poY);
void setPixel(uint16_t poX, uint16_t poY,uint16_t color);
void drawHorizontalLine( uint16_t poX, uint16_t poY,uint16_t length,uint16_t color);
void drawVerticalLine( uint16_t poX, uint16_t poY, uint16_t length,uint16_t color);
void fillScreen(uint16_t XL, uint16_t XR, uint16_t YU, uint16_t YD, uint16_t color);
uint16_t constrain(uint16_t X, uint16_t A, uint16_t B);
void fillRectangle(uint16_t poX, uint16_t poY, uint16_t length, uint16_t width, uint16_t color);
void drawLine( uint16_t x0,uint16_t y0,uint16_t x1, uint16_t y1,uint16_t color);
void drawCircle(uint16_t poX, uint16_t poY, uint16_t r,uint16_t color);
void fillCircle(uint16_t poX, uint16_t poY, uint16_t r,uint16_t color);
void drawChar( uint8_t ascii, uint16_t poX, uint16_t poY,uint16_t size, uint16_t fgcolor);
void drawString(char *string,uint16_t poX, uint16_t poY, uint16_t size,uint16_t fgcolor);
uint16_t RGBConv(uint16_t R, uint16_t G, uint16_t B);

void TFTinit(void);

