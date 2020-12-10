BINARY = gui_ili9341
OBJS += delay.o ili9341.o xpt2046.o ugui.o


OPENCM3_DIR=../libopencm3
LDSCRIPT = $(OPENCM3_DIR)/lib/stm32/f1/stm32f103x8.ld

include ../libopencm3.target.mk
