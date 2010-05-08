#
# Makefile for the ADC.
#

obj-$(CONFIG_EDDY_ADC)		+= eddy_adc.o

ifeq ($(CONFIG_EDDY_ADC_DEBUG),y)
EXTRA_CFLAGS += -DDEBUG
endif
