######################################################################
#  libwwg/src/Makefile for libwwg.a
######################################################################

SRCFILES	= uartlib.o miniprintf.o getline.o

TEMP1 		= $(patsubst %.c,%.o,$(SRCFILES))
TEMP2		= $(patsubst %.asm,%.o,$(TEMP1))
OBJS 		= $(patsubst %.cpp,%.o,$(TEMP2))

../libwwg.a: $(OBJS)
	@rm -f ../libwwg.a
	$(AR) rv ../libwwg.a $(OBJS)

uartlib.o: ../include/uartlib.h

include ../../MakefileF4.incl
include ../../Makefile.rtos

# End Makefile
