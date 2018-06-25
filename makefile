#INCLUDEDIRA 	:= $(shell pwd)/infrastructure/vmf/include
#INCLUDEDIRB 	:= $(shell pwd)/infrastructure/vgw/public
#INCLUDEDIRC 	:= $(shell pwd)/infrastructure/osal/public
#INCLUDEDIRD 	:= $(shell pwd)/infrastructure/osal/protected
#INCLUDEDIRE 	:= $(shell pwd)/infrastructure/osal/cfg
INCLUDEDIRF 	:= $(shell pwd)/tslib/include
#LIBSDIRA	:= $(shell pwd)/infrastructure/vmf/libs
LIBSDIRB	:= $(shell pwd)/tslib/lib
CFLAGS 		:= -Wall -O2
#CPPFLAGS   	:= -I$(INCLUDEDIRA) -I$(INCLUDEDIRB) -I$(INCLUDEDIRC) -I$(INCLUDEDIRD) -I$(INCLUDEDIRE) -I$(INCLUDEDIRF)
#LIBFLAGS   	:= -L$(LIBSDIRA) -L$(LIBSDIRB)
CPPFLAGS   	:= -I$(INCLUDEDIRF)
LIBFLAGS   	:= -L$(LIBSDIRB)

Mycerclient : Mycerclient.c myfb.c Screenshot.c
	arm-poky-linux-gnueabi-gcc -march=armv7-a -mfpu=neon -mfloat-abi=hard -mcpu=cortex-a9 --sysroot=/opt/fsl-imx-fb/4.1.15-2.0.0/sysroots/cortexa9hf-neon-poky-linux-gnueabi $(CPPFLAGS) $(CFLAGS) $(LIBFLAGS) -o Serclient -lpthread -lrt -lts Mycerclient.c myfb.c Screenshot.c

clean:
	rm -f Serclient
