obj-m := mu.o

default: mu

all: clean mu

clean:
	make -C ${KERNEL_PATH} M=$(PWD) clean

mu:
	ARCH=arm make -C ${KERNEL_PATH} M=$(PWD) EXTRA_CFLAGS="-DDEBUG" modules
