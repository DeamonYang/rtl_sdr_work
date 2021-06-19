OBJ=$(wildcard ./lib/*.c  ./convenience/*.c)
CFLAGS=-I include -lusb-1.0 -lm -lpthread -lfftw3

all: rtl_sdr rtl_sdr_dump_data rtl_fm

rtl_sdr:$(OBJ) rtl_sdr.c 
	gcc -o $@ $^ $(CFLAGS) 

rtl_sdr_dump_data:$(OBJ) rtl_sdr_dump_data.c 
	gcc -o $@ $^ $(CFLAGS) 

rtl_fm:$(OBJ) rtl_fm.c
	gcc -o $@ $^ $(CFLAGS) 
clean:
	rm rtl_sdr rtl_sdr_dump_data rtl_fm
