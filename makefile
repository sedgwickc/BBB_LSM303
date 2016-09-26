GPP=g++
CFLAGS= 
DEBUG=-Wall -g -DDEBUG

all: LSM

LSM:
	$(GPP) $(CFLAGS) application_LSM303.cpp Adafruit_LSM303.cpp -o LSM303 -lmraa

LSM_debug:
	$(GPP) $(CFLAGS) $(DEBUG) application_LSM303.cpp Adafruit_LSM303.cpp -o \
	    LSM303 -lmraa

clean:
	rm *.o LSM303 *.log
