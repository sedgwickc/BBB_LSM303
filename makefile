GPP=g++
CFLAGS=
DEBUG=-Wall -g -DDEBUG
LINKER=-lmraa

all: LSM

LSM:
	$(GPP) $(CFLAGS) application_LSM303.cpp Adafruit_LSM303.cpp -o LSM303 $(LINKER)

LSM_debug:
	$(GPP) $(CFLAGS) $(DEBUG) application_LSM303.cpp Adafruit_LSM303.cpp -o LSM303 $(LINKER)

clean:
	rm *.o LSM303 *.log
