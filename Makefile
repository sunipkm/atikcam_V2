CXX = g++
CC = gcc

EDCFLAGS:= -std=gnu11 -O2 $(CFLAGS)
EDCXXFLAGS:= -Wall -O2 -std=c++11 -I ./ -I drivers/ -I include/ -I network/ -I clkgen/include $(CXXFLAGS) -DDEFINE_WEAK
EDLDFLAGS:= -lpthread -lcfitsio -lncursest -lm $(LDFLAGS)

ifeq ($(OS),Windows_NT)
    CCFLAGS += -D WIN32
    ifeq ($(PROCESSOR_ARCHITEW6432),AMD64)
        CCFLAGS += -D AMD64
    else
        ifeq ($(PROCESSOR_ARCHITECTURE),AMD64)
            CCFLAGS += -D AMD64
        endif
        ifeq ($(PROCESSOR_ARCHITECTURE),x86)
            CCFLAGS += -D IA32
        endif
    endif
else
    UNAME_S := $(shell uname -s)
    UNAME_P := $(shell uname -p)
	EDLDFLAGS += -lm -lpthread
    ifeq ($(UNAME_S),Linux)
        ifeq ($(UNAME_P),x86_64)
        	EDLDFLAGS += -L lib/linux/64/NoFlyCapture -latikcameras
    	endif
    	ifeq ($(filter %86,$(UNAME_P)),)
        	CCFLAGS += -D IA32
    	endif
    	ifeq ($(filter arm%,$(UNAME_P)),)
        	EDLDFLAGS += -L lib/ARM/32/NoFlyCapture -latikcameras
    	endif
    endif
    ifeq ($(UNAME_S),Darwin)
        EDLDFLAGS += -L lib/macos -latikcameras
    endif
endif

COBJS = gpiodev/gpiodev.o
CPPOBJS = $(patsubst %.cpp,%.o,$(wildcard src/*.cpp)) mcp9808/mcp9808.o
CLKGENTARGET = clkgen/libclkgen.a

all: $(COBJS) $(CPPOBJS) $(CLKGENTARGET)
	$(CXX) -o atiktest.out $(COBJS) $(CPPOBJS) $(CLKGENTARGET) $(EDLDFLAGS)

$(CLKGENTARGET):
	cd clkgen && make && cd ..

%.o: %.cpp
	$(CXX) $(EDCXXFLAGS) -o $@ -c $<

%.o: %.c
	$(CC) $(EDCFLAGS) -o $@ -c $<

.PHONY: clean

clean:
	rm -vf $(CPPOBJS)
	rm -vf *.out
	rm -vf *.jpg