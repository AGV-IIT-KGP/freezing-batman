SRC = testBot.cpp serial_lnx.cpp

TARG = testBot

CC = g++

CPPFLAGS = -g -Wall -Wno-unused-function `pkg-config --cflags opencv`  

LDFLAGS = `pkg-config --libs opencv` 

OBJ = $(SRC:.cpp=.o)

all: $(TARG)

$(TARG):
	$(CC) $(CPPFLAGS)  $(SRC) -o $(TARG) $(LDFLAGS) $<
clean:
	rm -f *~ *.o $(TARG)
