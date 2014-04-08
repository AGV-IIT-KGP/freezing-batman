SRC = tesLogger.cpp

TARG = tesLogger

CC = g++

CPPFLAGS = -g -w #-Wall -Wno-unused-function

LDFLAGS = 

OBJ = $(SRC:.cpp=.o)

all: $(TARG)

$(TARG):
	$(CC) $(CPPFLAGS)  $(SRC) -o $(TARG) $(LDFLAGS) $<
clean:
	rm -f *~ *.o $(TARG)
