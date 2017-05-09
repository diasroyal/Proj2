#C compiler
CC = g++ -O3 -std=c++11

# SRC = main.cpp 
SRC = ./tracking.cpp


CFLAGS = `pkg-config --cflags opencv`
LIBS = `pkg-config --libs opencv`

EXE = tracking

release:$(SRC)
	$(CC)    $(SRC) $(LIBS) $(CFLAGS) -o $(EXE)

clean: $(SRC)
	rm -f $(EXE) $(EXE_X) $(EXE).linkinfo 