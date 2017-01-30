# Localization Challenge
# Makefile

INCLUDE = -I/usr/include/
LIBRARIES = LocalizationController.a -lGL -lGLU -lglut -lm -lXi -lXmu -lpthread -L/usr/lib/nvidia-367 

COMPILER = g++ 
CXXFLAGS=-g -std=c++11
COMPILERFLAGS = -O2 $(INCLUDE)


PROGRAM =	localization_test

SOURCE =	main.cpp main.h 

OBJECT =	main.o


.cc.o: $(SOURCE)
	$(COMPILER) -c $(COMPILERFLAGS) $<

all: $(PROGRAM)

$(PROGRAM): $(OBJECT) $(SOURCE)
	$(COMPILER) $(COMPILERFLAGS) -o $(PROGRAM) $(OBJECT) $(LIBRARIES)

clean:
	-rm -rf core *.o *~ .*~ $(PROGRAM)
