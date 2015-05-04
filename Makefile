CC = g++
CFLAGS = -g -DGL_GLEXT_PROTOTYPES -Iglut-3.7.6-bin
LDFLAGS = -lglut -lGLU -lGL -lGLEW

FILES = shader.cpp joint.cpp readfile.cpp

RM = /bin/rm -f 

all: main

main: clean main.cpp $(FILES)
	$(CC) $(CFLAGS) -o as4 main.cpp $(FILES) $(LDFLAGS) 

clean: 
	$(RM) *.o *~ as4
