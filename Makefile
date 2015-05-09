CC = g++
CFLAGS = -g -DGL_GLEXT_PROTOTYPES
LIBS = -Ieigen-3.2.4 -Iglm-0.9.6.3 -Iglew-1.12.0

LDFLAGS = -lglut -lGLU -lGL -L/home/cc/cs184/sp15/class/cs184-cw/Assignments/as4/glew-1.12.0/lib -lGLEW

FILES = shader.cpp joint.cpp readfile.cpp keylistener.cpp

RM = /bin/rm -f 

all: main

main: main.cpp $(FILES)
	$(CC) $(LIBS) $(CFLAGS) -o as4 main.cpp $(FILES) $(LDFLAGS) 

clean: 
	$(RM) *.o *~ Inputs/*~ Shaders/*~ as4 test

check: joint.cpp
	$(CC) $(LIBS) $(CFLAGS) -o test joint.cpp $(LDFLAGS)
