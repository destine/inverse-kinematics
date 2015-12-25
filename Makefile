CC = g++

CFLAGS = -I lib/eigen3 -Wall -Wno-deprecated-declarations -std=c++0x -O2
LFLAGS = -framework GLUT -framework OpenGL \
	-L"/System/Library/Frameworks/OpenGL.framework/Libraries" \
	-lGL -lGLU -lm -lstdc++

TARGET = as4

SRCS  := $(wildcard src/*.cpp)
OBJS  := $(SRCS:.cpp=.o)

RM = /bin/rm -rf

all: $(TARGET)

$(TARGET): $(OBJS) main.cpp
	$(CC) $(CFLAGS) $(OBJS) main.cpp $(LFLAGS) -o $(TARGET)

.cpp.o:
	$(CC) $(CFLAGS) -c $< -o $@

clean: 
	$(RM) $(OBJS) $(TARGET)
