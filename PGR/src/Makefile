##################################################
# nastaveni
CFLAGS_COMMON=-pipe -Wall -ggdb
CC=gcc
CXX=g++

##################################################
# nastaveni knihoven s OpenGL
ifeq ($(OS), Windows_NT)
	CFLAGS=$(CFLAGS_COMMON) -Wl,--enable-auto-import
	LIBS=-lopengl32 -lglew32 -lSDL
	RM=del
else
	USE_GLEE=1
	CFLAGS=$(CFLAGS_COMMON) `sdl-config --cflags` -I. -IGL -DUSE_GLEE
	LIBS=-L/usr/lib `sdl-config --libs` -lGL
	RM=rm -f
endif

CXXFLAGS=$(CFLAGS)

##################################################
# obecne cile

PROJEKT=Mandelbrot2D

ifdef USE_GLEE
	DEPS=pgr.o GLee.o
else
	DEPS=pgr.o
endif

.PHONY: all projekt clean

all: projekt

projekt: $(PROJEKT)

clean:
	$(RM) $(PROJEKT) $(DEPS)

##################################################
##################################################
# pravidla

Mandelbrot2D: projekt.cpp
	$(CXX) -o $@ $^ $(CFLAGS) $(LIBS)

##################################################
# zavislosti

pgr.o: pgr.cpp pgr.h

Mandelbrot2D: projekt.cpp $(DEPS)

