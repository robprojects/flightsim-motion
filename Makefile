LDLIBS=-lGL -lglut -lm -lGLU -lpthread
CFLAGS=-g

all: motion

motion : motion.o washout.o xplane.o geo6dof.o matrix.o actuator.o nolimits2.o


clean:
	rm -f *.o motion
