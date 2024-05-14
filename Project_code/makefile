CC = gcc

LDFLAGS = -O0 -pg -g -Wcpp -Wall -lpthread -lrt

run : main.o camera_driver.o
    $(CC) $(LDFLAGS) -o run main.o camera_driver.o

main.o : main.c camera_driver.h
    $(CC) $(LDFLAGS) -c main.c

camera_driver.o : camera_driver.c camera_driver.h
    $(CC) $(LDFLAGS) -c camera_driver.c

clean :
    rm -f *.o run gmon.out
