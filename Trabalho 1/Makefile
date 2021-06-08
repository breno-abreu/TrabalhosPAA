all: closest

closest: closest.o
	g++ -Wall closest.o closest -no-pie
	
closest.o: closest.cpp
	g++ -c closest.c
	
clean:
	rm -rf *.o closest
