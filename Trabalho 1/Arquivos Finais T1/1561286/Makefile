all: closest

closest: closest.o
	g++ -Wall closest.o -o closest
	
closest.o: closest.cpp
	g++ -c closest.cpp
	
clean:
	rm -rf *.o closest
