CXXFLAGS = -Wall -D__VERBOSE__

all: testImuDriver

testImuDriver: objs/testImuDriver.o objs/BNO055-BBB_driver.o
	g++ $(CXXFLAGS) -o testImuDriver objs/testImuDriver.o objs/BNO055-BBB_driver.o

objs/testImuDriver.o: testImuDriver.cpp include/BNO055-BBB_driver.h
	g++ $(CXXFLAGS) -c -I ./include -o objs/testImuDriver.o testImuDriver.cpp

objs/BNO055-BBB_driver.o: BNO055-BBB_driver.cpp include/BNO055-BBB_driver.h
	g++ $(CXXFLAGS) -c -I ./include -o objs/BNO055-BBB_driver.o BNO055-BBB_driver.cpp

clean:
	rm objs/*.o testImuDriver
