CXXFLAGS = -std=c++11 -Wall -O2 -D__VERBOSE__
#CXXFLAGS = -std=c++11 -g -Wall -D__VERBOSE__

all: ImuDriverTest ImuThreadedTest

ImuDriverTest: objs/ImuDriverTest.o objs/BNO055-BBB_driver.o
	g++ $(CXXFLAGS) -o ImuDriverTest objs/ImuDriverTest.o objs/BNO055-BBB_driver.o -lncurses

ImuThreadedTest: objs/ImuThreadedTest.o objs/BNO055-BBB_driver.o
	g++ $(CXXFLAGS) -o ImuThreadedTest objs/ImuThreadedTest.o objs/BNO055-BBB_driver.o -lncurses -lpthread

objs/ImuDriverTest.o: ImuDriverTest.cpp include/BNO055-BBB_driver.h
	g++ $(CXXFLAGS) -c -I ./include -o objs/ImuDriverTest.o ImuDriverTest.cpp

objs/ImuThreadedTest.o: ImuThreadedTest.cpp include/BNO055-BBB_driver.h
	g++ $(CXXFLAGS) -c -I ./include -o objs/ImuThreadedTest.o ImuThreadedTest.cpp

objs/BNO055-BBB_driver.o: BNO055-BBB_driver.cpp include/BNO055-BBB_driver.h
	g++ $(CXXFLAGS) -c -I ./include -o objs/BNO055-BBB_driver.o BNO055-BBB_driver.cpp

clean:
	rm objs/*.o ImuDriverTest ImuThreadedTest
