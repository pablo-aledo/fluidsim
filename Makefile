TARGET=fluid


SRCS=$(shell ls *.cpp)
OPENMP=#-fopenmp
OBJS=$(SRCS:.cpp=.o)
CXX=g++
OPT=#-O3
DEBUG=-g
CFLAGS=-c $(DEBUG) -Wall $(OPENMP) $(OPT) -I /usr/local/include/opencv/
LIBS=$(OPENMP) -lopencv_highgui -lopencv_core -lopencv_imgproc

TARGET:$(OBJS)
	$(CXX) $(LIBS) $(OBJS) -o $(LEVEL)$(TARGET)

$(OBJS): %.o : %.cpp
	$(CXX) $(CFLAGS) $^ -o $@

run:$(TARGET)
	./$(TARGET)

valgrind:$(TARGET)
	valgrind ./$(TARGET)

debug:$(TARGET)
	gdb ./$(TARGET)

clean:
	rm -rf *.o *~ cscope.out tags

distclean: clean
	rm $(TARGET)

distrib:

depends:
	sudo dpkg -i ../install/*.deb

install:
	sudo dpkg -i ../install/*.deb
