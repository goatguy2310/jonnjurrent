.PHONY: all clean

CXX = g++
CXXFLAGS = -O3 -flto -march=native -std=c++23 -fopenmp -Wall -MMD -MP

TARGET = build/main
SRCS = main.cpp stb/stb.cpp
OBJS = $(SRCS:%.cpp=build/%.o)

all: $(TARGET)

$(TARGET): $(OBJS)
	$(CXX) $(CXXFLAGS) -o $(TARGET) $(OBJS)

build/%.o: %.cpp
	@mkdir -p $(dir $@)
	$(CXX) $(CXXFLAGS) -c $< -o $@

-include $(OBJS:.o=.d)

clean:
	rm -rf build/ image.png
