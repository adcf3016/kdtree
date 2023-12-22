CXX = g++
CXXFLAGS = -O3 -Wall -shared -std=c++11 -fPIC
PIBINDFLAG = `python3 -m pybind11 --includes` `python3-config --includes --ldflags`
LFLAGS = -lblas
TARGET = ./lib/kdtree_mod.so ./lib/kdtree.so ./build/kdtree_mod ./build/kdtree
SRC = ./src/kdtree_mod_pybind.cpp ./src/kdtree_mod.cpp ./src/kdtree.cpp ./src/kdtree_pybind.cpp
PYTHON = python3
DIR = ./lib/ ./build/

all: $(DIR) $(TARGET)

$(TARGET): $(SRC)
	$(CXX) -o ./lib/kdtree_mod.so $(CXXFLAGS) $(PIBINDFLAG) ./src/kdtree_mod_pybind.cpp $(LFLAGS)
	$(CXX) -o ./lib/kdtree.so $(CXXFLAGS) $(PIBINDFLAG) ./src/kdtree_pybind.cpp $(LFLAGS)
	$(CXX) -o ./build/kdtree_mod ./src/kdtree_mod.cpp
	$(CXX) -o ./build/kdtree ./src/kdtree.cpp

$(DIR):
	mkdir -p lib
	mkdir -p build

test: $(TARGET)
	cd ./test
	$(PYTHON) -m pytest -v -s
	cd ..

clean:
	rm -r -f *.o *.so __pycache__ .pytest_cache build lib

animate:
	$(PYTHON) ./animation/animation.py
	$(PYTHON) ./animation/animation2.py
	$(PYTHON) ./animation/animation3.py

.PHONY: all test clean animate
