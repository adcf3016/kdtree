CXX = g++
CXXFLAGS = -O3 -Wall -shared -std=c++11 -fPIC
PIBINDFLAG = `python3 -m pybind11 --includes` `python3-config --includes --ldflags`
LFLAGS = -lblas
TARGET = ./lib/kdtree_mod.so kdtree_mod
SRC = ./src/kdtree_mod_pybind.cpp ./src/kdtree_mod.cpp

all: $(TARGET)
$(TARGET): $(SRC)
	$(CXX) -o $@ $(CXXFLAGS) $(PIBINDFLAG) $< $(LFLAGS)
	g++ ./src/kdtree_mod.cpp -o kdtree_mod

test: $(TARGET)
	python3 -m pytest -v -s

clean:
	rm -r -f *.o *.so __pycache__ .pytest_cache

.PHONY: all test clean
# all:
# 	g++ ./src/kdtree.cpp -o kdtree
# 	g++ ./src/kdtree_alter.cpp -o kdtree_alter
# 	g++ ./src/kdtree_traverse.cpp -o kdtree_traverse
# 	g++ ./src/kdtree_dimensionless.cpp -o kdtree_dimensionless
