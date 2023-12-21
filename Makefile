
.PHONY = all
all:
	g++ ./src/kdtree.cpp -o kdtree
	g++ ./src/kdtree2.cpp -o kdtree2
	g++ ./src/kdtree_alter.cpp -o kdtree_alter
	g++ ./src/kdtree_traverse.cpp -o kdtree_traverse
	g++ ./src/kdtree_dimensionless.cpp -o kdtree_dimensionless
