# gurobi needs an older version of g++
GUROBI   = ./gurobi702/linux64
BOOST    = /usr/include/boost
SLIC     = ./SLIC
STANDARD = /usr/lib/x86_64-linux-gnu/
CPP      = g++-4.9
CPP2     = g++
CARGS    = -std=c++11
CPPLIB   = -lgurobi_c++ -lgurobi70 -lpng -ljpeg

################################################################################
ARGS =  -l 0.06 -s 500 -m 10 --grb-heuristic 1 --draw-superpixels 0 --disallow-one-pixel-segments 1 "./pics/big_elephant.jpg"

multicut: main.o callback.o SLIC/SLIC.o
	$(CPP) $(CARGS) -L$(STANDARD) -L$(GUROBI)/lib main.o callback.o SLIC/SLIC.o $(CPPLIB) -o ./bin/multicut

main.o: main.cpp graph.h image.h math_vector.h SLIC/SLIC.h
	$(CPP) $(CARGS) -I$(BOOST) -I$(GUROBI)/include -c main.cpp

SLIC.o: SLIC/SLIC.cpp SLIC/SLIC.h
	$(CPP) $(CARGS) -I$(BOOST) -I$(SLIC) -o SLIC/SLIC.o -c SLIC/SLIC.cpp

callback.o: callback.cpp callback.h graph.h
	$(CPP) $(CARGS) -I$(BOOST) -I$(GUROBI)/include -c callback.cpp

run:
	./bin/multicut $(ARGS)

slictest: slictest.cpp graph.h
	$(CPP) $(CARGS) -L$(STANDARD) -L$(GUROBI)/lib -I$(BOOST) -I$(SLIC) -I$(GUROBI)/include slictest.cpp SLIC/SLIC.cpp $(CPPLIB) -o bin/slic_test

l0test: l0test.cpp graph.h image.h math_vector.h l0_gradient_minimization.h
	$(CPP) $(CARGS) -L$(STANDARD) -L$(GUROBI)/lib -I$(BOOST) -I$(GUROBI)/include l0test.cpp $(CPPLIB) -o bin/l0_test
