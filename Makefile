# gurobi needs an older version of g++
GUROBI   = ./gurobi702/linux64
BOOST    = /usr/include/boost
SLIC     = ./SLIC
STANDARD = /usr/lib/x86_64-linux-gnu/
CPP      = g++-4.9
CARGS    = -std=c++11
CPPLIB   = -lgurobi_c++ -lgurobi70 -lpng -ljpeg

################################################################################
ARGS =  -l 0.3 --grb_heuristic 0 "./pics/mango.jpg"

multicut: main.o callback.o
	$(CPP) $(CARGS) -L$(STANDARD) -L$(GUROBI)/lib main.o callback.o $(CPPLIB) -o ./bin/multicut

main.o: main.cpp graph.h
	$(CPP) $(CARGS) -I$(BOOST) -I$(GUROBI)/include -c main.cpp

callback.o: callback.cpp callback.h graph.h
	$(CPP) $(CARGS) -I$(BOOST) -I$(GUROBI)/include -c callback.cpp

run:
	./bin/multicut $(ARGS)
