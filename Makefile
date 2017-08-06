# gurobi needs an older version of g++
GUROBI = ./gurobi702/linux64
BOOST = /usr/include/boost
STANDARD = /usr/lib/x86_64-linux-gnu/
CPP      = g++-4.9
CARGS    = -std=c++11
CPPLIB   = -lgurobi_c++ -lgurobi70 -lpng -ljpeg

################################################################################
ARGS = 0.1

multicut: main.o
	$(CPP) $(CARGS) -L$(STANDARD) -L$(GUROBI)/lib main.o $(CPPLIB) -o ./bin/multicut

main.o: main.cpp
	$(CPP) $(CARGS) -I$(BOOST) -I$(GUROBI)/include -c main.cpp main.h

run:
	./bin/multicut $(ARGS)
