#ifndef CALLBACK_H
#define CALLBACK_H
#include "graph.h"

enum Color {white, grey, black};

int find_segments(Grid& grid, Graph& non_cuts);
int find_bad_cuts(Grid& grid, Graph& non_cuts);

class myGRBCallback: public GRBCallback
{
  public:
    Grid& grid;

    myGRBCallback(Grid& grid_) : grid(grid_) {};
  protected:
    void callback ();

};
#endif
