#ifndef CALLBACK_H
#define CALLBACK_H
#include "graph.h"

enum Color {white, grey, black};

int find_segments(SuperpixelGraph& graph, Graph& non_cuts);
int find_bad_cuts(SuperpixelGraph& graph, Graph& non_cuts);

class myGRBCallback: public GRBCallback
{
  public:
    SuperpixelGraph& graph;

    myGRBCallback(SuperpixelGraph& graph_) : graph(graph_) {};
  protected:
    void callback ();

};
#endif
