#ifndef GRAPH_H
#define GRAPH_H

#define png_infopp_NULL (png_infopp)NULL
#define int_p_NULL (int*)NULL

#include <boost/gil/gil_all.hpp>
#include <boost/gil/extension/io/png_dynamic_io.hpp>
#include <boost/gil/extension/io/jpeg_dynamic_io.hpp>

#include <boost/utility.hpp>                // for boost::tie
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/copy.hpp>

#include "gurobi_c++.h"

#include "math_vector.h"

// global picture size
extern boost::gil::point2<int> size;

struct SuperpixelEdgeProperties{
  GRBVar var;
  bool error;
};

struct SuperpixelVertexProperties{
  int multicut_label;
};

struct HeuristicEdgeProperties{
  double cut;
};

struct HeuristicVertexProperties{
  MathVector<scalar_t, vector3_t> value;
};

struct do_nothing
{
  template <typename VertexOrEdge1, typename VertexOrEdge2>
  void operator()(const VertexOrEdge1& , VertexOrEdge2& ) const
  {
  }
}; // for copying

// create a typedef for the Graph type
// watch out that we have unique edges
typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS> Graph; // used for finding bad Cuts
typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, SuperpixelVertexProperties, SuperpixelEdgeProperties> SuperpixelGraph;
typedef boost::adjacency_list<boost::setS, boost::vecS, boost::undirectedS, HeuristicVertexProperties, HeuristicEdgeProperties> HeuristicGraph;

//typedef boost::property_map<Graph, boost::vertex_index_t>::type IndexMap;

template <typename VertexIterator>
VertexIterator xy_to_iterator(int x, int y, VertexIterator v) {
  assert(abs(x) <= size.x && abs(y) <= size.y);
  v += x + y*size.x;
  return v;
};

inline int xy_to_index(int x, int y) {
  assert(abs(x) <= size.x && abs(y) <= size.y);
  return x + y*size.x;
};

inline boost::gil::point2<int> index_to_xy(int index) {
  int x = index % size.x;
  int y = index / size.x;
  return boost::gil::point2<int>(x, y);
};
//xEdges and yEdges indices functions
// int x_index(int x, int y) {return x+(size.x-1)*y;}
// int y_index(int x, int y) {return y+(size.y-1)*x;}


#endif
