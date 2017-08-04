#include <cstdlib>
#include <cmath>
#include <random>

#define png_infopp_NULL (png_infopp)NULL
#define int_p_NULL (int*)NULL
#include <boost/gil/gil_all.hpp>
#include <boost/gil/extension/io/png_dynamic_io.hpp>
#include <boost/gil/extension/io/jpeg_dynamic_io.hpp>

#include <functional>
#include <queue>
#include <vector>
#include <utility>                   // for std::pair
#include <algorithm>                 // for std::for_each
#include <limits>
#include <numeric>
#include <string>
#include <iostream>
#include <fstream>

#include <boost/utility.hpp>                // for boost::tie
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/copy.hpp>

#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>
#include "gurobi_c++.h"

namespace bg = boost::gil;
namespace b = boost;

bg::point2<int> size;
int num_vertices;
 
struct EdgeProperties{
  //bool error = false; // tells if there is a bad cut
  enum position {hor=0, vert=1};
  typedef std::pair<int, position> Index;
  Index index;
  GRBVar var; // a gurobi variable for every edge
  bool error;
  EdgeProperties(Index index_) : index(index_) {};
};
// create a typedef for the Graph type
typedef b::adjacency_list<b::vecS, b::vecS, b::undirectedS> Graph; // watch out that we have unique edges
typedef b::adjacency_list<b::vecS, b::vecS, b::undirectedS, b::no_property, EdgeProperties> Grid;
//typedef b::property_map<Graph, b::vertex_index_t>::type IndexMap;

template <typename VertexIterator>
VertexIterator xy_to_iterator(int x, int y, VertexIterator v) {
  assert(abs(x) <= size.x && abs(y) <= size.y);
  v += x + y*size.x;
  return v;
};

int xy_to_index(int x, int y) {
  assert(abs(x) <= size.x && abs(y) <= size.y);
  return x + y*size.x;
};
bg::point2<int> index_to_xy(int index) {
  int x = index % size.x;
  int y = index / size.x;
  return bg::point2<int>(x, y);
};
//xEdges and yEdges indices functions
int x_index(int x, int y) {return x+(size.x-1)*y;}
int y_index(int x, int y) {return y+(size.y-1)*x;}

typedef bg::rgb8_view_t view_t;
typedef bg::rgb8c_view_t cview_t;
typedef bg::rgb8_pixel_t pixel_t;

typedef double scalar_t;

cview_t src;
view_t dst;
const int VDIM = bg::num_channels<pixel_t>::value; // 3 for rgb, 1 for greyscale
const scalar_t CHANNEL_DIST = (scalar_t)bg::channel_traits<bg::channel_type<pixel_t>::type>::max_value() - bg::channel_traits<bg::channel_type<pixel_t>::type>::min_value();

typedef std::array<scalar_t, VDIM> vector_t;

template <typename Vector, typename Scalar>
Vector scalar_mult(Scalar s, Vector v) {
  Vector out;
  std::transform(v.begin(), v.end(), out.begin(),
                 [s](Scalar v_i) { return s*v_i; });
  return out;
}

template <typename Vector, typename Scalar>
Vector addition(Vector v1, Vector v2) {
  Vector out;
  std::transform(v1.begin(), v1.end(), v2.begin(), out.begin(),
                 [](Scalar v1_i, Scalar v2_i) { return v1_i+v2_i; });
  return out;
}

template <typename Vector, typename Scalar>
Vector subtraction(Vector v1, Vector v2) {
  Vector out;
  std::transform(v1.begin(), v1.end(), v2.begin(), out.begin(),
                 [](Scalar v1_i, Scalar v2_i) { return v1_i-v2_i; });
  return out;
}

template <typename Vector, typename Scalar>
Scalar norm(Vector v) {
  Vector help;
  std::transform(v.begin(), v.end(), help.begin(),
                 [](Scalar v_i) { return v_i*v_i; });

  //std::reduce(out.begin(), out.end(), 0, std::plus<>());
  Scalar sum(0);
  std::for_each(help.begin(), help.end(), [&sum](Scalar u){sum += u;});
  return sqrt(sum);
}
