#ifndef GRAPH_H
#define GRAPH_H

#define png_infopp_NULL (png_infopp)NULL
#define int_p_NULL (int*)NULL

#include <boost/gil/gil_all.hpp>
#include <boost/gil/extension/io/png_dynamic_io.hpp>
#include <boost/gil/extension/io/jpeg_dynamic_io.hpp>

#include <boost/utility.hpp>                // for boost::tie
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/copy.hpp>

#include "gurobi_c++.h"

// global picture size
extern boost::gil::point2<int> size;

struct EdgeProperties{
  //bool error = false; // tells if there is a bad cut
  enum position {hor=0, vert=1};
  typedef std::pair<int, position> Index;
  Index index;
  GRBVar var; // a gurobi variable for every edge
  bool error;
  EdgeProperties(Index index_) : index(index_) {};
};

struct VertexProperties{
  int segment;
};

// create a typedef for the Graph type
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS> Graph; // watch out that we have unique edges
typedef boost::adjacency_list<boost::vecS, boost::vecS, boost::undirectedS, VertexProperties, EdgeProperties> Grid;
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

typedef boost::gil::rgb8_view_t view_t;
typedef boost::gil::rgb8c_view_t cview_t;
typedef boost::gil::rgb8_pixel_t pixel_t;

typedef double scalar_t;
//these are used implicitely internally linked
const int VDIM = boost::gil::num_channels<pixel_t>::value; // 3 for rgb, 1 for greyscale
const scalar_t CHANNEL_DIST = (scalar_t)boost::gil::channel_traits<boost::gil::channel_type<pixel_t>::type>::max_value() - boost::gil::channel_traits<boost::gil::channel_type<pixel_t>::type>::min_value();
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
#endif
