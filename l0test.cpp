#include <cstdlib>
#include <cmath>
#include <random>
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
#include <signal.h>
#include <stdlib.h>
#include <stdio.h>
#include <unistd.h>

#include "math_vector.h"
#include "image.h"
#include "graph.h"

#include "l0_gradient_minimization.h"

namespace b = boost;
namespace bg = b::gil;

boost::gil::point2<int> size;

cview_t src;
view_t dst;

int main(int argc, char const *argv[]) {
  bg::rgb8_image_t src_img;
  std::cout << "get image..." << std::endl;
  bg::png_read_image("pics/modern_art.png", src_img);
  src = const_view(src_img);

  size.x = src.width();
  size.y = src.height();

  bg::rgb8_image_t dst_img(size.x, size.y);
  dst = view(dst_img);// global picture size

  //convert from pixel type:
  std::vector<vector3_t> pixels(size.x*size.y);

  SuperpixelGraph graph(size.x*size.y);

  for(int y = 0; y < size.y; ++y) {
    for (int x = 0; x < size.x; ++x) {
      int nc = 3;
      vector3_t v;
      for(int j = 0; j < nc; ++j)
        v[j] = (scalar_t)src(x,y)[j];
      graph[xy_to_index(x,y)].value = MathVector<scalar_t, vector3_t>(v);
    }
  }
  for(int y = 0; y < size.y; ++y) {
    for (int x = 0; x < size.x; ++x) {
      if(x != size.x-1) {
         b::add_edge(xy_to_index(x, y), xy_to_index(x+1, y), graph);
      }
      if(y != size.y-1) {
         b::add_edge(xy_to_index(x, y), xy_to_index(x, y+1), graph);
      }
    }
  }
  int num_segments = l0_gradient_minimization<SuperpixelGraph, MathVector<scalar_t, vector3_t>>(graph, 1000);
  std::cout << "number of segments: " << num_segments << std::endl;

  for(int y = 0; y < size.y; ++y) {
    for (int x = 0; x < size.x; ++x) {
      int nc = 3;
      for(int j = 0; j < nc; ++j) {
        dst(x, y)[j] = graph[(SuperpixelGraph::vertex_descriptor)xy_to_index(x, y)].value[j];
      }
    }
  }

  bg::png_write_view("out.png", bg::const_view(dst_img));
  return 0;
}
