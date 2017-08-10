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
#include "gurobi_c++.h"

#include "graph.h"

#include "callback.h"




namespace b = boost;
namespace bg = b::gil;

#define EPS 0.000001

//#define DEBUG

// global picture size
boost::gil::point2<int> size;

cview_t src;
view_t dst;

// interrupt handler, for exiting the omptimization early
GRBModel* model_ref = NULL;
void my_handler(int s) {
  if(model_ref != NULL)
    model_ref->terminate();
  else {
    exit(1);
  }
}

void error_and_bye(std::string error_string, int error_code) {
  std::cerr << error_string << std::endl;
  exit(error_code);
}

int main(int argc, char const *argv[]) {
  // set the handler
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = (void(*)(int))my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  // arguments
  double lambda = 0.3;
  double grb_heuristic = 0.0;
  std::string image_file_type = "";
  std::string image_file_name;
  std::string image_file_path(argv[argc-1]);

  std::string crop(image_file_path);
  int pos = image_file_path.find_last_of('.');
  if(pos != std::string::npos) {
    image_file_type = image_file_path.substr(pos+1);
    crop = image_file_path.substr(0, pos);
  }
  pos = image_file_path.find_last_of("/\\");
  if(pos != std::string::npos)
    image_file_name = crop.substr(pos+1);
  else
    image_file_name = crop;
  if (argc < 2) {
    //usage:
    std::cout << "multicut [--jpg, --png] [-l lambda] image_file" << std::endl;
    error_and_bye("missing image_file", -1);
  }
  int i = 1;
  while(i < argc-1) {
    std::string arg(argv[i]);
    if(arg == "-l" || arg == "--lambda") {
        if(i+1 < argc) {
          lambda = std::stod(argv[++i]);
          goto next_param;
        } else {
          error_and_bye("not enough parameters for option --lambda", -1);
        }
    }
    if(arg == "--grb_heuristic") {
        if(i+1 < argc) {
          grb_heuristic = (double)std::stoi(argv[++i]);
          goto next_param;
        } else {
          error_and_bye("not enough parameters for option --grb_heuristic", -1);
        }
    }
    if(arg == "--jpeg" || arg == "--jpg") {
      image_file_type = "jpg";
      goto next_param;
    }
    if(arg == "--png") {
      image_file_type = "png";
      goto next_param;
    }
    error_and_bye("not implemented", -1);

    next_param:
    ++i;
  }

  bg::rgb8_image_t src_img;
  std::cout << "get image..." << std::endl;
  if(image_file_type == "jpg" || image_file_type == "jpeg")
    bg::jpeg_read_image(image_file_path, src_img);
  else if(image_file_type == "png")
    bg::png_read_image(image_file_path, src_img);
  else
    error_and_bye("unsupported image file type", -2);
  bg::rgb8_image_t dst_img(src_img); // copy
  src = const_view(src_img);

  dst = view(dst_img);// global picture size

  size.x = src.width();
  size.y = src.height();

  Grid grid(size.x*size.y);

  //convert from pixel type:
  std::vector<vector_t> pixels(b::num_vertices(grid));
  for(int y = 0; y < size.y; ++y) {
    for (int x = 0; x < size.x; ++x) {
      int nc = VDIM;
      for (int i = 0; i < nc; ++i) {
        pixels[xy_to_index(x, y)][i] = (scalar_t)src(x,y)[i];
      }
    }
  }

  try {
    std::cout << "Add variables, set objective..." << std::endl;
    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);

    // Turn off display and heuristics and enable adding constraints in our callback function
    model.set(GRB_IntParam_OutputFlag, 0); // shut up
    model.set(GRB_DoubleParam_Heuristics, grb_heuristic);
    model.set(GRB_IntParam_LazyConstraints, 1);


    // options
    //double lambda;
    double* lambda_row = new double[size.y];
    double* lambda_col = new double[size.x];
    int* k_row = new int[size.y];
    int* k_col = new int[size.x];
    // the lower lambda is the more cuts the more bad cuts the more lazy cuts we have to put in
    //lambda = std::stod(argv[1]);  // 0.2 should seperate for example (0,0,0) and (52(>51=255*0.2),0,0) ... for CHANNEL_DIST = 255 and VDIM = 3
    lambda *= CHANNEL_DIST*sqrt(VDIM); // ... since |vec| ranges from 0 to |(CHANNEL_DIST, ... , CHANNEL_DIST)[[VDIM times]]| = sqrt(VDIM)*CHANNEL_DIST
    std::fill(lambda_row, lambda_row+size.y, 1.0);
    std::fill(lambda_col, lambda_col+size.x, 1.0);
    std::fill(k_row, k_row+size.y, std::numeric_limits<int>::max());
    std::fill(k_col, k_col+size.x, std::numeric_limits<int>::max());
    //std::fill(k_row, k_row+size.y, 5);
    //std::fill(k_col, k_col+size.x, 5);
    for(int i = 0; i < size.y; ++i) {
      //lambda_row[i] = (1/sqrt(VDIM))*0.2*(i+10)/10; //
      //k_row[i] = i+1;
    }
    for(int i = 0; i < size.x; ++i) {
      //lambda_col[i] = (1/sqrt(VDIM))*0.2*(i+10)/10; //
      //k_col[i] = i+1;
    }

    GRBLinExpr obj1(0.0); // left side of term
    GRBLinExpr obj2(0.0); // right side of term
    GRBLinExpr objective(0.0);
    for(int y = 0; y < size.y; ++y) {
      for (int x = 0; x < size.x; ++x) {
        // not needed
        #if 0
        if (y != 0)
          if(rng() % 5 == 0)
            add_edge(xy_to_index(x,y), xy_to_index(x,y-1), g);
        if (x != 0)
          if(rng() % 5 == 0)
            add_edge(xy_to_index(x,y), xy_to_index(x-1,y), g);
        #endif
        if (y != size.y-1) {
          // add_edge(..) and edge(..) returns an std::pair, with the second entry beeing a bool telling if the edge exists,
          // and the first entry beeing the actual edge
          Grid::edge_descriptor e = add_edge(xy_to_index(x,y), xy_to_index(x,y+1), EdgeProperties(EdgeProperties::Index(xy_to_index(x,y), EdgeProperties::hor)), grid).first;
          //grid[e].error = false;
          grid[e].var = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "edge");
          obj2 += lambda_row[y]*grid[e].var;
          obj1 += norm<vector_t, scalar_t>(subtraction<vector_t, scalar_t>(pixels[xy_to_index(x,y)], pixels[xy_to_index(x,y+1)]))*grid[e].var;
        }
        if (x != size.x-1) {
          Grid::edge_descriptor e = add_edge(xy_to_index(x+1,y), xy_to_index(x,y), EdgeProperties(EdgeProperties::Index(xy_to_index(x,y), EdgeProperties::vert)), grid).first;
          //grid[e].error = false;
          grid[e].var = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "edge");
          obj2 += lambda_col[x]*grid[e].var;
          obj1 += norm<vector_t, scalar_t>(subtraction<vector_t, scalar_t>(pixels[xy_to_index(x,y)], pixels[xy_to_index(x+1,y)]))*grid[e].var;

        }
      }
    }
    obj2 *= lambda;
    objective = obj1 - obj2;
    model.setObjective(objective, GRB_MAXIMIZE);

    std::cout << "adding initial constraints..." << std::endl;
    // set optional constraints on number of egdes
    for(int y = 0; y < size.y; ++y) {
      GRBLinExpr left(0.0);
      for(int x = 0; x < (size.x-1); ++x) {
        //std::cout << b::edge(xy_to_index(x, y), xy_to_index(x+1,y), grid).second << std::endl;
        left += grid[b::edge(xy_to_index(x, y), xy_to_index(x+1,y), grid).first].var;
      }
      model.addConstr(left <= k_row[y]);
    }

    for(int x = 0; x < size.x; ++x) {
      GRBLinExpr left(0.0);
      for(int y = 0; y < (size.y-1); ++y) {
        left += grid[b::edge(xy_to_index(x, y), xy_to_index(x,y+1), grid).first].var;
      }
      model.addConstr(left <= k_col[x]);
    }

    // THESE CONSTRAINTS ARE NOT ENOUGH
    // only small squares cycles are considered in the beginning
    for(int x = 0; x < size.x-1; ++x) {
      for(int y = 0; y < size.y-1; ++y) {
        //a---b
        //|   |
        //d---c
        GRBVar& a_b = grid[b::edge(xy_to_index(x, y), xy_to_index(x+1,y), grid).first].var;
        GRBVar& b_c = grid[b::edge(xy_to_index(x+1, y), xy_to_index(x+1,y+1), grid).first].var;
        GRBVar& c_d = grid[b::edge(xy_to_index(x, y+1), xy_to_index(x+1,y+1), grid).first].var;
        GRBVar& d_a = grid[b::edge(xy_to_index(x, y+1), xy_to_index(x,y), grid).first].var;
        model.addConstr(a_b + b_c + c_d >= d_a);
        model.addConstr(b_c + c_d + d_a >= a_b);
        model.addConstr(c_d + d_a + a_b >= b_c);
        model.addConstr(d_a + a_b + b_c >= c_d);
      }
    }

    // set callback
    myGRBCallback cb = myGRBCallback(grid);
    model.setCallback(&cb);

    // Optimize model
    std::cout << "optimizing..." << std::endl;
    //model.optimize();
    model.optimizeasync();
    model_ref = &model; // for sig int handler
    bool wait = true;
    while( wait ) {
      wait = model.get(GRB_IntAttr_Status) == GRB_INPROGRESS;
      sleep(1);
    }
    model_ref = NULL;

    std::cout << "solution: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;

    // check if solution is correct
    // build new graph but only with edges, if there is no cutting edge in the grid (so if sol = 0)
    Graph non_cuts(b::num_vertices(grid));
    Grid::edge_iterator ei, ei_end;
    for(b::tie(ei, ei_end) = b::edges(grid); ei != ei_end; ++ei) {
      grid[*ei].error = false; // set to false
      if(std::abs(grid[*ei].var.get(GRB_DoubleAttr_X) - 0.0) < EPS) {
        Graph::vertex_descriptor src = source(*ei, grid), targ = target(*ei, grid);
        add_edge(src, targ, non_cuts);
      }
    }

    int num_segments = find_segments(grid, non_cuts);
    int num_bad_cuts = find_bad_cuts(grid, non_cuts);
    std::cout << "Calculated " << num_segments << " segments." << std::endl;

    if(num_bad_cuts > 0) {
      std::cout << "error: found bad cuts in result" << std::endl;
      for(b::tie(ei, ei_end) = b::edges(grid); ei != ei_end; ++ei) {
        if(grid[*ei].error) {
          std::cout << "bad cut!: " << "edge at vertex: " << index_to_xy(grid[*ei].index.first).x << ", " << index_to_xy(grid[*ei].index.first).y << " position: " << grid[*ei].index.second << std::endl;
        }
      }
    }
    std::cout << "building image..."  << std::endl;
    vector_t black;
    black.fill(0);

    for(int x = 0; x < (size.x-1); ++x) {
      for(int y = 0; y < size.y; ++y) {
        if(std::abs(grid[b::edge(xy_to_index(x, y), xy_to_index(x+1,y), grid).first].var.get(GRB_DoubleAttr_X) - 1.0) < EPS) {
          for(int j = 0; j < VDIM; ++j) {
              dst(x, y)[j] = black[j]; // links
              //dst(x+1, y)[j] = black[j];
          }
        }
      }
    }
    for(int x = 0; x < size.x; ++x) {
      for(int y = 0; y < (size.y-1); ++y) {
        if(std::abs(grid[b::edge(xy_to_index(x, y), xy_to_index(x,y+1), grid).first].var.get(GRB_DoubleAttr_X) - 1.0) < EPS) {
          for(int j = 0; j < VDIM; ++j) {
              //dst(x, y)[j] = black[j];
              dst(x, y+1)[j] = black[j]; // unten
          }
        }
      }
    }

    bg::png_write_view(image_file_name+"_contours"+"."+image_file_type, bg::const_view(dst_img));

    //segments out
    std::ofstream segment_file;
    segment_file.open (image_file_name+"_segments.txt");
    int width = 2;
    Grid::vertex_iterator vi, vi_end;
    b::tie(vi, vi_end) = b::vertices(grid);
    for(auto it = vi; it != vi_end; ++it) {
      int seg = grid[*it].segment;
      int width_;
      if(seg != 0 && (width_ = floor(log10(seg)) + 2) > width)
        width = (int)width_;
    }

    for(int y = 0; y < size.y; ++y) {
      for (int x = 0; x < size.x-1; ++x) {
        Grid::edge_descriptor e = b::edge(xy_to_index(x+1,y), xy_to_index(x,y), grid).first;
        if(std::abs(grid[e].var.get(GRB_DoubleAttr_X) - 1.0) < EPS)
          segment_file << std::setw(width) << std::left << std::setfill(static_cast<char>('-')) << grid[xy_to_index(x,y)].segment;
        else
          segment_file << std::setw(width) << std::left << std::setfill(static_cast<char>(' ')) << grid[xy_to_index(x,y)].segment;
      }
      segment_file << std::setw(width) << std::left << std::setfill(static_cast<char>(' ')) << grid[xy_to_index(size.x-1, y)].segment;
      segment_file << std::endl;
      if (y != size.y-1) {
        for (int x = 0; x < size.x; ++x) {
          Grid::edge_descriptor e = b::edge(xy_to_index(x,y), xy_to_index(x,y+1), grid).first;
          if(std::abs(grid[e].var.get(GRB_DoubleAttr_X) - 1.0) < EPS)
            segment_file << std::setw(width) << std::left << std::setfill(static_cast<char>(' ')) << '|';
          else
            segment_file << std::setw(width) << std::left << std::setfill(static_cast<char>(' ')) << ' ';
        }
      }
      segment_file << std::endl;
    }

  } catch(GRBException e) {
    std::cout << "Error code = " << e.getErrorCode() << std::endl;
    std::cout << e.getMessage() << std::endl;
  } catch(...) {
    std::cout << "Exception during optimization" << std::endl;
  }

  return 0;
}
#undef DEBUG
