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

#include "l0_gradient_minimization.h"
#include "graph.h"
#include "image.h"
#include "math_vector.h"
#include "callback.h"
#include "SLIC/SLIC.h"

#define EPS 0.000001
// global picture size
boost::gil::point2<int> size;

namespace b = boost;
namespace bg = b::gil;

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


  if (argc < 2) {
    //usage:
    std::cout << "usage: " << std::endl;
    std::cout << "multicut [flags] image_file" << std::endl;
    std::cout << "--jpg (--jpeg), --png (default: uses file ending): explicitly set image file type" << std::endl;
    std::cout << "--lambda (-l) [0 to 1] (default 0.3): the global lambda variable" << std::endl;
    std::cout << "--grb_heuristic [0 or 1] (default 1): set gurobi heuristic algorithm on/off" << std::endl;
    std::cout << "--superpixels (-s) [int] (default 1000): the number of superpixels used" << std::endl;
    std::cout << "--superpixel-compactness (-m) [10 - 40] (default 29): the compactness of the superpixels" << std::endl;
    std::cout << "--draw-superpixels [0 or 1] (default 0): draw superpixelpixels on image or not" << std::endl;
    std::cout << "--disallow-one-pixel-segments [0 or 1] (default 0): if set to 1 will add constraints so that no one pixel sized segments will be in the result" << std::endl;

    error_and_bye("missing image_file", -1);
  }

  // arguments
  double lambda = 0.3;
  double grb_heuristic = 1.0;
  int slic_number_of_superpixels = 1000;
  int slic_compactness = 20;
  int slic_draw_superpixels = 0;
  int disallow_one_pixel_segments = 0;
  int use_l0_heuristic = 1;

  std::string image_file_type = "";
  std::string image_file_name;
  std::string image_file_path(argv[argc-1]);
  std::string pixel_type = "rgb"; // const

  std::string crop(image_file_path);

  int pos = image_file_path.find_last_of("/\\");
  if(pos != std::string::npos)
    crop = image_file_path.substr(pos+1);
  else
    crop = image_file_path;
  pos = crop.find_last_of('.');
  if(pos != std::string::npos) {
    image_file_type = crop.substr(pos+1);
    image_file_name = crop.substr(0, pos);
  } else
    image_file_name = crop;

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
    if(arg == "--grb-heuristic") {
        if(i+1 < argc) {
          grb_heuristic = (double)std::stoi(argv[++i]);
          goto next_param;
        } else {
          error_and_bye("not enough parameters for option --grb_heuristic", -1);
        }
    }
    if(arg == "--superpixels" || arg == "-s") {
        if(i+1 < argc) {
          slic_number_of_superpixels = std::stoi(argv[++i]);
          goto next_param;
        } else {
          error_and_bye("not enough parameters for option --superpixels", -1);
        }
    }
    if(arg == "--superpixel-compactness" || arg == "-m") {
        if(i+1 < argc) {
          slic_compactness = (double)std::stod(argv[++i]);
          goto next_param;
        } else {
          error_and_bye("not enough parameters for option --superpixel-compactness", -1);
        }
    }
    if(arg == "--draw-superpixels") {
        if(i+1 < argc) {
          slic_draw_superpixels = std::stoi(argv[++i]);
          goto next_param;
        } else {
          error_and_bye("not enough parameters for option --draw-superpixels", -1);
        }
    }
    if(arg == "--disallow-one-pixel-segments") {
        if(i+1 < argc) {
          disallow_one_pixel_segments = std::stoi(argv[++i]);
          goto next_param;
        } else {
          error_and_bye("not enough parameters for option --disallow-one-pixel-segments", -1);
        }
    }
    if(arg == "--use-l0-heuristic") {
        if(i+1 < argc) {
          use_l0_heuristic = std::stoi(argv[++i]);
          goto next_param;
        } else {
          error_and_bye("not enough parameters for option --use-l0-heuristic", -1);
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

  image_t src_img;
  std::cout << "get image..." << std::endl;
  if(image_file_type == "jpg" || image_file_type == "jpeg")
    bg::jpeg_read_image(image_file_path, src_img);
  else if(image_file_type == "png")
    bg::png_read_image(image_file_path, src_img);
  else
    error_and_bye("unsupported image file type", -2);
  image_t dst_img(src_img); // copy
  src = const_view(src_img);

  dst = view(dst_img);// global picture size

  size.x = src.width();
  size.y = src.height();


  //convert from pixel type:
  std::vector<vector3_t> pixels(size.x*size.y);
  for(int y = 0; y < size.y; ++y) {
    for (int x = 0; x < size.x; ++x) {
      int nc = 3;
      for (int i = 0; i < nc; ++i) {
        pixels[xy_to_index(x, y)][i] = (scalar_t)src(x,y)[i];
      }
    }
  }

  // unsigned int (32 bits) to hold a pixel in ARGB format as follows:
  // from left to right,
  // the first 8 bits are for the alpha channel (and are ignored)
  // the next 8 bits are for the red channel
  // the next 8 bits are for the green channel
  // the last 8 bits are for the blue channel
  std::cout << "converting data" << std::endl;
  uint32_t* slic_pixel_data = new uint32_t[size.x*size.y];// 64 bit problems?
  for (int x = 0; x < size.x; ++x) {
    for (int y = 0; y < size.y; ++y) {
      uint32_t data = 0;
      uint32_t red = bg::get_color(src(x,y), bg::red_t());
      uint32_t green = bg::get_color(src(x,y), bg::green_t());
      uint32_t blue = bg::get_color(src(x,y), bg::blue_t());

      data |= 0xff;
      data <<= 8;
      data |= red;
      data <<= 8;
      data |= green;
      data <<= 8;
      data |= blue;
      slic_pixel_data[xy_to_index(x, y)] = data;
    }

  }

  std::cout << "starting slic..." << std::endl;
	slic_number_of_superpixels = std::min(slic_number_of_superpixels, size.x*size.y / 5);//Desired number of superpixels.
	//double slic_compactness; //Compactness factor. use a value ranging from 10 to 40 depending on your needs. Default is 10
	int* slic_labels = new int[size.x*size.y];
	int slic_numlabels(0);
	SLIC slic;
	slic.PerformSLICO_ForGivenK(slic_pixel_data, size.x, size.y, slic_labels, slic_numlabels, slic_number_of_superpixels, slic_compactness);



  try {
    std::cout << "Add variables, set objective..." << std::endl;
    //std::cout << slic_numlabels << " " << slic_number_of_superpixels << std::endl;
    SuperpixelGraph graph(slic_numlabels);

    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);

    // Turn off display and heuristics and enable adding constraints in our callback function
    model.set(GRB_IntParam_OutputFlag, 0); // shut up
    model.set(GRB_DoubleParam_Heuristics, grb_heuristic);
    model.set(GRB_IntParam_LazyConstraints, 1);


    // options
    //double lambda;
    // the lower lambda is the more cuts the more bad cuts the more lazy cuts we have to put in
    //lambda = std::stod(argv[1]);  // 0.2 should seperate for example (0,0,0) and (52(>51=255*0.2),0,0) ... for CHANNEL_DIST = 255 and VDIM = 3
    // lambda *= CHANNEL_DIST*sqrt(VDIM); // ... since |vec| ranges from 0 to |(CHANNEL_DIST, ... , CHANNEL_DIST)[[VDIM times]]| = sqrt(VDIM)*CHANNEL_DIST

    GRBLinExpr obj1(0.0); // left side of term
    GRBLinExpr obj2(0.0); // right side of term
    GRBLinExpr objective(0.0);

    for(int y = 0; y < size.y; ++y) {
      for (int x = 0; x < size.x; ++x) {
        if(x != size.x-1) {
          int a = slic_labels[xy_to_index(x, y)] ;
          int b = slic_labels[xy_to_index(x+1, y)];
          if(a != b) {
            auto e = b::add_edge(a, b, graph);
            if(e.second) {
              graph[e.first].var = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "edge");
            }
            obj2 += graph[e.first].var;
            MathVector<scalar_t, vector3_t> v(pixels[xy_to_index(x,y)]);
            MathVector<scalar_t, vector3_t> w(pixels[xy_to_index(x+1,y)]);
            obj1 += (v-w).norm()*graph[e.first].var;
          }
        }
        if(y != size.y-1) {
          int a = slic_labels[xy_to_index(x, y)] ;
          int b = slic_labels[xy_to_index(x, y+1)];
          if(a != b) {
            auto e = b::add_edge(a, b, graph);
            if(e.second) {
              graph[e.first].var = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "edge");
            }
            obj2 += graph[e.first].var;
            MathVector<scalar_t, vector3_t> v(pixels[xy_to_index(x,y)]);
            MathVector<scalar_t, vector3_t> w(pixels[xy_to_index(x,y+1)]);
            obj1 += (v-w).norm()*graph[e.first].var;

          }
        }
      }
    }
    obj2 *= lambda*CHANNEL_DIST*sqrt(VDIM);
    objective = obj1 - obj2;
    model.setObjective(objective, GRB_MAXIMIZE);

    // disallow 1 pixel size segments?
    if(disallow_one_pixel_segments) {
      SuperpixelGraph::vertex_iterator vi, vi_end;
      for(b::tie(vi, vi_end) = b::vertices(graph); vi != vi_end; ++vi) {
        SuperpixelGraph::out_edge_iterator ei, ei_end;
        GRBLinExpr sum_edges(0.0);
        for(b::tie(ei, ei_end) = b::out_edges(*vi, graph); ei != ei_end; ++ei) {
          sum_edges += graph[*ei].var;
        }
        model.addConstr(sum_edges <= (double)b::out_degree(*vi, graph)-1);
      }
    }
    //  std::cout << "adding initial constraints..." << std::endl;
    /*  // THESE CONSTRAINTS ARE NOT ENOUGH
      // only small squares cycles are considered in the beginning
      for(int x = 0; x < size.x-1; ++x) {
        for(int y = 0; y < size.y-1; ++y) {
          //a---b
          //|   |
          //d---c
          GRBVar& a_b = graph[b::edge(xy_to_index(x, y), xy_to_index(x+1,y), graph).first].var;
          GRBVar& b_c = graph[b::edge(xy_to_index(x+1, y), xy_to_index(x+1,y+1), graph).first].var;
          GRBVar& c_d = graph[b::edge(xy_to_index(x, y+1), xy_to_index(x+1,y+1), graph).first].var;
          GRBVar& d_a = graph[b::edge(xy_to_index(x, y+1), xy_to_index(x,y), graph).first].var;
          model.addConstr(a_b + b_c + c_d >= d_a);
          model.addConstr(b_c + c_d + d_a >= a_b);
          model.addConstr(c_d + d_a + a_b >= b_c);
          model.addConstr(d_a + a_b + b_c >= c_d);
        }
      }
    */
    // make heuristic
    if( use_l0_heuristic ) {
      std::cout << "performing l0 heuristic..." << std::endl;
      HeuristicGraph graph_copy(0);
      b::copy_graph(graph, graph_copy, b::vertex_copy(do_nothing()).edge_copy(do_nothing())); // doesn't copy vertex or edge properties
      std::vector<int> count(slic_numlabels, 0);
      for(int i = 0; i < slic_numlabels; ++i) {
        vector3_t zero = {0.0, 0.0, 0.0};
        graph_copy[(HeuristicGraph::vertex_descriptor)i].value = MathVector<scalar_t, vector3_t>(zero);
      }
      for(int x = 0; x < size.x; ++x) {
        for(int y = 0; y < size.y; ++y) {
          int a = slic_labels[xy_to_index(x,y)];
          graph_copy[(HeuristicGraph::vertex_descriptor)a].value += MathVector<scalar_t, vector3_t>(pixels[xy_to_index(x,y)]);
          count[a]++;
        }
      }
      for(int i = 0; i < slic_numlabels; ++i) {
        graph_copy[(HeuristicGraph::vertex_descriptor)i].value *= 1.0/(double)count[i];
      }
      //std::cout << b::num_vertices(graph_copy) << std::endl;

      // fusion criterion is |Y_i - Y_j|^2 / 2 <= lambda
      // so the term |Y_i-Y_j|^2 / 2 ranges from 0 to |(CHANNEL_DIST, ..., CHANNEL_DIST)[[VDIM times]]|^2 / 2
      l0_gradient_minimization<HeuristicGraph, MathVector<scalar_t, vector3_t>>(graph_copy, lambda*VDIM*CHANNEL_DIST*CHANNEL_DIST*0.5 *1);
      HeuristicGraph::vertex_iterator vi, vi_end;
      for(b::tie(vi, vi_end) = b::vertices(graph_copy); vi != vi_end; ++vi){
        HeuristicGraph::adjacency_iterator ai, ai_end;
        for(b::tie(ai, ai_end) = b::adjacent_vertices(*vi, graph_copy); ai != ai_end; ++ai) {
          //std::cout << b::edge((SuperpixelGraph::vertex_descriptor)*vi, (SuperpixelGraph::vertex_descriptor)*ai, graph).second << std::endl;
          auto he = b::edge(*vi, *ai, graph_copy).first;
          auto e = b::edge(*vi, *ai, graph).first;
          if(graph_copy[*vi].value != graph_copy[*ai].value) {
            graph_copy[he].cut = 1.0; // not necessary
            graph[e].var.set(GRB_DoubleAttr_Start, 1.0); // set mip init feasible solution
          } else {
            graph_copy[he].cut = 0.0; // not necessary
            graph[e].var.set(GRB_DoubleAttr_Start, 0.0); // set mip init feasible solution
          }
        }
      }
      //debug
      #if 1
      std::cout << "saving heuristic..." << std::endl;
      for(int x = 0; x < size.x; ++x) {
        for(int y = 0; y < size.y; ++y) {
          int a = slic_labels[xy_to_index(x,y)];
          for(int j = 0; j < 3; ++j) {
            dst(x,y)[j] = graph_copy[(HeuristicGraph::vertex_descriptor)a].value[j];
          }
        }
      }
      if(image_file_type == "jpg" || image_file_type == "jpeg")
        bg::jpeg_write_view(image_file_name+"_heu"+"."+image_file_type, bg::const_view(dst_img));
      else if(image_file_type == "png")
        bg::png_write_view(image_file_name+"_heu"+"."+image_file_type, bg::const_view(dst_img));
      #endif
    }

    // set callback
    myGRBCallback cb = myGRBCallback(graph);
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

    std::cout << "objective value: " << model.get(GRB_DoubleAttr_ObjVal) << std::endl;

    // check if solution is correct
    // build new graph but only with edges, if there is no cutting edge in the graph (so if sol = 0)
    Graph non_cuts(b::num_vertices(graph));
    SuperpixelGraph::edge_iterator ei, ei_end;
    for(b::tie(ei, ei_end) = b::edges(graph); ei != ei_end; ++ei) {
      graph[*ei].error = false; // set to false
      if(std::abs(graph[*ei].var.get(GRB_DoubleAttr_X) - 0.0) < EPS) {
        Graph::vertex_descriptor src = source(*ei, graph), targ = target(*ei, graph);
        add_edge(src, targ, non_cuts);
      }
    }

    int num_segments = find_segments(graph, non_cuts);
    int num_bad_cuts = find_bad_cuts(graph, non_cuts);
    std::cout << "Calculated " << num_segments << " segments." << std::endl;

    if(num_bad_cuts > 0) {
      std::cout << "error: found bad cuts in result" << std::endl;
      for(b::tie(ei, ei_end) = b::edges(graph); ei != ei_end; ++ei) {
        if(graph[*ei].error) {
          std::cout << "bad cut!" << std::endl;
        }
      }
    }
    std::cout << "building image..."  << std::endl;

    if(slic_draw_superpixels)
      slic.DrawContoursAroundSegments(slic_pixel_data, slic_labels, size.x, size.y, 0xff000000);

    for(int x = 0; x < size.x; ++x) {
      for(int y = 0; y < size.y; ++y) {
        if(x != size.x-1) {
          int a = slic_labels[xy_to_index(x,y)];
          int b = slic_labels[xy_to_index(x+1,y)];
          auto e = b::edge(a, b, graph);
          if(a!=b && e.second) {
            if (std::abs(graph[e.first].var.get(GRB_DoubleAttr_X) - 1.0) < EPS)
              slic_pixel_data[xy_to_index(x,y)] = 0xffff0000;
          }
        }
        if(y != size.y-1) {
          int a = slic_labels[xy_to_index(x,y)];
          int b = slic_labels[xy_to_index(x,y+1)];
          auto e = b::edge(a, b, graph);
          if(a!=b && e.second) {
            if (std::abs(graph[e.first].var.get(GRB_DoubleAttr_X) - 1.0) < EPS)
              slic_pixel_data[xy_to_index(x,y)] = 0xffff0000;
          }
        }
      }
    }

    for (int x = 0; x < size.x; ++x) {
      for (int y = 0; y < size.y; ++y) {
        uint32_t data = slic_pixel_data[xy_to_index(x, y)];

        bg::get_color(dst(x,y), bg::blue_t()) = 0xff & data;
        data >>= 8;
        bg::get_color(dst(x,y), bg::green_t()) = 0xff & data;
        data >>= 8;
        bg::get_color(dst(x,y), bg::red_t()) = 0xff & data;
        data >>= 8;
      }
    }

    if(image_file_type == "jpg" || image_file_type == "jpeg")
      bg::jpeg_write_view(image_file_name+"_contours"+"."+image_file_type, bg::const_view(dst_img));
    else if(image_file_type == "png")
      bg::png_write_view(image_file_name+"_contours"+"."+image_file_type, bg::const_view(dst_img));

    //segments out
    /*
    std::ofstream segment_file;
    segment_file.open (image_file_name+"_segments.txt");
    int width = 2;
    SuperpixelGraph::vertex_iterator vi, vi_end;
    b::tie(vi, vi_end) = b::vertices(graph);
    for(auto it = vi; it != vi_end; ++it) {
      int seg = graph[*it].segment;
      int width_;
      if(seg != 0 && (width_ = floor(log10(seg)) + 2) > width)
        width = (int)width_;
    }

    for(int y = 0; y < size.y; ++y) {
      for (int x = 0; x < size.x-1; ++x) {
        SuperpixelGraph::edge_descriptor e = b::edge(xy_to_index(x+1,y), xy_to_index(x,y), graph).first;
        if(std::abs(graph[e].var.get(GRB_DoubleAttr_X) - 1.0) < EPS)
          segment_file << std::setw(width) << std::left << std::setfill(static_cast<char>('-')) << graph[xy_to_index(x,y)].segment;
        else
          segment_file << std::setw(width) << std::left << std::setfill(static_cast<char>(' ')) << graph[xy_to_index(x,y)].segment;
      }
      segment_file << std::setw(width) << std::left << std::setfill(static_cast<char>(' ')) << graph[xy_to_index(size.x-1, y)].segment;
      segment_file << std::endl;
      if (y != size.y-1) {
        for (int x = 0; x < size.x; ++x) {
          SuperpixelGraph::edge_descriptor e = b::edge(xy_to_index(x,y), xy_to_index(x,y+1), graph).first;
          if(std::abs(graph[e].var.get(GRB_DoubleAttr_X) - 1.0) < EPS)
            segment_file << std::setw(width) << std::left << std::setfill(static_cast<char>(' ')) << '|';
          else
            segment_file << std::setw(width) << std::left << std::setfill(static_cast<char>(' ')) << ' ';
        }
      }
      segment_file << std::endl;
    }*/

  } catch(GRBException e) {
    std::cout << "Error code = " << e.getErrorCode() << std::endl;
    std::cout << e.getMessage() << std::endl;
  } catch(...) {
    std::cout << "Exception during optimization" << std::endl;
  }

  return 0;
}
#undef DEBUG
