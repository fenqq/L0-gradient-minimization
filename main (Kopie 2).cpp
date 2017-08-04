#include "main.h"

#define DEBUG

// for adding constrains lazily
// we use the gurobi callback mechanism
class myGRBCallback: public GRBCallback
{
  public:
    Grid& grid;

    myGRBCallback(Grid& grid_) : grid(grid_) {};
  protected:
    void callback () {
      try {
        if (where == GRB_CB_MIPSOL) {
          // MIP solution callback
          std::cout << "callback" << std::endl;
          int nodecnt = (int) getDoubleInfo(GRB_CB_MIPSOL_NODCNT); // node number
          double obj = getDoubleInfo(GRB_CB_MIPSOL_OBJ); // objective value
          double obj_bnd = getDoubleInfo(GRB_CB_MIPSOL_OBJBND); // current error
          int solcnt = getIntInfo(GRB_CB_MIPSOL_SOLCNT); // solution number

          // build new graph but only with edges, if there is no cutting edge in the grid (so if sol = 0)
          Graph g(b::num_vertices(grid));
          //std::map<EdgeProperties::Index, double> sol;
          //std::map<EdgeProperties::Index, double> error;
          Grid::edge_iterator ei, ei_end;
          for(b::tie(ei, ei_end) = b::edges(grid); ei != ei_end; ++ei) {
            grid[*ei].error = false; // (re)set to false
            if(getSolution(grid[*ei].var) == 0.0) {
              Graph::vertex_descriptor src = source(*ei, grid), targ = target(*ei, grid);
              add_edge(src, targ, g);
            }
          }

          // now the algorithm to find bad edges
          enum Color {white, grey, black};
          std::vector<Graph::vertex_descriptor> segments(num_vertices, -1);
          int segment_count = 0;
          std::vector<Graph::vertex_descriptor> p(num_vertices, -1);
          std::vector<Color> color(num_vertices, white);
          std::queue<Graph::vertex_descriptor> Q;
          Graph::vertex_iterator vi, vi_end;
          b::tie(vi, vi_end) = vertices(g);
          for(auto it = vi; it != vi_end; ++it) {
            Graph::vertex_descriptor s = *it;
            if(segments[s] != -1)
              continue;
            else
              segment_count++;
            p[s] = -1;
            segments[s] = segment_count;
            color[s] = grey;
            Q.push(s);
            while(!Q.empty()) {
              Graph::vertex_descriptor u = Q.front();
              Q.pop();
              // check now if there is a cutting edge between u and an neighbour
              // by checking if there is  no edge to an neighbour vertex on the grid
              {
                Grid::adjacency_iterator ai, ai_end;
                for(b::tie(ai, ai_end) = b::adjacent_vertices(u, grid); ai != ai_end; ++ai){
                  auto v = *ai;
                  Grid::edge_descriptor e = b::edge(u,v,grid).first;
                  bool bad_cut = !b::edge(u,v,g).second;
                  if(bad_cut && segments[u] == segments[v] && !grid[e].error) {
                    grid[e].error = true; // we don't want to find the same bad cut twice
                    #ifdef DEBUG
                    auto u_pos = index_to_xy(u);
                    auto v_pos = index_to_xy(v);
                    std::cout << "found bad cut: " << "(" << u_pos.x << ", " << u_pos.y << ")" << " to " <<
                    "(" << v_pos.x << ", " << v_pos.y << ")" << std::endl;
                    #endif
                    //add constraint:
                    // sum(path_from_u_to_s) + sum(path_from_v_to_s) >= edge_u_to_v
                    GRBLinExpr sum_u_to_s(0.0);
                    Graph::vertex_descriptor a1 = u;
                    Graph::vertex_descriptor a2 = p[u];
                    while(a2 != -1) {
                      Grid::edge_descriptor e_path = b::edge(a1,a2,grid).first;
                      sum_u_to_s += grid[e_path].var;
                      a1 = a2;
                      a2 = p[a1];
                    }
                    GRBLinExpr sum_v_to_s(0.0);
                    a1 = v;
                    a2 = p[v];
                    while(a2 != -1) {
                      Grid::edge_descriptor e_path = b::edge(a1,a2,grid).first;
                      sum_v_to_s += grid[e_path].var;
                      a1 = a2;
                      a2 = p[a1];
                    }
                    addLazy(sum_u_to_s + sum_v_to_s >= grid[e].var);
                  }
                }
              }
              {
                Graph::adjacency_iterator ai, ai_end;
                for(b::tie(ai, ai_end) = b::adjacent_vertices(u, g); ai != ai_end; ++ai){
                  if(color[*ai] == white){
                    color[*ai] = grey;
                    p[*ai] = u;
                    segments[*ai] = segment_count;
                    Q.push(*ai);
                  }
                }
              }
              color[u] = black;
            }
          }

          std::cout << "**** New solution at node number " << nodecnt
               << ", obj value " << obj << ", obj bnd " << obj_bnd << ", sol number " << solcnt << std::endl;
        }
      } catch (GRBException e) {
        std::cout << "Error number: " << e.getErrorCode() << std::endl;
        std::cout << e.getMessage() << std::endl;
      } catch (...) {
        std::cout << "Error during callback" << std::endl;
      }
    }
};

// interrupt handler, for exiting the omptimization early
GRBModel* model_ref = NULL;
void my_handler(int s) {
  if(model_ref != NULL)
    model_ref->terminate();
  else {
    exit(1);
  }
}

int main(int argc, char const *argv[]) {
  // set the handler
  struct sigaction sigIntHandler;
  sigIntHandler.sa_handler = (void(*)(int))my_handler;
  sigemptyset(&sigIntHandler.sa_mask);
  sigIntHandler.sa_flags = 0;
  sigaction(SIGINT, &sigIntHandler, NULL);

  // get source image
  bg::rgb8_image_t src_img;
  std::cout << "get image..." << std::endl;
  //bg::jpeg_read_image("elephant.jpg", src_img);
  //bg::png_read_image("easy_test.png",src_img);
  bg::png_read_image("simple.png", src_img);
  //bg::jpeg_read_image("baum-mit-kugeln.jpg",src_img);
  bg::rgb8_image_t dst_img(src_img); // copy
  src = const_view(src_img);
  dst = view(dst_img);

  size.x = src.width();
  size.y = src.height();
  num_vertices = size.x*size.y;

  //convert from pixel type:
  std::vector<vector_t> pixels(num_vertices);
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
    model.set(GRB_IntParam_OutputFlag, 0);
    //model.set(GRB_DoubleParam_Heuristics, 0.0); // probably not necessary
    model.set(GRB_IntParam_LazyConstraints, 1);

    Grid grid(num_vertices);

    // options
    double lambda;
    double* lambda_row = new double[size.y];
    double* lambda_col = new double[size.x];
    int* k_row = new int[size.y];
    int* k_col = new int[size.x];
    // the lower lambda is the more cuts the more bad cuts the more lazy cuts we have to put in
    lambda = std::stod(argv[1]);  // 0.2 should seperate for example (0,0,0) and (52(>51=255*0.2),0,0) ... for CHANNEL_DIST = 255 and VDIM = 3
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

    //show segments
    std::cout << "building image..."  << std::endl;
    vector_t black;
    black.fill(0);

    for(int x = 0; x < (size.x-1); ++x) {
      for(int y = 0; y < size.y; ++y) {
        if(grid[b::edge(xy_to_index(x, y), xy_to_index(x+1,y), grid).first].var.get(GRB_DoubleAttr_X) == 1.0) {
          for(int j = 0; j < VDIM; ++j) {
              dst(x, y)[j] = black[j]; // links
              //dst(x+1, y)[j] = black[j];
          }
        }
      }
    }
    for(int x = 0; x < size.x; ++x) {
      for(int y = 0; y < (size.y-1); ++y) {
        if(grid[b::edge(xy_to_index(x, y), xy_to_index(x,y+1), grid).first].var.get(GRB_DoubleAttr_X) == 1.0) {
          for(int j = 0; j < VDIM; ++j) {
              //dst(x, y)[j] = black[j];
              dst(x, y+1)[j] = black[j]; // unten
          }
        }
      }
    }
    bg::png_write_view("out.png", bg::const_view(dst_img));

  } catch(GRBException e) {
    std::cout << "Error code = " << e.getErrorCode() << std::endl;
    std::cout << e.getMessage() << std::endl;
  } catch(...) {
    std::cout << "Exception during optimization" << std::endl;
  }

  return 0;
}
