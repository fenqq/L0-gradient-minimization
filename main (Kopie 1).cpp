#include "main.h"


// gurboi callback handler
class myGRBCallback: public GRBCallback
{
  public:
    GRBVar* xEdges;
    int num_xEdges;
    GRBVar* yEdges;
    int num_yEdges;

    myGRBCallback(GRBVar* xEdges_, int num_xEdges_, GRBVar* yEdges_, int num_yEdges_)
    : xEdges(xEdges_), num_xEdges(num_xEdges_), yEdges(yEdges_), num_yEdges(num_yEdges_) {}
  protected:
    void callback () {
      try {
        if (where == GRB_CB_MIPSOL) {
          // MIP solution callback
          int nodecnt = (int) getDoubleInfo(GRB_CB_MIPSOL_NODCNT); // node number
          double obj = getDoubleInfo(GRB_CB_MIPSOL_OBJ); // objective value
          int solcnt = getIntInfo(GRB_CB_MIPSOL_SOLCNT); // solution number
          double* xEdges_sol = getSolution(xEdges, num_xEdges);
          double* yEdges_sol = getSolution(yEdges, num_yEdges);
          //addLazy(xEdges[0] <= xEdges[1]);

          std::cout << "**** New solution at node number " << nodecnt
               << ", obj value " << obj << ", sol number " << solcnt << std::endl;
          delete[] xEdges_sol;
          delete[] yEdges_sol;
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
  bg::jpeg_read_image("baum-mit-kugeln.jpg",src_img);
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
    GRBEnv env = GRBEnv();
    GRBModel model = GRBModel(env);

    // Turn off display and heuristics and enable adding constraints in our callback function
    model.set(GRB_IntParam_OutputFlag, 0);
    model.set(GRB_DoubleParam_Heuristics, 0.0); // probably not necessary
    model.set(GRB_IntParam_LazyConstraints, 1);

    // indexing functions
    auto x_index = [](int x, int y) -> int{return x+(size.x-1)*y;};
    auto y_index = [](int x, int y) -> int{return y+(size.y-1)*x;};
    // add variables
    std::cout << "adding variables..." << std::endl;
    //GRBVar* xEdges = new GRBVar[(size.x-1)*size.y]; // edges from i to i+1 x-Axis
    //GRBVar* yEdges = new GRBVar[(size.y-1)*size.x]; // edges from i to i+1 y-Axis

    Grid grid(num_vertices);

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
          Grid::edge_descriptor e = add_edge(xy_to_index(x,y), xy_to_index(x,y+1), grid).first;
          //grid[e].error = false;
          //grid[e].var
        }
        if (x != size.x-1) {
          Grid::edge_descriptor e = add_edge(xy_to_index(x,y), xy_to_index(x+1,y), grid).first;
          //grid[e].error = false;
          //grid[e].var
        }
      }
    }

    for(int i = 0; i < (size.x-1)*size.y; ++i) {
      xEdges[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "edge");
    }
    for(int i = 0; i < (size.y-1)*size.x; ++i) {
      yEdges[i] = model.addVar(0.0, 1.0, 0.0, GRB_BINARY, "edge");
    }
    //set objective
    std::cout << "setting objective..." << std::endl;

    double lambda;
    double* lambda_row = new double[size.y];
    double* lambda_col = new double[size.x];
    int* k_row = new int[size.y];
    int* k_col = new int[size.x];
    lambda = 0.2;  // 0.2 should seperate for example (0,0,0) and (52(>51=255*0.2),0,0) ... for CHANNEL_DIST = 255 and VDIM = 3
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

    GRBLinExpr objective(0.0);
    GRBLinExpr obj1(0.0); // left side of term
    GRBLinExpr obj2(0.0); // right side of term

    for(int y = 0; y < size.y; ++y) {
      for(int x = 0; x < (size.x-1); ++x) {
        obj2 += lambda_row[y]*xEdges[x_index(x, y)];
      }
    }
    for(int y = 0; y < size.y; ++y) {
      for(int x = 0; x < (size.x-1); ++x) {
        obj2 += lambda_col[x]*yEdges[y_index(x, y)];
      }
    }

    obj2 *= lambda;

    for(int y = 0; y < size.y; ++y) {
      for(int x = 0; x < (size.x-1); ++x) {
        obj1 += norm<vector_t, scalar_t>(subtraction<vector_t, scalar_t>(pixels[xy_to_index(x,y)], pixels[xy_to_index(x+1,y)]))*xEdges[x_index(x, y)];
      }
    }
    for(int x = 0; x < size.x; ++x) {
      for(int y = 0; y < (size.y-1); ++y) {
        obj1 += norm<vector_t, scalar_t>(subtraction<vector_t, scalar_t>(pixels[xy_to_index(x,y)], pixels[xy_to_index(x,y+1)]))*yEdges[y_index(x, y)];
      }
    }
    objective = obj1 - obj2;
    model.setObjective(objective, GRB_MAXIMIZE);

    // Add constraints
    // THESE CONSTRAINTS ARE NOT ENOUGH
    // only small squares cycles are considered in the beginning
    std::cout << "adding constraints..." << std::endl;
    for(int x = 0; x < size.x-1; ++x) {
      for(int y = 0; y < size.y-1; ++y) {
          //the square: xEdges[x_index(x, y)] + xEdges[x_index(x, y+1)] + yEdges[y_index(x, y)] + yEdges[y_index(x+1, y)]
          model.addConstr(xEdges[x_index(x, y)]  +xEdges[x_index(x, y+1)]+yEdges[y_index(x, y)]   >= yEdges[y_index(x+1, y)]);
          model.addConstr(xEdges[x_index(x, y)]  +xEdges[x_index(x, y+1)]+yEdges[y_index(x+1, y)] >= yEdges[y_index(x, y)]);
          model.addConstr(xEdges[x_index(x, y)]  +yEdges[y_index(x, y)]  +yEdges[y_index(x+1, y)] >= xEdges[x_index(x, y+1)]);
          model.addConstr(xEdges[x_index(x, y+1)]+yEdges[y_index(x, y)]  +yEdges[y_index(x+1, y)] >= xEdges[x_index(x, y)]);
      }
    }

    for(int y = 0; y < size.y; ++y) {
      GRBLinExpr left(0.0);
      for(int x = 0; x < (size.x-1); ++x) {
        left += xEdges[x_index(x, y)];
      }
      model.addConstr(left <= k_row[y]);
    }
    for(int x = 0; x < size.x; ++x) {
      GRBLinExpr left(0.0);
      for(int y = 0; y < (size.y-1); ++y) {
        left += yEdges[y_index(x, y)];
      }
      model.addConstr(left <= k_col[x]);
    }
    // set callback
    myGRBCallback cb = myGRBCallback(xEdges, (size.x-1)*size.y, yEdges, (size.y-1)*size.x);
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
        if(xEdges[x_index(x, y)].get(GRB_DoubleAttr_X) == 1.0) {
          for(int j = 0; j < VDIM; ++j) {
              dst(x, y)[j] = black[j]; // links
              //dst(x+1, y)[j] = black[j];
          }
        }
      }
    }
    for(int x = 0; x < size.x; ++x) {
      for(int y = 0; y < (size.y-1); ++y) {
        if(yEdges[y_index(x, y)].get(GRB_DoubleAttr_X) == 1.0) {
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
