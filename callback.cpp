#include "callback.h"
#include <queue>

namespace b = boost;

#define EPS 0.000001

// for adding constrains lazily
// we use the gurobi callback mechanism
void myGRBCallback::callback() {
  try {
    if (where == GRB_CB_MIPSOL) {
      // MIP solution callback
      std::cout << "callback" << std::endl;
      int nodecnt = (int) getDoubleInfo(GRB_CB_MIPSOL_NODCNT); // node number
      double obj = getDoubleInfo(GRB_CB_MIPSOL_OBJ); // objective value
      double obj_bnd = getDoubleInfo(GRB_CB_MIPSOL_OBJBND); // current error
      int solcnt = getIntInfo(GRB_CB_MIPSOL_SOLCNT); // solution number

      // build new graph but only with edges, if there is no cutting edge in the grid (so if sol = 0)
      Graph non_cuts(b::num_vertices(grid));
      Grid::edge_iterator ei, ei_end;
      for(b::tie(ei, ei_end) = b::edges(grid); ei != ei_end; ++ei) {
        grid[*ei].error = false; // (re)set to false
        // if(getSolution(grid[*ei].var)!=0.0 && getSolution(grid[*ei].var)!=1.0)
        //   std::cout << getSolution(grid[*ei].var) << std::endl;
        if(std::abs(getSolution(grid[*ei].var) - 0.0) < EPS) {
          Graph::vertex_descriptor src = source(*ei, grid), targ = target(*ei, grid);
          add_edge(src, targ, non_cuts);
        }
      }
//slic
      find_segments(grid, non_cuts);
      find_bad_cuts(grid, non_cuts);

      for(b::tie(ei, ei_end) = b::edges(grid); ei != ei_end; ++ei) {
        if(!grid[*ei].error)
          continue;
        Grid::vertex_descriptor u = source(*ei, grid), v = target(*ei, grid);

        // find shortest path from u to v
        std::vector<Graph::vertex_descriptor> p(b::num_vertices(grid), -1);
        std::vector<Color> color(b::num_vertices(grid), white);
        std::queue<Graph::vertex_descriptor> Q;
        p[u] = -1;
        color[u] = grey;
        Q.push(u);
        while(!Q.empty()) {
          Graph::vertex_descriptor u = Q.front();
          Q.pop();
          Graph::adjacency_iterator ai, ai_end;
          for(b::tie(ai, ai_end) = b::adjacent_vertices(u, non_cuts); ai != ai_end; ++ai){
            if(color[*ai] == white){
              color[*ai] = grey;
              p[*ai] = u;
              if(*ai == v) {
                goto found_it;
              }
              Q.push(*ai);
            }
          }
          color[u] = black;
        }
        found_it:
        GRBLinExpr sum_u_to_v(0.0);
        Graph::vertex_descriptor a1 = v;
        Graph::vertex_descriptor a2 = p[v];
        while(a2 != -1) {
          Grid::edge_descriptor e_path = b::edge(a1,a2,grid).first;
          sum_u_to_v += grid[e_path].var;
          a1 = a2;
          a2 = p[a1];
        }
        addLazy(sum_u_to_v >= grid[*ei].var);
      }
      std::cout << "New solution at node number " << nodecnt
           << ", obj value " << obj << ", obj bnd " << obj_bnd << ", sol number " << solcnt << std::endl;
    }
  } catch (GRBException e) {
    std::cout << "Error number: " << e.getErrorCode() << std::endl;
    std::cout << e.getMessage() << std::endl;
  } catch (...) {
    std::cout << "Error during callback" << std::endl;
  }
}


int find_segments(Grid& grid, Graph& non_cuts) {
  std::vector<Graph::vertex_descriptor> p(b::num_vertices(grid), -1);
  std::vector<Color> color(b::num_vertices(grid), white);
  std::queue<Graph::vertex_descriptor> Q;
  int segment_count = 0;
  {
    Grid::vertex_iterator vi, vi_end;
    b::tie(vi, vi_end) = b::vertices(grid);
    for(auto it = vi; it != vi_end; ++it) {
      grid[*it].segment = -1;
    }
  }
  Graph::vertex_iterator vi, vi_end;
  b::tie(vi, vi_end) = b::vertices(non_cuts);
  for(auto it = vi; it != vi_end; ++it) {
    Graph::vertex_descriptor s = *it;
    if(grid[s].segment != -1)
      continue;

    segment_count++;
    p[s] = -1;
    grid[s].segment = segment_count;
    color[s] = grey;
    Q.push(s);
    while(!Q.empty()) {
      Graph::vertex_descriptor u = Q.front();
      Q.pop();
        Graph::adjacency_iterator ai, ai_end;
        for(b::tie(ai, ai_end) = b::adjacent_vertices(u, non_cuts); ai != ai_end; ++ai){
          if(color[*ai] == white){
            color[*ai] = grey;
            p[*ai] = u;
            grid[*ai].segment = segment_count;
            Q.push(*ai);
          }
        }
      color[u] = black;
    }
  }

  return segment_count;
}

int find_bad_cuts(Grid& grid, Graph& non_cuts){
  int num_bad_cuts = 0;
  Grid::edge_iterator ei, ei_end;
  for(b::tie(ei, ei_end) = b::edges(grid); ei != ei_end; ++ei) {
    Graph::vertex_descriptor src = source(*ei, grid), targ = target(*ei, grid);
    bool cut = !b::edge(src, targ, non_cuts).second;
    if(cut && grid[src].segment == grid[targ].segment && !grid[*ei].error) { // cut in same segment!
      grid[*ei].error = true; // we don't want to find the same bad cut twice
      num_bad_cuts++;
      #ifdef DEBUG
      auto u_pos = index_to_xy(u);
      auto v_pos = index_to_xy(v);
      std::cout << "found bad cut: " << "(" << u_pos.x << ", " << u_pos.y << ")" << " to " <<
      "(" << v_pos.x << ", " << v_pos.y << ")" << std::endl;
      #endif
    }
  }
  return num_bad_cuts;
}

#undef DEBUG
