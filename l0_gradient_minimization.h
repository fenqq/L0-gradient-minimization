#ifndef L0_GRADIENT_MINIMIZATION_H
#define L0_GRADIENT_MINIMIZATION_H

//#include "graph.h"
#include <vector>
#include <set>
#include <map>
#include <cmath>

#include <boost/utility.hpp>                // for boost::tie
#include <boost/graph/graph_traits.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/graphviz.hpp>
#include <boost/graph/copy.hpp>


inline double non_linear_beta(int iter, int iteration_number, double lambda) {
  double gamma = 2.2;
  return pow((double)iter/(double) iteration_number, gamma)*lambda;
}

// performs l0 gradient minimization as described in
// Fast and Effective L0 Gradient Minimization by Region Fusion - by
// Rang M. H. Nguyen
// Michael S. Brownon
// for a general Graph, which has to have 'vecS'  as vertices type ande a vertex property named 'value' of type Vector
// returns number of groups
template <typename Graph, typename Vector>
int l0_gradient_minimization(Graph& graph, double lambda) {
  struct group {
    std::vector<int> elements;
    std::set<int> neighbours;
    std::map<int, int> number_of_connections;
    int number_of_elements;
    Vector average_value;
    group() : elements(1) {}
  };

  std::vector<group> groups(boost::num_vertices(graph));
  std::vector<bool> skipover(boost::num_vertices(graph), false);

  for(typename Graph::vertex_descriptor i = 0; i < boost::num_vertices(graph); ++i) {
    groups[i].average_value = graph[i].value;
    groups[i].elements[0] = i;
    groups[i].number_of_elements = 1;
    typename Graph::adjacency_iterator ai, ai_end;
    for(boost::tie(ai, ai_end) = boost::adjacent_vertices(i, graph); ai != ai_end; ++ai){
      groups[i].neighbours.insert(*ai);
      groups[i].number_of_connections[*ai] = 1;
    }
  }

  double beta = 0.0;
  int beta_iter = 0;
  int iteration_number = 100;
  do {
    //std::cout << "beta = " << beta << std::endl;
    //for(auto i = 0; i < groups.size(); ++i) {
    for(int i = 0; i < boost::num_vertices(graph); ++i) {
      if(skipover[i])
        continue;
      auto neighbour_it = groups[i].neighbours.begin();
      while(neighbour_it != groups[i].neighbours.end()){
        const int j = *neighbour_it; // no ref or else it wont work
        int& c_i_j = groups[i].number_of_connections[j];
        int& w_i = groups[i].number_of_elements;
        int& w_j = groups[j].number_of_elements;
        std::set<int>& N_i = groups[i].neighbours;
        std::set<int>& N_j = groups[j].neighbours;
        std::vector<int>& G_i = groups[i].elements;
        std::vector<int>& G_j = groups[j].elements;
        Vector& Y_i = groups[i].average_value;
        Vector& Y_j = groups[j].average_value;

        typename Vector::Scalar merge_left_hand_term = w_i*w_j*(Y_i-Y_j).norm()*(Y_i-Y_j).norm();
        typename Vector::Scalar merge_right_hand_term = beta*c_i_j*(w_i+w_j);

        double ERR = 0.0;
        if(merge_left_hand_term <= merge_right_hand_term + ERR) {
          std::vector<int> grp;
          std::set_union(G_i.begin(), G_i.end(),
                   G_j.begin(), G_j.end(),
                   std::back_inserter(grp));
          G_i = grp;
          Y_i = (1/(double)(w_i+w_j))*(w_i*Y_i + w_j*Y_j);
          w_i = w_i + w_j;
          neighbour_it = N_i.erase(neighbour_it);//N_i.erase(j);
          for(auto neighbour_neighbour_it = N_j.begin(); neighbour_neighbour_it != N_j.end(); ++neighbour_neighbour_it) {
            const int k = *neighbour_neighbour_it;
            if(k == i)
              continue;
            std::set<int>& N_k = groups[k].neighbours;
            int& c_j_k = groups[j].number_of_connections[k];
            int& c_i_k = groups[i].number_of_connections[k];
            int& c_k_i = groups[k].number_of_connections[i];
            //auto it = N_i.find(k);
            if(N_i.find(k) != N_i.end()) {
              c_i_k += c_j_k;
              //c_k_i += c_j_k
              c_k_i = c_i_k; // wrong in text?
            } else {
              N_i.insert(k); // problem since we iterate over it?
              N_k.insert(i);
              c_i_k = c_j_k;
              c_k_i = c_j_k;
            }
            N_k.erase(j);
            groups[k].number_of_connections.erase(j);
          }
          skipover[j] = true;
          //groups.erase(groups.begin()+j); // delete j'th group PROBLEMOOO dadurch ändert sich der index
        } else {
          ++neighbour_it;
        }
      }
    }
    if(lambda == 0.0)
      break;
    beta_iter++;
    beta = non_linear_beta(beta_iter, iteration_number-1, lambda);
  } while(beta <= lambda);

  //reconstruct the output signal
  int count_groups = 0;
  for(int i = 0; i < boost::num_vertices(graph); ++i) {
    if(skipover[i]) {
      continue;
    }
    count_groups++;
    for(auto element_it = groups[i].elements.begin(); element_it != groups[i].elements.end(); ++element_it) {
      graph[(typename Graph::vertex_descriptor)(*element_it)].value = groups[i].average_value;
    }
  }
  return count_groups;
}
#endif
