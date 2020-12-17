#include <vector>
#include <fstream>
#include <iostream>
#include <string>
#include "graph.hpp"


int N_SUBGRAPHS = 2218;
std::string SUBGRAPH_DIR = "./ny-subgraph/";

int main(int argc, char** argv) {

    std::vector<Graph*> subgraphs;
    for(int i = 0; i < 4; i += 1) {
        std::ifstream infile;
        infile.open(SUBGRAPH_DIR+"subgraph"+std::to_string(i));
        int n;

        infile >> n;
        // cout<<n<<" Edges\n";
        std::vector<std::vector<int> > edges;
        for(int i = 0; i < n; i++) {
            //cout<<i<<" ";
            edges.push_back(std::vector<int>(3, 0));
            infile>>edges[i][0];
            infile>>edges[i][1];
            infile>>edges[i][2];

        }

        int m;

        infile >> m;

        std::vector<int> boundary_vertices;

        for(int i = 0; i < m; i++) {
            int b;
            infile >> b;

            boundary_vertices.push_back(b);
        }

        Graph* subgraph = new Graph(edges, boundary_vertices);
        subgraph->initialise_bounding_paths(1);
        subgraphs.push_back(subgraph);
        infile.close();

        //std::cout << "Generated subgraph on processor rank - " << world_rank << std::endl;
    }

    for(int i = 0; i < 4; i++) {
        std::cout << subgraphs[i]->boundary_vertices.size() << std::endl;
    }
    
}