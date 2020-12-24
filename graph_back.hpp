#include <mpi.h>
#include <vector>
#include <fstream>
#include <iostream>
#include <stdlib.h>
#include <unordered_map>
#include <string>
#include "graph.hpp"


int N_SUBGRAPHS = 100;
std::string SUBGRAPH_DIR = "./ny/";
int K = 2;

int main(int argc, char** argv) {
    // Initialize the MPI environment
    MPI_Init(NULL, NULL);

    // Get the number of processes
    int world_size;
    MPI_Comm_size(MPI_COMM_WORLD, &world_size);

    // Get the rank of the process
    int world_rank;
    MPI_Comm_rank(MPI_COMM_WORLD, &world_rank);

    std::vector<Graph*> subgraphs;
    for(int i = world_rank; i < N_SUBGRAPHS; i += world_size) {
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
        subgraph->initialise_bounding_paths(K);
        subgraphs.push_back(subgraph);
        infile.close();

        //std::cout << "Generated subgraph on processor rank - " << world_rank << std::endl;
    }

    // wait till all processors have created subgraphs
    MPI_Barrier(MPI_COMM_WORLD);

    std::vector<std::vector<int> > edges;

    // subgraph index to send for the current node
    int si = 0;
    std::vector<long long> data_buf;
    for(int i = 0; i < N_SUBGRAPHS; i++) {
        int pid = i%world_size;
        data_buf.clear();
        int n;

        if(pid == world_rank) {
            // put data into buffer
            /*for(auto path : subgraphs[si]->bounding_paths) {
                //if(di >= n) continue;
                data[di++] = path.front()->id;
                //if(di >= n) continue;
                data[di++] = path.back()->id;
                //if(di >= n) continue;
                data[di++] = path_distance(path, subgraphs[si]->adjacency_list);
            }
            si++;
            */
           for(auto v1: subgraphs[si]->bounding_paths) {
               for(auto v2: v1.second) {
                   data_buf.push_back(v1.first->id);
                   data_buf.push_back(v2.first->id);
                   //data_buf.push_back(path_distance(v2.second[0], subgraphs[si]->adjacency_list));
                   data_buf.push_back(0);
                   //data[di++] = v1.first->id;
                   //data[di++] = v2.first->id;
                   //data[di++] = path_distance(v2.second[0], subgraphs[si]->adjacency_list);
               }
           }
           si++;
           n = data_buf.size();
        }

        MPI_Bcast(&n, 1, MPI_INT, pid, MPI_COMM_WORLD);
        MPI_Barrier(MPI_COMM_WORLD);
        // now we create a data buffer
        int data[n];
        if(pid == world_rank) {
            for(int di = 0; di < n; di++) {
                data[di] = data_buf[di];
                //std::cout << data[di] << std::endl;
            }
        }
        
        MPI_Bcast(data, n, MPI_INT, pid, MPI_COMM_WORLD);
        for(int j = 0; j < n; j += 3) {
            edges.push_back({data[j+0], data[j+1], data[j+2]});
        }
        MPI_Barrier(MPI_COMM_WORLD);
    }
    // now all have edges
    std::cout << "Generated subgraph on processor rank - " << world_rank << " has edges - " << edges.size() << std::endl;

    // Finalize the MPI environment.
    MPI_Finalize();
}