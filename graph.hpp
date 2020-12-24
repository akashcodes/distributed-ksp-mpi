#include <vector>
#include <tuple>
#include <algorithm>
#include <map>
#include <set>
#include <unordered_set>
#include <queue>
#include <stack>
#include <utility>
#include <climits>
#include <unordered_map>

#define EPSILON 10

/** 
 * TODO:
 * 
 * - Implement Yen's algorithm for Graph::get_bounding_paths
 * - Implement EP-Index for bounding paths
 * 
*/

class Vertex;
class Edge;
class Graph;
class BoundingPath;

class Vertex {
    public:
    int id;
    bool enabled;
    Vertex();
    Vertex(int);
};

Vertex::Vertex() {
    this->id = -1;
    this->enabled = true;
}

Vertex::Vertex(int id) {
    this->id = id;
    this->enabled = true;
}



bool operator==(const Vertex& a, const Vertex& b) {
    if(a.id == b.id) return true;
    return false;
}

bool operator<(const Vertex& a, const Vertex& b) {
    if(a.id < b.id) return true;
    return false;
}



class Edge {
    public:
    Vertex* source;
    Vertex* destination;
    int n_vfrags;
    float vfrag_weight;
    int weight;
    bool enabled;

    Edge(){};
    Edge(Vertex*, Vertex*, int);
    void update_weight(int);
};



/**
 * constructor for the Edge class
*/
Edge::Edge(Vertex* source, Vertex* destination, int weight_param) {
    this->source = source;
    this->destination = destination;
    this->weight = weight_param;
    this->n_vfrags = weight_param;
    this->enabled = true;
    this->vfrag_weight = 1.0; // initially the unit weight is 1
    
}



void Edge::update_weight(int new_weight) {
    this->weight = new_weight;
    this->vfrag_weight = (float)this->n_vfrags/(float)this->weight;
}


bool bfs_path_sort_comparator(const std::vector<Vertex*> a, const std::vector<Vertex*> b) {
    return a.size() < b.size();
}


struct edge_vfrag_weight_comparator {
    bool operator() (const Edge* a, const Edge* b) const {
        return a->vfrag_weight < b->vfrag_weight;
    }
};


// for dijkstra's algorithm
struct edge_weight_comparator {
    bool operator() (const std::pair<int, Vertex*>& a, const std::pair<int, Vertex*>& b) const {
        return a.first > b.first;
    }
};



/**
 * Graph Class
 * 
 * 
*/
class Graph {
    public:

    // adjacency list has vertices and edges
    // efficient storage imo
    std::map<Vertex*, std::map<Vertex*, Edge*> > adjacency_list;
    std::map<int, Vertex*> vertices;
    
    // sorted by vfrag_weight (here vfrag_wright is the unit weight)
    // # of vfrags always remain the same but their weight is updated
    // when the edge weight is updated
    std::set<Edge*, edge_vfrag_weight_comparator> edges;
    
    // keep this sorted, fam
    std::unordered_set<int> boundary_vertices;

    // EP-Index
    //std::unordered_map<Edge*, std::vector<std::vector<Vertex*>>> bounding_paths;

    // bounding paths
    std::unordered_map<Vertex*, std::unordered_map<Vertex*, std::vector<std::vector<Vertex*> > > > bounding_paths;
    std::vector<std::vector<Vertex*> > lower_bounding_paths;

    // default constructor
    Graph() {};

    // parametrized constructor
    // takes in a vector of vertices
    // takes in a vector of edge objects
    Graph(std::vector<std::vector<int> > const&, std::vector<int> const&);

    std::vector<std::vector<Vertex*> > get_k_bounding_paths(Vertex*, Vertex*, int);

    void initialise_bounding_paths(int);
    
    std::vector<Vertex*> get_shortest_path(Vertex*, Vertex*);
    
    std::vector<Vertex*> get_bfs_shortest_path(Vertex*, Vertex*);
    

    Edge* get_edge(int, int);

    // returns true if the edge weight is updated successfully
    bool update_edge_weight(int u, int v, int);

    bool update_edge_weight(Edge*, int);

    void print_neighbours(int);

    void print_vertices();
};



// constructor for Graph
Graph::Graph(std::vector<std::vector<int> > const& edges_param, std::vector<int> const& boundary_vertices_param) {
    for(long int i = 0; i < (long int) edges_param.size(); i++) {
        //std::cout<<"\n";
        int u = edges_param[i][0];
        int v = edges_param[i][1];
        int w = edges_param[i][2];

        if(this->vertices.find(u) == this->vertices.end()) {
            //this->adjacency_list[u] = std::map<Vertex*, Edge*>();
            this->vertices[u] = new Vertex(u);
        }
        if(this->vertices.find(v) == this->vertices.end()) {
            this->vertices[v] = new Vertex(v);
        }
        Edge* e = new Edge(this->vertices[u], this->vertices[v], w);
        this->adjacency_list[this->vertices[u]][this->vertices[v]] = e;
        this->edges.insert(e);

    }

    for(long int i = 0; i < (long int) boundary_vertices_param.size(); i++) {
        int u = boundary_vertices_param[i];

        if(this->vertices.find(u) == this->vertices.end()) {
            this->vertices[u] = new Vertex(u);
        }

        this->boundary_vertices.insert(u);
    }
}




/**
 * TODO:
 * Implement the given function
*/
Edge* Graph::get_edge(int u, int v) {
    if(this->vertices.find(u) == this->vertices.end() || this->vertices.find(u) == this->vertices.end()) {
        return nullptr;
    }
    return this->adjacency_list[this->vertices[u]][this->vertices[v]];
}


/** 
 * This method will return the bounding path between the source and destination vertex
 * source and destination vertices belong to the boundary vertices set of the graph
 * bounding path is the number of edges
 * implementing Yen's algorithm for this
 * finding EPSILON-Shortest paths
 * Reference: https://en.wikipedia.org/wiki/Yen's_algorithm
 * 
 * TODO:
 * 
 * - complete the method
*/
std::vector<std::vector<Vertex*> > Graph::get_k_bounding_paths(Vertex* source, Vertex* destination, int k) {
    
    // this will contain all the paths
    // will be returned
    std::vector<std::vector<Vertex*> > paths;

    std::vector<Vertex*> curpath = this->get_shortest_path(source, destination);

    if(curpath.size() < 2) return paths;

    paths.push_back(curpath);

    std::vector<std::vector<Vertex*> > candidate_paths;

    for(int ki = 1; ki < k; ki++) {

        for(int i = 0; i < paths[ki-1].size()-2; i++) {

            Vertex* spur_node = paths[ki-1][i];

            std::vector<Vertex*> root_path = std::vector<Vertex*> (paths[ki-1].begin(), paths[ki-1].begin() + i);

            std::vector<Edge*> disabled_edges;
            std::vector<Vertex*> disabled_vertices;

            for(std::vector<Vertex*> path: paths) {

                std::vector<Vertex*> sub_path = std::vector<Vertex*> (path.begin(), path.begin() + i);
                
                if(root_path == sub_path) {
                    Vertex* u = path[i];
                    Vertex* v = path[i+1];

                    // remove the edge from the graph
                    // disable it
                    this->adjacency_list[u][v]->enabled = false;
                    disabled_edges.push_back(this->adjacency_list[u][v]);
                }
            }

            for(Vertex* root_path_node: root_path) {
                if(root_path_node == spur_node) {
                    continue;
                }
                root_path_node->enabled = false;
                disabled_vertices.push_back(root_path_node);
            }


            std::vector<Vertex*> spur_path = this->get_shortest_path(spur_node, destination);

            if(spur_path.size() >= 2) {
                std::vector<Vertex*> total_path;

                for(Vertex* n: root_path) {
                    total_path.push_back(n);
                }

                for(Vertex* n: spur_path) {
                    total_path.push_back(n);
                }
                
                
                // if total path not in candidate_paths
                bool existing = true;
                int cpn = candidate_paths.size();
                for(int i = 0; i < cpn; i++) {
                    std::vector<Vertex*> p = candidate_paths[i];

                    if(p != total_path) {
                        existing = false;
                    }
                }

                if(cpn == 0 || !existing) {
                    //cout<<"Existing\n";
                    candidate_paths.push_back(total_path);
                }
            }
            
            for(Vertex* v : disabled_vertices) {
                v->enabled = true;
            }

            for(Edge* e: disabled_edges) {
                e->enabled = true;
            }

        }
        
        if(candidate_paths.empty()) {
            break;
        }

        std::sort(candidate_paths.begin(), candidate_paths.end(), bfs_path_sort_comparator);
        std::vector<Vertex*> p0 = candidate_paths[0];

        paths.push_back(p0);

        candidate_paths.erase(candidate_paths.begin());

    }

    return paths;
}


void Graph::initialise_bounding_paths(int k) {
    std::vector<Vertex*> vertices;
    std::transform(this->boundary_vertices.begin(), this->boundary_vertices.end(), 
                            std::back_inserter(vertices), 
                            [this](const int v) {
                                return this->vertices[v];
                            });
    for(int i = 0; i < vertices.size(); i++) {
        for(int j = 0; j < vertices.size(); j++) {
            if(i == j) {
                continue;
            }
            //this->bounding_paths[vertices[i]][vertices[j]] = this->get_k_bounding_paths(vertices[i], vertices[j], k);
            this->bounding_paths[vertices[i]][vertices[j]] = {this->get_shortest_path(vertices[i], vertices[j])};
            this->lower_bounding_paths.push_back(this->bounding_paths[vertices[i]][vertices[j]][0]);
        }
    }
}

/**
 * this method will return the shortest path from source to destination
 * using the classic Dijkstra's algorithm
 * can be modified to work for unweighted graphs
 * Note: Given that both the source and destination vertices exist in the Graph
*/
std::vector<Vertex*> Graph::get_bfs_shortest_path(Vertex* source, Vertex* destination) {
    
    // this will contain the shortest path
    // will be returned
    std::vector<Vertex*> path;

    std::queue<Vertex*> open;
    
    std::set<Vertex*> visited;
    std::map<Vertex*, Vertex*> predecessor;

    open.push(source);

    std::map<Vertex*, int> dist;

    dist[source] = 0;

    //predecessor[source] = new Vertex();;
    visited.insert(source);
    bool found  = false;
    while(!open.empty()) {
        Vertex* node = open.front();
        open.pop();

        for(auto neighbour: this->adjacency_list[node]) {
            if(!(neighbour.second->enabled)) {
                continue;
            }
            if(!(neighbour.first->enabled)) {
                continue;
            }


            if(visited.find(neighbour.first) == visited.end()) {

                visited.insert(neighbour.first);
                
                predecessor[neighbour.first] = node;
                open.push(neighbour.first);

                dist[neighbour.first] = dist[node] + neighbour.second->weight;

                if(neighbour.first == destination) {
                    found = true;
                    break;
                }
            }
        }
        if(found) {
            break;
        }
    }
    //std::cout<<"Distance "<<dist[destination]<<"\n";
    // constructing the path now
    // using the predessor array
    Vertex* current = destination;
    path.push_back(current);
    //std::cout<<predecessor[current->id]<<" ";
    //return path;
    while(predecessor[current] ){//&& predecessor[current]->id != -1) {
        //std::cout<<predecessor[current]->id<<" ";
        path.push_back(predecessor[current]);
        current = predecessor[current];
    }
    std::reverse(path.begin(), path.end());

    return path;
}



/**
 * this method will return the shortest path from source to destination
 * using the classic Dijkstra's algorithm
 * can be modified to work for unweighted graphs
 * Note: Given that both the source and destination vertices exist in the Graph
*/
std::vector<Vertex*> Graph::get_shortest_path(Vertex* source, Vertex* destination) {
    
    // this will contain the shortest path
    // will be returned
    std::vector<Vertex*> path;

    std::priority_queue<std::pair<int, Vertex*>, std::vector<std::pair<int, Vertex*>>, edge_weight_comparator> open;
    
    std::map<Vertex*, Vertex*> predecessor;

    std::map<Vertex*, int> dist;

    open.push(std::make_pair(0, source));

    dist[source] = 0;

    //predecessor[source] = new Vertex();;
    while(!open.empty()) {
        std::pair<int, Vertex*> pq_pair = open.top();
        open.pop();

        Vertex* node = pq_pair.second;

        if(node == destination) {
            break;
        }

        for(auto neighbour: this->adjacency_list[node]) {
            if(!(neighbour.second->enabled)) {
                continue;
            }
            if(!(neighbour.first->enabled)) {
                continue;
            }

            Vertex* neigh_node = neighbour.first;
            Edge* neigh_edge = neighbour.second;

            if(dist.find(neigh_node) == dist.end()) {
                dist[neigh_node] = INT_MAX;
            }
            if(dist[neigh_node] > dist[node] + neigh_edge->weight) {

                dist[neigh_node] = dist[node] + neigh_edge->weight;

                open.push(std::make_pair(dist[neigh_node], neigh_node));
                
                predecessor[neighbour.first] = node;
            }
        }
    }
    //std::cout<<"Distance "<<dist[destination]<<"\n";
    // constructing the path now
    // using the predessor array
    Vertex* current = destination;
    path.push_back(current);
    //std::cout<<predecessor[current->id]<<" ";
    //return path;
    while(predecessor[current] ){//&& predecessor[current]->id != -1) {
        //std::cout<<predecessor[current]->id<<" ";
        path.push_back(predecessor[current]);
        current = predecessor[current];
    }
    std::reverse(path.begin(), path.end());

    return path;
}


bool Graph::update_edge_weight(Edge* edge, int new_weight) {
    if(edge == nullptr || edge == NULL) {
        return false;
    }
    if(this->edges.find(edge) == this->edges.end()) {
        return false;
    }
    this->edges.erase(edge);
    edge->update_weight(new_weight);
    this->edges.insert(edge);

    return true;
}


bool Graph::update_edge_weight(int u, int v, int new_weight) {
    Edge* edge = this->get_edge(u, v);
    return update_edge_weight(edge, new_weight);
}



void Graph::print_neighbours(int node) {
    Vertex* v = this->vertices[node];
    for(auto neighbours: this->adjacency_list[v]) {
        std::cout<<neighbours.first->id<<" ";
    }
    std::cout<<"\n";
}

void Graph::print_vertices() {
    for(auto v: this->vertices) {
        std::cout<<v.first<<"\n";
    }
    //std::cout<<"\n";
}


class BoundingPath {
    public:
    std::vector<Vertex*> path;
    long long distance;
    Graph* graph;

    BoundingPath(std::vector<Vertex*>, Graph*);

    long long update_distance();
};


BoundingPath::BoundingPath(std::vector<Vertex*> path, Graph* graph) {
    this->graph = graph;
    this->path = path;
    
}


long long BoundingPath::update_distance() {
    return this->distance;
}


/**
 * FIXME: UTILITY FUNCTIONS, NOT TIED TO CLASSES
*/

// to compute the distance of the path
// first parameter is the path vector
// second parameter is the graph's adjacency list
// both passed by reference
int path_distance(std::vector<Vertex*>& path, std::map<Vertex*, std::map<Vertex*, Edge*> >& adjacency_list) {
    int distance = 0;
    int n = path.size();
    for(int i = 1; i < n; i++) {
        Vertex* u = path[i-1];
        Vertex* v = path[i];
        distance += adjacency_list[u][v]->weight;
    }

    return distance;
}