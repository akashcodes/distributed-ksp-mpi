import numpy as np
from collections import deque
import sys
import time
import argparse
import os
import shutil
import pprint
from queue import PriorityQueue

THRESHOLD = 40

def is_boundary(node, adj_list, closed):
    for adj in adj_list[node]:
        nxt = adj[1]
        if not closed[nxt]:
            return True
    return False


if __name__ == "__main__":
    pp = pprint.PrettyPrinter(indent=2)

    args = sys.argv

    print(args)

    input_graph = args[1]
    output_dir = args[2]

    if os.path.exists(output_dir):
        shutil.rmtree(output_dir)
    
    try:
        os.makedirs(output_dir)
    except:
        print("Unable to create output directory.")
        sys.exit(0)

    data = None

    try:
        data = np.loadtxt(input_graph, dtype=int)
    except:
        print("Input file not found")
        sys.exit(0)

    n_nodes = data[0][0]
    n_edges = data[0][1]

    adj_list = []
    adj_list_set = []
    neigh_count = []

    for i in range(1, n_nodes+1):
        adj_list.append([])
        adj_list_set.append(set())
        neigh_count.append(0)

    for i in range(1, n_edges+1):
        adj_list[data[i][0]-1].append((data[i][0]-1, data[i][1]-1, data[i][2]))
        adj_list_set[data[i][0]-1].add(data[i][1]-1)
        neigh_count[data[i][0]-1] += 1

    maxd = 0
    totald = 0

    for i, adj in enumerate(adj_list):
        # sort based on distance
        # adj.sort(key=lambda x: x[2])

        # sort based on neighbour id
        adj.sort(key=lambda x: x[1])

        maxd1 = len(adj)
        if maxd1 > maxd:
            maxd = maxd1
        totald += maxd1

    print("Maximum Degree", maxd)

    print(adj_list[0])

    #sys.exit(0)

    visited = [False] * n_nodes
    closed = [False] * n_nodes
    subgraphs = []
    boundary_vertices = [False] * n_nodes

    nodes = set([x for x in range(0, n_nodes)])
    """
    while nodes:
        n = nodes.pop()
        if visited[n]:
            continue
        discovered = deque()
        discovered.append(n)
        visited[n] = True
        subgraph = []
        
        while discovered:
            cur = discovered.popleft()
            subgraph.append(cur)
            closed[cur] = True
            if len(subgraph) >= THRESHOLD:
                for v in subgraph:
                    if is_boundary(v, adj_list, closed):
                        visited[v] = False
                        boundary_vertices[v] = True
                break

            for adj in adj_list[cur]:
                nxt = adj[1]
                if not visited[nxt]:
                    discovered.append(nxt)
                    visited[nxt] = True
        
        if len(subgraph) > 0:
            subgraphs.append(list(subgraph))
    """
    """
    for node in range(0, n_nodes):
        if not visited[node]:
            discovered = deque()
            discovered.append(node)
            visited[node] = True
            subgraph = []

            while discovered:
                cur = discovered.popleft()
                subgraph.append(cur)
                closed[cur] = True
                if len(subgraph) >= THRESHOLD:
                    subgraphs.append(list(subgraph))
                    boundary = []
                    for v in subgraph:
                        if is_boundary(v, adj_list, closed):
                            visited[v] = False
                    break
                
                for adj in adj_list[cur]:
                    nxt = adj[1]
                    if not visited[nxt]:
                        discovered.append(nxt)
                        visited[nxt] = True
            if len(subgraph) > 0:
                subgraphs.append(subgraph)
    """
    subgraph = []
    sg_temp = []
    root = 0
    #discovered = PriorityQueue()
    #discovered.put((0, root))
    discovered = deque()
    discovered.append(0)
    #while not discovered.empty():
    while discovered:
        #dist, node = discovered.get()
        node = discovered.popleft()
        subgraph.append(node)
        sg_temp.append(node)
        closed[node] = True
        if len(subgraph) >= THRESHOLD:
            subgraphs.append(list(subgraph))
            boundary = []
            for v in sg_temp:
                if is_boundary(v, adj_list, closed):
                    boundary.append(v)
                    boundary_vertices[v] = True
            subgraph = list(boundary)
            sg_temp = []
        
        for adj in adj_list[node]:
            nxt = adj[1]
            if not visited[nxt]:
                #discovered.put((dist + adj[0], nxt))
                discovered.append(nxt)
                visited[nxt] = True
    
    if len(subgraph) > 0:
        #boundary = []
        for v in subgraph:
            if is_boundary(v, adj_list, closed):
                #boundary.append(v)
                boundary_vertices[v] = True
        subgraphs.append(list(subgraph))
    
    sgn = len(subgraphs)
    print("Created", sgn, "Subgraphs")
    nbthres = 0
    nb5 = 0
    nb10 = 0
    nb20 = 0
    nb50 = 0
    mbv = 0
    edges = 0
    for sg in subgraphs:
        val = 0
        for v in sg:
            if boundary_vertices[v]:
                val += 1
        #print(val)
        mbv = max(mbv, val)
        e = v*(val-1)/2
        edges += e
        if val == THRESHOLD:
            nbthres += 1
        if val > 5:
            nb5 += 1
            if val > 10:
                nb10 += 1
                if val > 20:
                    nb20 += 1
                    if val > 50:
                        nb50 += 1
    print("Maximum boundary vertex size", mbv)
    print("#edges in skeleton graph", edges)
    print("thres, 5, 10, 20, 50 ==> ", nbthres, nb5, nb10, nb20, nb50)
    #sys.exit(0)
    fname = 0
    for subgraph in subgraphs:
        with open(output_dir+"/subgraph"+str(fname), "w+") as sg_file:
            dat = []
            bv = []
            for s in subgraph:
                if boundary_vertices[s]:
                    bv.append(s)
                for tup in adj_list[s]:
                    if tup[1] in subgraph:
                        da = tup
                        dat.append(da)
            sg_len = len(dat)
            sg_file.write(str(sg_len)+"\n")
            for d in dat:
                sg_file.write(' '.join([str(x) for x in d])+"\n")
            bd_len = len(bv)
            sg_file.write(str(bd_len)+"\n")
            for b in bv:
                sg_file.write(str(b)+"\n")
        fname += 1