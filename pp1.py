import numpy as np
from collections import deque
import sys
import time
import argparse
import os
import shutil
import pprint


THRESHOLD = 200


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
    neigh_count = []

    for i in range(1, n_nodes+1):
        adj_list.append([])
        neigh_count.append(0)

    for i in range(1, n_edges+1):
        adj_list[data[i][0]-1].append((data[i][0]-1, data[i][1]-1, data[i][2]))
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

    discovered = deque()
    visited = [False] * n_nodes
    closed = [False] * n_nodes
    discovered.append((0, -1))
    subgraphs = []
    subgraph = []
    count = 0

    neighbours = [set()] * n_nodes
    boundary_vertices = [False] * n_nodes
    while discovered:
        node, parent = discovered.popleft()
        subgraph.append(node)
        count += 1
        closed[node] = True
        visited[node] = True
        if count >= THRESHOLD:
            boundary = []
            for v in subgraph:
                for adj in adj_list[v]:
                    nxt = adj[1]
                    if not closed[nxt]:
                        boundary.append(v)
                        #boundary_vertices.append(v)
                        boundary_vertices[v] = True
                        break
            subgraphs.append(list(subgraph))
            subgraph = list(boundary)
            count = len(subgraph)
        
        for adj in adj_list[node]:
            nxt = adj[1]
            if not visited[nxt]:
                visited[nxt] = True
                discovered.append((nxt, node))
    if count > 0:
        boundary = []
        for v in subgraph:
            for nxt in adj_list[v]:
                if not closed[nxt[1]]:
                    boundary.append(v)
                    #boundary_vertices.append(v)
                    boundary_vertices[v] = True
                    break
        subgraphs.append(list(subgraph))
    #pp.pprint(subgraphs)
    print("Created", len(subgraphs), "Subgraphs")
    #for sg in subgraphs:
    #    print(len(sg))
    sys.exit(0)
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