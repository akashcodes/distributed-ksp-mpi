import numpy as np
from collections import deque
import sys
import time
import argparse
import os
import shutil


THRESHOLD = 200


if __name__ == "__main__":

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

    visited = [False] * n_nodes
    closed = 0

    discovered = deque()
    # starting from node 0
    discovered.append((0, 0, 0))
    visited[0] = True

    neigh_count[0] += 1

    count = 0
    subgraph_count = 0

    subgraphs = []
    subgraph = []

    prev_bvces = []
    setval = 0
    while discovered:
        parent, node, c = discovered.popleft()
        #print("Visited & popped -- ", node, "   Parent:", parent)
        subgraph.append(node)
        neigh_count[parent] -= 1
        count += 1
        setval += 1
        closed += 1

        if count == THRESHOLD:
            #print(prev_bvces)
            bvces = list(prev_bvces)
            tmp_bvces = []
            for x in subgraph:
                if neigh_count[x] > 0:
                    bvces.append(x)
                    tmp_bvces.append(x)
            subgraphs.append({
                "subgraph": subgraph + list(prev_bvces),
                "boundary": bvces
            })
            prev_bvces = list(tmp_bvces)
            # print("Subgraph -- ", subgraph)
            # print("Boundary -- ", bvces)
            subgraph = []
            count = len(prev_bvces)
            setval = 0
        
        for adj in adj_list[node]:
            nxt = adj[1]
            #print(nxt, "is visited:", (nxt in visited))
            if not visited[nxt]:
                visited[nxt] = True
                #print("Adding", nxt)
                discovered.append(adj)
            else:
                neigh_count[node] -= 1
    print("Closed", closed, "nodes")
    if setval != 0:
        bvces = prev_bvces
        for x in subgraph:
            if neigh_count[x] != 0:
                bvces.append(x)

        subgraphs.append({
            "subgraph": subgraph,
            "boundary": bvces
        })

    print("Total number of subgraphs", len(subgraphs))

    fname = 0
    for subgraph in subgraphs:

        with open(output_dir+"/subgraph"+str(fname), "w+") as sg_file:
            dat = []
            for s in subgraph["subgraph"]:
                # sg_file.write(str(s)+"\n")
                
                for tup in adj_list[s]:
                    if tup[1] in subgraph["subgraph"]:
                        da = tup
                        dat.append(da)
                        # da = ' '.join([str(x) for x in tup])
                        # sg_file.write(da+"\n")
            sg_len = len(dat)
            sg_file.write(str(sg_len)+"\n")
            for d in dat:
                sg_file.write(' '.join([str(x) for x in d])+"\n")
            
                
            bd_len = len(subgraph["boundary"])
            sg_file.write(str(bd_len)+"\n")

            for b in subgraph["boundary"]:
                sg_file.write(str(b)+"\n")
        
        fname += 1