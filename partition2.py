import numpy as np
from collections import deque
import sys
import time
import argparse
import os
import shutil
import pprint
from queue import PriorityQueue

THRESHOLD = 200


def is_boundary(node, adj_list, closed):
    for adj in adj_list[node]:
        nxt = adj[1]
        if not closed[nxt]:
            return True
    return False

def is_closed(node, adj_list, closed):
    for adj in adj_list[node]:
        nxt = adj[1]
        if not closed[nxt]:
            return False
    return True
"""
def get_subgraph(root, adj_list, visited, closed, thres):
    subgraph = []
    boundary = []
    discovered = deque()
    discovered.append(root)
    visited[root] = True
    while discovered:
        node = discovered.popleft()
        subgraph.append(node)
        closed[node] = True
        if len(subgraph) >= thres:
            break
        for adj in adj_list[node]:
            nxt = adj[1]
            if not visited[nxt]:
                visited[nxt] = True
                discovered.append(nxt)
    
    visited = list(closed)
    for v in subgraph:
        if is_boundary(v, adj_list, closed):
            boundary.append(v)
            visited[v] = False
    return subgraph, boundary
"""


def get_subgraph(root, adj_list, visited, closed, thres, prev):
    subgraph = list(prev)
    sg_temp = []
    boundary = []
    discovered = PriorityQueue()
    discovered.put((0, root))
    visited[root] = True
    while not discovered.empty():
        dist, node = discovered.get()
        subgraph.append(node)
        sg_temp.append(node)
        closed[node] = True
        if len(subgraph) >= thres:
            break
        for adj in adj_list[node]:
            nxt = adj[1]
            if not visited[nxt]:
                visited[nxt] = True
                discovered.put((dist+adj[0], nxt))
    
    #visited = list(closed)
    for v in sg_temp:
        if is_boundary(v, adj_list, closed):
            boundary.append(v)
            #visited[v] = False
    return subgraph, boundary

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

    visited = [False] * n_nodes
    closed = [False] * n_nodes
    subgraphs = []
    boundary_vertices = [False] * n_nodes

    root = 0
    boundary_vertices[root] = True
    boundaries = deque()
    boundaries.append(root)
    prev = []
    for v in range(0, n_nodes):
        if visited[v]:
            continue
        subgraph, boundary = get_subgraph(v, adj_list, visited, closed, THRESHOLD, prev)
        prev = list(boundary)
        subgraphs.append(subgraph)
        for bv in boundary:
            boundary_vertices[bv] = True
    
    #pp.pprint(subgraphs)
    sgn = len(subgraphs)
    print("Created", sgn, "Subgraphs")
    nb1 = 0
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
        if val == 1:
            nb1 += 1
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
    print("1, 5, 10, 20, 50 ==> ", nb1, nb5, nb10, nb20, nb50)
    sys.exit(0)
    #for sg in subgraphs:
    #    print(len(sg))
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