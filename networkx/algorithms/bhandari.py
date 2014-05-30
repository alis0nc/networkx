# -*- coding: utf-8 -*-
#    Copyright (C) 2014 by
#    Alison Chan <alisonc@alisonc.net>
#    All rights reserved.
#    BSD license.
import heapq
import networkx as nx
__author__ = """\n""".join(['Alison Chan <alisonc@alisonc.net>'])
__all__ = ['edge_disjoint_pair', 'node_disjoint_pair', 'bellman_ford_path']


def path_to_edgelist(path):
    # there is probably a more pythonic way to do this
    edgelist=[]
    i=1
    while i < len(path):
        a,b = path[i-1],path[i]
        edgelist.append((a, b))
        i+=1
    return edgelist
   
def partition(P, source, target):
    paths = []
    while P.number_of_edges():
        p = nx.shortest_path(P, source, target)
        for src, dst in path_to_edgelist(p):
            P.remove_edge(src, dst)
        paths.append(p)
    return paths

def edge_disjoint_pair(G, source, target, weight='weight', fully_disjoint=False):
    """Generate maximally edge-disjoint pair of paths from source to target. 
    
    Two paths are edge-disjoint if they have no edges in common.

    Parameters
    ----------
    G : NetworkX graph

    source : node
       Starting node for path

    target : node
       Ending node for path
       
    weight : string, optional
        Edge attribute to use as weight (defaults to 'weight')
        
    fully_disjoint : boolean, optional
        Require the paths not to share _any_ edges (defaults to False)

    Returns
    -------
    paths: list
       A list of disjoint paths, each represented as a list. If there 
       are no paths between source and target, the empty list will be 
       returned.

    Examples
    --------
    

    Notes
    -----
    This algorithm uses a modified version of Algorithm 4.1 given in 
    [1]_. Instead of BFS or (modified) Dijkstra, it uses Bellman-Ford 
    shortest path algorithm.

    References
    ----------
    .. [1] R. Bhandari, "Survivable Networks: Algorithms for Diverse
       Routing", Kluwer Academic Publishers, 1999.

    See Also
    --------
    all_shortest_paths, shortest_path
    """
    H = G.to_directed() # working copy
    P = nx.MultiDiGraph() # graph of paths
    # transform to weighted so algorithms work
    for s, t in H.edges_iter():
        if weight not in H[s][t]:
            H[s][t][weight] = 1
    # INF2 = \left\vertE\right\vert l_{max} + \epsilon
    inf2 = (max(H.edges(data=True), key=lambda e: e[2][weight]))[2][weight]*H.number_of_edges() + 1
    # 1. For the pair of vertices under consideration, find the shortest
    #    path
    shortest_path = nx.shortest_path(H, source, target, weight)
    P.add_edges_from(path_to_edgelist(shortest_path))
    # 2. Increment the length of each arc of the shortest path by INF2
    # 3. Make the oppositely directed arcs negative
    for s, t in P.edges_iter():
        H[s][t][weight] += inf2
        H[t][s][weight] = -H[t][s][weight]
    # 4. Run a shortest path algorithm again (this time, it has to deal 
    #    with negative edges)
    new_path = nx.bellman_ford_path(H, source, target, weight)
    # 5. Remove interlacing edges and the desired pair of paths results
    for s, t in path_to_edgelist(new_path):
        if P.has_edge(t, s):
            P.remove_edge(t, s)
        elif P.has_edge(s, t) and fully_disjoint:
            # the two paths share an edge!
            raise nx.NetworkXNoPath("No fully disjoint paths between %s and %s." % (source, target))
        else:
            P.add_edge(s, t)
    #print P.edges()
    # partitioning the set of edges into two distinct paths
    paths = partition(P, source, target)
    return paths
    
def node_disjoint_pair(G, source, target, weight='weight', fully_disjoint=False):
    """Generate maximally node-disjoint pair of paths from source to target. 
    
    Two paths are node-disjoint if they have no edges in common.

    Parameters
    ----------
    G : NetworkX graph

    source : node
       Starting node for path

    target : node
       Ending node for path
       
    weight : string, optional
        Edge attribute to use as weight (defaults to 'weight')
        
    fully_disjoint : boolean, optional
        Require the paths not to share _any_ nodes (defaults to False)

    Returns
    -------
    paths: list
       A list of disjoint paths, each represented as a list. If there 
       are no paths between source and target, the empty list will be 
       returned.

    Examples
    --------
    

    Notes
    -----
    This algorithm uses a modified version of Algorithm 4.1 given in 
    [1]_. Instead of BFS or (modified) Dijkstra, it uses Bellman-Ford 
    shortest path algorithm.

    References
    ----------
    .. [1] R. Bhandari, "Survivable Networks: Algorithms for Diverse
       Routing", Kluwer Academic Publishers, 1999.

    See Also
    --------
    all_shortest_paths, shortest_path
    """
    H = G.to_directed() # working copy
    P = nx.MultiDiGraph() # graph of paths    
    # transform to weighted so algorithms work
    for s, t in H.edges_iter():
        if weight not in H[s][t]:
            H[s][t][weight] = 1
    # INF2 = \left\vertE\right\vert l_{max} + \epsilon
    inf2 = (max(H.edges(data=True), key=lambda e: e[2][weight]))[2][weight]*H.number_of_edges() + 1
    # 1. For the pair of vertices under consideration, find the shortest
    #    path
    shortest_path = nx.shortest_path(H, source, target, weight)
    P.add_edges_from(path_to_edgelist(shortest_path))
    # 2. Increment the length of each arc of the shortest path by INF2
    # 3. Make the oppositely directed arcs negative
    for s, t in P.edges_iter():
        H[s][t][weight] += inf2
        H[t][s][weight] = -H[t][s][weight]
    # 4. Vertex splitting
    for v in P.nodes_iter():
        if not (v==source or v==target): # don't split source or target
            vp = 'p_' + str(v) # prime
            vpp = 'pp_' + str(v) # double prime
            H.add_node(vp) 
            H.add_node(vpp)
            H.add_edge(vpp, vp, weight=0)
            H.add_edge(vp, vpp, weight=G.number_of_nodes()*inf2)
            for nv in G.neighbors(v):
                print v, nv
                if (v, nv) not in P.edges() and (nv, v) not in P.edges():
                    H.add_edge(nv, vp, weight=H[v][nv][weight])                    
                    H.add_edge(vpp, nv, weight=H[v][nv][weight])
                else: # link up the path with prime vertices
                    assert shortest_path.index(nv) != shortest_path.index(v)
                    if shortest_path.index(nv) < shortest_path.index(v):
                        if nv==source:
                            H.add_edge(nv, vp, weight=H[nv][v][weight])
                            H.add_edge(vp, nv, weight=H[v][nv][weight])
                        else:
                            H.add_edge('pp_' + str(nv), vp, weight=H[nv][v][weight])
                            H.add_edge(vp, 'pp_' + str(nv), weight=H[v][nv][weight])
                    else:
                        if nv==target:
                            H.add_edge(vpp, nv, weight=H[v][nv][weight])
                            H.add_edge(nv, vpp, weight=H[nv][v][weight])
                        else:
                            H.add_edge(vpp, 'p_' + str(nv), weight=H[v][nv][weight])
                            H.add_edge('p_' + str(nv), vpp, weight=H[nv][v][weight])
    for v in P.nodes_iter(): # clean up nodes that were split
        if not (v==source or v==target):
            H.remove_node(v)
    # 5. Run a shortest path algorithm again (this time, it has to deal 
    #    with negative edges)
    new_path = nx.bellman_ford_path(H, source, target, weight)
    # Coalesce all split nodes
    
    # 6. Remove interlacing edges and the desired pair of paths results
    for s, t in path_to_edgelist(new_path):
        if P.has_edge(t, s):
            P.remove_edge(t, s)
        elif P.has_edge(s, t) and fully_disjoint:
            # the two paths share an edge!
            raise nx.NetworkXNoPath("No fully disjoint paths between %s and %s." % (source, target))
        else:
            P.add_edge(s, t)
    #print P.edges()
    # partitioning the set of edges into two distinct paths
    paths = partition(P, source, target)
    return paths

def bellman_ford_path(G, source, target=None, weight='weight'):
    """Compute shortest paths in the graph using Bellman-Ford algorithm. 
    Works for digraphs with negative edges.

    Parameters
    ----------
    G : NetworkX graph

    source : node
        Starting node for path.

    target : node, optional
        Ending node for path.
        If not specified, compute shortest paths using all nodes as target nodes.

    weight : None or string, optional (default = 'weight')
        If None, every edge has weight/distance/cost 1.
        If a string, use this edge attribute as the edge weight.
        Any edge attribute not present defaults to 1.

    Returns
    -------
    path: list or dictionary
        All returned paths include both the source and target in the path.

        If the source and target are both specified, return a single list
        of nodes in a shortest path from the source to the target.

        If only the source is specified, return a dictionary keyed by
        targets with a list of nodes in a shortest path from the source
        to one of the targets.

    Examples
    --------
    >>> G=nx.path_graph(5)
    >>> print(nx.bellman_ford_path(G,source=0,target=4))
    [0, 1, 2, 3, 4]
    >>> p = nx.bellman_ford_path(G,source=0)
    >>> print p
    {0: [0], 1: [0, 1], 2: [0, 1, 2], 3: [0, 1, 2, 3], 4: [0, 1, 2, 3, 4]}
    >>> print p[4]
    [0, 1, 2, 3, 4]


    Notes
    -----
    There may be more than one shortest path between a source and target.
    This returns only one of them.

    For digraphs this returns a shortest directed path. To find paths in the
    reverse direction first use G.reverse(copy=False) to flip the edge
    orientation.

    See Also
    --------
    shortest_path()
    bellman_ford()
    """
    if target is None:
        ## Find paths to all nodes accessible from the source.
        paths={}
        pred, dist = nx.bellman_ford(G, source, weight)
        for node in dist:
            p=[]
            edon = node
            while edon is not None:
                # build the path
                p.append(edon)
                edon = pred[edon]
            p.reverse()
            paths[node] = p
    else:
        ## Find shortest source-target path.
        paths=[]
        pred, dist = nx.bellman_ford(G, source, weight)
        # if you can't get there from here, don't bother trying
        if not target in dist: 
            raise nx.NetworkXNoPath("No path between %s and %s." % (source, target))
        while target is not None:
            # build the path
            paths.append(target)
            target = pred[target]
        paths.reverse()
    return paths

