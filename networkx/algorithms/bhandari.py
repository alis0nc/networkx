# -*- coding: utf-8 -*-
#    Copyright (C) 2012 by
#    Alison Chan <alisonc@alisonc.net>
#    All rights reserved.
#    BSD license.
import heapq
import networkx as nx
__author__ = """\n""".join(['Alison Chan <alisonc@alisonc.net>'])
__all__ = ['edge_disjoint_paths', 'bellman_ford_path']

def edge_disjoint_paths(G, source, target, weight='weight', count=None):
    """Generate edge-disjoint paths from source to target. 
    
    Two paths are edge-disjoint if they have no edges in common.

    Parameters
    ----------
    G : NetworkX graph

    source : node
       Starting node for path

    target : node
       Ending node for path
       
    weight : string, optional
        Edge attribute to use as weight

    count : integer, optional
        How many disjoint paths to generate and return

    Returns
    -------
    paths: list
       A list of disjoint paths, each represented as a list. If there 
       are no paths between source and target, the empty list will be 
       returned. If there are fewer disjoint paths than the given count, 
       as many paths as do exist will be returned.

    Examples
    --------
    

    Notes
    -----
    This algorithm uses a modified version of Algorithm 3.1 given in 
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
    
    def path_to_edgelist(path):
        # there is probably a more pythonic way to do this
        edgelist=[]
        i=1
        while i < len(path):
            a,b = path[i-1],path[i]
            edgelist.append((a, b))
            i+=1
        return edgelist
      
    H = G.copy()
    # 1. Run the shortest path algorithm for the given pair of vertices
    shortest_path = nx.bellman_ford_path(H, source, target, weight)
    if shortest_path = None:
        # oh no! you can't get there from here! :(
        return None
    # 2. Replace each edge of the shortest path (equivalent to two 
    #    oppositely directed arcs) by a single arc directed towards the 
    #    source vertex
    # 3. Make the length of each of the above arcs negative
    shortest_path = path_to_edgelist(shortest_path)
    print shortest_path
    for a, b in shortest_path:
        H.remove_edge(a, b)
        H[b][a][weight] = -1
    # 4. Run the shortest path algorithm again
    new_path = nx.bellman_ford_path(H, source, target, weight)
    if new_path is None:
        # oh no! we have a bridge! :(
        return [shortest_path]
    new_path = path_to_edgelist(new_path)
    print new_path
    # 5. Erase the overlapping edges of the two paths found, and reverse
    #    the direction of the remaining arcs on the first shortest path 
    #    such that each arc on it is directed towards the sink vertex 
    #    now. The desired pair of paths results.
    edge_set = set(shortest_path) ^ set(new_path)
    overlap = set()
    for e1 in shortest_path:
        for e2 in new_path:
            if (e1[0]==e2[0] and e1[1]==e2[1]) or (e1[0]==e2[1] and e1[1]==e2[0]):
                overlap.add(e1)
                overlap.add(e2)
            else:
                pass
    edge_set -= overlap
    print edge_set

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
        if not target in dist: return None
        while target is not None:
            # build the path
            paths.append(target)
            target = pred[target]
        paths.reverse()
    return paths

