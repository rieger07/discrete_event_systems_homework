# Graph Based Disassembly Sequence Planning

## Cut Vertex Search Analysis
* Do a depth first search (dfs) of the graph to generate G_d from G_c where G_c = (V,E)
* For each step in your search p[i] where i is your step number...
* Let prenum[v] be the number assigned by the search
* After carrying out dfs to create G_d, traverse G_d and calculate the `lowest[v]` as the minimum of the `lowest[x]`
* `lowest[v] = min{prenum[v],min{prenum[w_ij],min{prenum[c_ij]}}}`
* `prenum[v]` is the search order for each node `i`
* `prenum[w_ij]` for each node j such that an edge `E_ij` exists in G_c that has no corresponding edge with node i in G_d... If no `prenum[w_ij]` exsts, the `prenum[w_ij]` is `inf`
* `prenum[c_ij]` for each child j of node i in G_d... If no `prenum[c_ij]` exists.. then `prenum[c_ij] = inf`

### Cut Verticies determination
* Rule 1: Root of graph G_d is a cut vertex of G_c iff it has more than one child
* Rule 2: a vertex v other than root of G_d is a cut vertex of i iff v has a child x such that lowest[x] >= prenum[v]

* When the cut vertices are found, the next step is to find the most complex cut vertex (CV) which has the max num edges in graph G_c
* If there are > two v that have the sam max # edges then CV is the vertex that has the max # of fasteners
* if there are more than two CV that have the have then same edges and fasteners then CV is the smallest component ID

### edges and fasteners of components connected with CV calcuated:
* EN_i = sum(n,j=1 E_ij)
* where i is cut vertex ID
* EN_i is the edge number that is connected with cut vertex i