using UnityEngine;
using System.Collections;
using System.Collections.Generic;
using String = System.String;

public class Pathfinding : MonoBehaviour
{
    public Transform seeker, target;
    private Grid grid;
    private int DFS_Count = 0;
    private List<Node> path;
    int flag=0;
    private void Awake()
    {
        grid = GetComponent<Grid>();
    }

    private void Update()
    {
        StartCoroutine(FindPaths());
    }

    private IEnumerator FindPaths()
    {
        if(flag==0){
        var DFS_run = new System.Diagnostics.Stopwatch();
        var BFS_run = new System.Diagnostics.Stopwatch();
        var UCS_run = new System.Diagnostics.Stopwatch();
        var Astar_Euclidian_run = new System.Diagnostics.Stopwatch();
        var Astar_Manhattan_run = new System.Diagnostics.Stopwatch();

        DFS_run.Start();
        DFS_findpath(seeker.position, target.position);
        DFS_run.Stop();
        Debug.Log("The time needed to execute the DFS algorithm is: " + DFS_run.Elapsed);

        BFS_run.Start();
        BFS_findpath(seeker.position, target.position);
        BFS_run.Stop();
        Debug.Log("The time needed to execute the BFS algorithm is: " + BFS_run.Elapsed);

        UCS_run.Start();
        UCS_findpath(seeker.position, target.position);
        UCS_run.Stop();
        Debug.Log("The time needed to execute the UCS algorithm is: " + UCS_run.Elapsed);

        Astar_Euclidian_run.Start();
        List<Node> Astar_Euclidean_path = FindPath(seeker.position, target.position, "EUCLIDEAN");
        Astar_Euclidian_run.Stop();
        Debug.Log("The time needed to execute the A* algorithm with the Euclidean distance heuristic is: " + Astar_Euclidian_run.Elapsed);

        Astar_Manhattan_run.Start();
        List<Node> Astar_Manhattan_path = FindPath(seeker.position, target.position, "MANHATTAN");
        Astar_Manhattan_run.Stop();
        Debug.Log("The time needed to execute the A* algorithm with the Manhattan distance heuristic is: " + Astar_Manhattan_run.Elapsed);

        foreach (Node node in Astar_Manhattan_path)
        {
            seeker.position = node.worldPosition;
            yield return new WaitForSeconds(0f); // wait for a short time before moving to the next node
        }
        flag++;
        }
    }

    private void DFS_findpath(Vector3 seeker, Vector3 Target)
    {
        Node startNode = grid.NodeFromWorldPoint(seeker);
        Node targetNode = grid.NodeFromWorldPoint(Target);
        Stack<Node> fringe = new Stack<Node>();
        HashSet<Node> Closed_list = new HashSet<Node>();
        fringe.Push(startNode);
        int maxFringeSize = 0;

        while (fringe.Count != 0)
        {
            Node currentNode = fringe.Pop();
            if (currentNode == targetNode)
            {
                RetracePath(startNode, targetNode, "DFS");
                Debug.Log("the number of expanded nodes of the DFS strategy is: " + DFS_Count);
                Debug.Log("the maximum size of the fringe during the search process of DFS is: " + maxFringeSize);
                return;
            }

            Closed_list.Add(currentNode);

            foreach (Node neighbour in grid.GetNeighbours(currentNode))
            {
                if (!neighbour.walkable || Closed_list.Contains(neighbour))
                {
                    continue;
                }
                if (neighbour.walkable || !fringe.Contains(neighbour))
                {
                    Closed_list.Add(neighbour);
                    neighbour.parent = currentNode;
                    fringe.Push(neighbour);
                    DFS_Count++;
                    maxFringeSize = Mathf.Max(maxFringeSize, fringe.Count);
                }
        }
    }
}

void BFS_findpath(Vector3 seeker, Vector3 Target){
    Node startNode = grid.NodeFromWorldPoint(seeker);
    Node targetNode = grid.NodeFromWorldPoint(Target);
    Queue<Node> BFS_queue = new Queue<Node>();
    HashSet<Node> Closed_list = new HashSet<Node>();
    BFS_queue.Enqueue(startNode);
	int count =0;
    int maxFringeSize = 0;

    while (BFS_queue.Count != 0){
        Node currentNode = BFS_queue.Dequeue();
        if (currentNode == targetNode){
            RetracePath(startNode, targetNode,"BFS");
            print("the maximum size of the fringe during the search process of BFS is: " + maxFringeSize);
			print("the number of expanded nodes of the bfs stratefy is:"+count);
            return;
        }

        Closed_list.Add(currentNode);

        foreach (Node neighbour in grid.GetNeighbours(currentNode)){
            if (!neighbour.walkable || Closed_list.Contains(neighbour)){
                continue;
            }
            if (neighbour.walkable || !BFS_queue.Contains(neighbour)){
                Closed_list.Add(neighbour);
                neighbour.parent = currentNode;
                BFS_queue.Enqueue(neighbour);
                maxFringeSize = Mathf.Max(maxFringeSize, BFS_queue.Count);
				count++;
            }
        }
    }
}


void UCS_findpath(Vector3 seeker, Vector3 Target){
    Node startNode = grid.NodeFromWorldPoint(seeker);
    Node targetNode = grid.NodeFromWorldPoint(Target);

    List<Node> UCS_stack = new List<Node>();
    HashSet<Node> Closed_list = new HashSet<Node>();
    UCS_stack.Add(startNode);

    int ucs_nodes = 0; // variable to count expanded nodes
    int ucs_max = 0; // variable to store maximum fringe size

    while (UCS_stack.Count > 0){
        Node currentNode = UCS_stack[UCS_stack.Count - 1];
        UCS_stack.RemoveAt(UCS_stack.Count - 1);
        ucs_nodes++;
        if (currentNode == targetNode){
            RetracePath(startNode, targetNode,"UCS");
            Debug.Log("UCS expanded " + ucs_nodes + " nodes"); // print number of expanded nodes
            Debug.Log("UCS maximum fringe size " + ucs_max); // print maximum fringe size
            return;
        }
        Closed_list.Add(currentNode);

        foreach (Node neighbour in grid.GetNeighbours(currentNode)){
            if (!neighbour.walkable || Closed_list.Contains(neighbour)){
                continue;
            }

            int newCostToNeighbour = currentNode.gCost;
            if (newCostToNeighbour < neighbour.gCost || !UCS_stack.Contains(neighbour)){
                neighbour.gCost = newCostToNeighbour;
                neighbour.hCost = 0;
                neighbour.parent = currentNode;

                if (!UCS_stack.Contains(neighbour))
                    UCS_stack.Add(neighbour);
            }
        }

        if (UCS_stack.Count > ucs_max) {
            ucs_max = UCS_stack.Count;
        }

        UCS_stack.Sort((x, y) => x.gCost.CompareTo(y.gCost));
    }
}



public List<Node> FindPath(Vector3 startPos, Vector3 targetPos, string strategy) {
    Node startNode = grid.NodeFromWorldPoint(startPos);
    Node targetNode = grid.NodeFromWorldPoint(targetPos);

    List<Node> openSet = new List<Node>();
    HashSet<Node> closedSet = new HashSet<Node>();
    openSet.Add(startNode);

    int maxFringeSize = 0;
    int expandedNodes = 0;

    while (openSet.Count > 0) {
        if (openSet.Count > maxFringeSize) {
            maxFringeSize = openSet.Count;
        }

        Node node = openSet[0];
        for (int i = 1; i < openSet.Count; i++) {
            if (openSet[i].fCost < node.fCost || openSet[i].fCost == node.fCost) {
                if (openSet[i].hCost < node.hCost)
                    node = openSet[i];
            }
        }

        openSet.Remove(node);
        closedSet.Add(node);

        if (node == targetNode) {
            if (strategy == "EUCLIDEAN") {
                Debug.Log("Number of expanded nodes for " + strategy + " heuristic: " + expandedNodes);
                Debug.Log("Max fringe size for " + strategy + " heuristic: " + maxFringeSize);
                return RetracePath(startNode, targetNode,strategy);
            }
            Debug.Log("Number of expanded nodes for " + strategy + " heuristic: " + expandedNodes);
            Debug.Log("Max fringe size for " + strategy + " heuristic: " + maxFringeSize);
            return  RetracePath(startNode, targetNode,strategy);
        }

        foreach (Node neighbour in grid.GetNeighbours(node)) {
            if (!neighbour.walkable || closedSet.Contains(neighbour)) {
                continue;
            }

            int newCostToNeighbour;
            if (strategy == "EUCLIDEAN") {
                newCostToNeighbour = node.gCost + GetDistance_Euclidian(node, neighbour);
            } else {
                newCostToNeighbour = node.gCost + GetDistance_Manhattan(node, neighbour);
            }

            if (newCostToNeighbour < neighbour.gCost || !openSet.Contains(neighbour)) {
                neighbour.gCost = newCostToNeighbour;

                if (strategy == "EUCLIDEAN") {
                    neighbour.hCost = GetDistance_Euclidian(neighbour, targetNode);
                } else {
                    neighbour.hCost = GetDistance_Manhattan(neighbour, targetNode);
                }

                neighbour.parent = node;

                if (!openSet.Contains(neighbour)) {
                    openSet.Add(neighbour);
                }
            }
        }

        expandedNodes++;
    }

    Debug.Log("No path found for " + strategy + " heuristic.");
    return null;
}



int GetDistance_Euclidian(Node nodeA, Node nodeB) {
    Vector3 posA = nodeA.worldPosition;
    Vector3 posB = nodeB.worldPosition;
    int distance =(int) Vector3.Distance(posA, posB);
    return distance;
}


	int GetDistance_Manhattan(Node nodeA, Node nodeB) {
    int dx = Mathf.Abs(nodeA.gridX - nodeB.gridX);
    int dy = Mathf.Abs(nodeA.gridY - nodeB.gridY);
    int cost = 10; // cost of moving horizontally or vertically
    int distance = cost * (dx + dy);
    return distance;
}


	public List<Node> RetracePath(Node startNode, Node endNode, String strategy){
		List<Node> path = new List<Node>();
		Node currentNode = endNode;
        if(strategy=="DFS"){

		while (currentNode != startNode){
			path.Add(currentNode);
			currentNode = currentNode.parent;
			
		}
		path.Reverse();

		grid.DFS_path = path;
        }
         if(strategy=="BFS"){

		while (currentNode != startNode)
		{
			path.Add(currentNode);
			currentNode = currentNode.parent;
		
		}
		path.Reverse();
		grid.BFS_path = path;
        }
         if(strategy=="UCS"){

		while (currentNode != startNode)
		{
			path.Add(currentNode);
			currentNode = currentNode.parent;
			
		}
		path.Reverse();
		grid.UCS_path = path;
        }
         if(strategy=="EUCLIDEAN"){

		while (currentNode != startNode)
		{
			path.Add(currentNode);
			currentNode = currentNode.parent;
			
		}
		path.Reverse();
		grid.Astar_Euclidian_path = path;
        }
         if(strategy=="MANHATTAN"){

		while (currentNode != startNode)
		{
			path.Add(currentNode);
			currentNode = currentNode.parent;
			
		}
		path.Reverse();
		grid.Astar_Manhattan_path = path;
        }
       return path;

	}

}


