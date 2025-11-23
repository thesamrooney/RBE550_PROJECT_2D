using UnityEngine;
using UnityEngine.Tilemaps;
using System.Collections.Generic;

public class RRTScript : MonoBehaviour {
    
    // this is all stuff for the settings planner inspector
    [Header("Tilemap")]
    [SerializeField] Tilemap tilemap;   
    public Rect workspace;              

    [Header("Start & Goal")]
    public Vector2 startPos = new Vector2(-8, -4);
    public Vector2 goalPos  = new Vector2(8,  4);
    public float goalRadius = 0.5f;

    [Header("Planner Settings")]
    public float stepSize = 0.1f;
    public float neighborRadius = 1.5f;
    public int   maxIterations = 2000;
    [Range(0,1)] public float goalBias = 0.1f;

    [Header("Collision Settings")]
    public LayerMask obstacleMask;
    public float agentRadius = 0.2f;

    [Header("Agent/Goal Sprites")]
    public GameObject agentSprite;
    public GameObject goalSprite;
    public float moveSpeed = 3f;

    // RRT graph node class
    class Node{
        public Vector2 pos;
        public Node parent;
        public float cost;
        public Node(Vector2 p, Node par, float c) {
            pos = p; 
            parent = par; 
            cost = c;
        }
    }

    // init tree, path, and other flags
    List<Node> tree = new List<Node>();
    List<Vector2> bestPath = new List<Vector2>();
    bool found = false;

    Node bestGoalNode;
    float bestGoalCost = Mathf.Infinity;

    // game objects
    GameObject agent, goal;
    Rigidbody2D agent_body;
    int pathIndex;

    private GoalProxCostMap proxCostMap;

    System.Random rng = new System.Random();
    double randomValue() => rng.NextDouble();

    void Start() {
        resetPlanner();
        proxCostMap = new GoalProxCostMap(tilemap, obstacleMask, goalPos);


        // spawn agent and rigid body stuff
        agent = Instantiate(agentSprite, startPos, Quaternion.identity);
        agent_body = agent.GetComponent<Rigidbody2D>();

        agent_body.bodyType = RigidbodyType2D.Kinematic;
        agent_body.gravityScale = 0f;
        agent_body.freezeRotation = true;

        // spawn goal
        goal = Instantiate(goalSprite, goalPos, Quaternion.identity);
    }

    void Update() {
        // limits tree expansions per frame
        int rrtCount = 50;
        if (!found && tree.Count < maxIterations){
            int i = 0;
            while(!found && i < rrtCount){
                rrtStep();
                i++;
            }
        }

                
        // moves agent if best path found and
        if (found && bestPath.Count > 1)
            moveAgent();

        if (!found && tree.Count >= maxIterations && bestGoalNode != null){
            bestPath = gatherPath(bestGoalNode);
            found = true;
        }

    }

    //
    // Planner stuff --------
    //

    // resets plannner/flags and new tree
    void resetPlanner() {
        tree.Clear();
        bestPath.Clear();
        found = false;
        pathIndex = 0;
        tree.Add(new Node(startPos, null, 0f));

        bestGoalNode = null;
        bestGoalCost = Mathf.Infinity;

    }

    // performs rrt step
    void rrtStep() {
        if (tree.Count == 0) 
            return;

        // random sample towards goal
        Vector2 samplePos;
        if (randomValue() < goalBias){
            samplePos = goalPos;
        }else{
            samplePos = samplePoint();
        }
        
        // finds the nearest node in the current tree
        // then moves towards sample by step size
        Node nearestNode = findNearestNode(samplePos);
        Vector2 newPoint = step(nearestNode.pos, samplePos, stepSize);

        // collision check, thanks chat
        if(collidesAlong(nearestNode.pos, newPoint)) 
            return;

        float goalCost = proxCostMap.getProxCost(newPoint);
        // if(isInfinity(goalCost)) return;
        float proxWeight = 100f;

        // adds new closest node to tree
        float newCost = nearestNode.cost + Vector2.Distance(nearestNode.pos, newPoint);
        newCost += proxWeight * goalCost;

        Node newNode = new Node(newPoint, nearestNode, newCost);
        tree.Add(newNode);

        // checks if reached goal
        if (Vector2.Distance(newNode.pos, goalPos) <= goalRadius && newNode.cost < bestGoalCost){
            bestGoalCost = newNode.cost;
            bestGoalNode = newNode;

            // bestPath = gatherPath(newNode);
            // found = true;
        }
    }

    // move drone along the best path (rigid body nonesense)
    void moveAgent(){
        if (pathIndex >= bestPath.Count) 
            return;

        Vector2 pos = agent_body.position;
        Vector2 target = bestPath[pathIndex];
        Vector2 direction = target - pos;

        if (direction.magnitude < 0.1f) { 
            pathIndex++; 
            return;
        }

        Vector2 vel = direction.normalized * moveSpeed;
        agent_body.MovePosition(pos + vel * Time.deltaTime);
    }

    //
    // Helpers --------
    //

    // pick random point within workspace
    Vector2 samplePoint(){
        float x = (float)(workspace.xMin + randomValue() * workspace.width);
        float y = (float)(workspace.yMin + randomValue() * workspace.height);
        return new Vector2(x, y);
    }

    // finds node in tree closest to given point
    // calculates distance to each node
    Node findNearestNode(Vector2 pos){
        Node best = tree[0];
        float bestDist = (pos - best.pos).sqrMagnitude;

        for (int i = 1; i < tree.Count; i++){
            float dist = (pos - tree[i].pos).sqrMagnitude;

            if (dist < bestDist) { 
                best = tree[i]; 
                bestDist = dist; 
            }
        }
        return best;
    }

    /**
        // finds all nodes within a given radius of a point, used in RRT*
        List<Node> findNeighbors(Vector2 pos, float radius){
            float r2 = radius * radius;
            var list = new List<Node>();
            
            foreach (var node in tree)
                if ((node.pos - pos).sqrMagnitude <= r2) 
                    list.Add(node);
            
            return list;
        }
    **/

    // handles stepping from different positions in RRT
    Vector2 step(Vector2 from, Vector2 to, float maxStep){
        Vector2 dir = to - from;
        float d = dir.magnitude;
        
        if (d < Mathf.Epsilon) 
            return from;
        
        return from + dir.normalized * Mathf.Min(maxStep, d);
    }

    // thanks chat, collision checking between two points, ensures clerance
    bool collidesAlong(Vector2 a, Vector2 b){
        float dist = Vector2.Distance(a, b);
        int samples = Mathf.Max(2, Mathf.CeilToInt(dist / (agentRadius * 0.9f)));
        Vector2 step = (b - a) / samples;
        Vector2 p = a;
        for (int i = 0; i <= samples; i++){
            if (Physics2D.OverlapCircle(p, agentRadius, obstacleMask)) return true;
            p += step;
        }
        return Physics2D.CircleCast(a, agentRadius, (b - a).normalized, dist, obstacleMask);
    }

    // once best bath is found, generates the path node list to traverse
    List<Vector2> gatherPath(Node goalNode){
        var path = new List<Vector2>();

        for (Node n = goalNode; n != null; n = n.parent){
            path.Add(n.pos);
        }

        path.Reverse();
        path.Add(goalPos);
        return path;
    }

    // thanks chat lol
    void OnDrawGizmos(){
        // Draw workspace bounds even in edit mode
        Gizmos.color = Color.white;
        Gizmos.DrawWireCube(workspace.center, workspace.size);

        Gizmos.color = Color.green;
        Gizmos.DrawWireSphere(startPos, 0.2f);
        Gizmos.color = Color.red;
        Gizmos.DrawWireSphere(goalPos, goalRadius);

        if (!Application.isPlaying) return;

        Gizmos.color = new Color(0f, 0.7f, 1f, 0.3f);
        foreach (var n in tree)
            if (n.parent != null) Gizmos.DrawLine(n.pos, n.parent.pos);

        if (bestPath != null && bestPath.Count > 1)
        {
            Gizmos.color = Color.yellow;
            for (int i = 0; i < bestPath.Count - 1; i++)
                Gizmos.DrawLine(bestPath[i], bestPath[i + 1]);
        }
    }
}


public class GoalProxCostMap{
    // fields for the inputs
    Tilemap tilemap;
    LayerMask obstacleMask;
    Vector2 goal;

    // fields that used to be initialized off the primary constructor params
    BoundsInt bounds;
    int width;
    int height;

    float[,] costMap;
    bool[,] occupancyGrid;

    // constructor: this replaces the `(Tilemap, LayerMask, Vector2)` in the class header
    public GoalProxCostMap(Tilemap tilemap, LayerMask obstacleMask, Vector2 goal){
        this.tilemap = tilemap;
        this.obstacleMask = obstacleMask;
        this.goal = goal;

        bounds = tilemap.cellBounds;
        width = bounds.size.x;
        height = bounds.size.y;

        costMap = new float[width, height];
        occupancyGrid = new bool[width, height];
    }
    
    void buildGrid() {
        Vector3 cellSize = tilemap.layoutGrid.cellSize;
        Vector2 size = new Vector2(cellSize.x, cellSize.y) * 0.9f;

        // loop over each cell in the bounds
        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                int cellX = bounds.xMin + x;
                int cellY = bounds.yMin + y;

                Vector3Int cell = new Vector3Int(cellX, cellY, 0);
                Vector3 worldCenter = tilemap.GetCellCenterWorld(cell);
                Collider2D hit = Physics2D.OverlapBox((Vector2)worldCenter, size, 0f, obstacleMask);

                if(hit == null){
                    occupancyGrid[x, y] = false;
                }else{
                    occupancyGrid[x, y] = true;
                }
            }
        }

        Debug.Log("built grid");
    }

    float[,] genCostMap(){
        buildGrid();

        int[] dX = { 2, -2, 1, -1, 0, 0 };
        int[] dY = { 0, 0, 1, -1, 2, -2 };
        int numNeigh = 8;

        float[,] costMap = new float[width, height];
        bool[,] visited = new bool[width, height];

        // WorldToCell takes a Vector3
        Vector3Int cellGoal = tilemap.WorldToCell(new Vector3(goal.x, goal.y, 0f));
        int gx = cellGoal.x - bounds.xMin;
        int gy = cellGoal.y - bounds.yMin;
        int totalCells = width * height;

        // init distances and visited to infinity/false
        for (int x = 0; x < width; x++) {
            for (int y = 0; y < height; y++) {
                costMap[x,y] = float.PositiveInfinity;
                visited[x,y] = false;
            }
        }

        // was 'dist[gx, gy]' (undefined); you clearly meant costMap
        costMap[gx, gy] = 0f;

        // main Dijkstra loop 
        for (int step = 0; step < totalCells; step++){
            // find unvisited cell with smallest dist
            float bestDist = float.PositiveInfinity;
            int bestX = -1;
            int bestY = -1;

            for (int x = 0; x < width; x++){
                for (int y = 0; y < height; y++){
                    if (!visited[x, y] && costMap[x,y] < bestDist){
                        bestDist = costMap[x, y];
                        bestX = x;
                        bestY = y;
                    }
                }
            }

            if (bestX == -1) break; // no more reachable cells

            visited[bestX, bestY] = true;

            // relax neighbors
            for (int k = 0; k < numNeigh; k++){
                int new_x = bestX + dX[k];  // was 'dx'
                int new_y = bestY + dY[k];  // was 'dy'

                // if out of bounds or cell is occupied (leaves occupied cell cost at inf)
                if (new_x < 0 || new_x >= width || new_y < 0 || new_y >= height || occupancyGrid[new_x, new_y])
                    continue;

                float newDist = costMap[bestX, bestY] + 1f; // uniform cost

                if (newDist < costMap[new_x, new_y])
                    costMap[new_x, new_y] = newDist;
            }
        }

        return costMap;
    }
        
    public float getProxCost(Vector2 worldPos){
        Vector3Int cell = tilemap.WorldToCell(worldPos);
        int x = cell.x - bounds.xMin;
        int y = cell.y - bounds.yMin;

        return costMap[x, y];
    }
}


