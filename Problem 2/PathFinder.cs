using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Scrips
{
     public class PathFinder
    {

        Dictionary<Node, bool> closedSet = new Dictionary<Node, bool>();
        Dictionary<Node, bool> openSet = new Dictionary<Node, bool>();

        //cost of start to this key node
        Dictionary<Node, float> gScore = new Dictionary<Node, float>();
        //cost of start to goal, passing through key node
        Dictionary<Node, double> fScore = new Dictionary<Node, double>();

        Dictionary<Node, Node> nodeLinks = new Dictionary<Node, Node>();

        // Heading resolution
        double heading_resolution = Math.PI / 4;
        int ticks;

        // Traversability manager
        TraversabilityManager traversability;
        double d;

        // Car
        float L;
        float max_steering;
        float dt = Time.fixedDeltaTime;
        float max_v;

        public List<Vector3> FindPath(bool[,] graph, Vector3 s, Vector3 g, TraversabilityManager traversabilityManager, float max_steering, float L, float max_v)
        {
            var start = new Node(new DiscreteConfig(traversabilityManager.get_i_index(s.x), traversabilityManager.get_j_index(s.z), 0), s, 0f, 0f, 0f, new List<Vector3>()); ;
            var goal = new Node(new DiscreteConfig(traversabilityManager.get_i_index(g.x), traversabilityManager.get_j_index(g.z), 0), g, 0f, 0f, 0f, new List<Vector3>());
            
            this.traversability = traversability;
            this.max_steering = max_steering * 0.9f;
            this.L = L;
            this.max_v = max_v;
            d = traversability.grid_hype + 0.01;
            ticks = Convert.ToInt32(Math.Round(2 * Math.PI / heading_resolution));

            openSet[start] = true;
            gScore[start] = 0;
            fScore[start] = Heuristic(start, goal);

            while (openSet.Count > 0)
            {
                var current = nextBest();
                if (current.config.i == goal.config.i && current.config.j == goal.config.j)
                {
                    var path = Reconstruct(current);
                    path.Insert(0, start.fa_pos);
                    return path; 
                }


                openSet.Remove(current);
                closedSet[current] = true;


                foreach (var neighbor in Neighbors(graph, current))
                {
                    if (closedSet.ContainsKey(neighbor))
                        continue;

                    // Penalise changing steering
                    var penalise_steer = Convert.ToSingle(neighbor.steer != 0f) * 10;
                    var projectedG = getGScore(current) + penalise_steer;

                    if (!openSet.ContainsKey(neighbor))
                        openSet[neighbor] = true;
                    else if (projectedG >= getGScore(neighbor))
                        continue;

                    //record it
                    nodeLinks[neighbor] = current;
                    gScore[neighbor] = projectedG;
                    fScore[neighbor] = projectedG + Heuristic(neighbor, goal);

                }
            }


            return new List<Vector3>();
        }

        private double Heuristic(Node start, Node goal)
        {
            var di = goal.config.i - start.config.i;
            var dj = goal.config.j - start.config.j;
            return Math.Sqrt(Math.Pow(di, 2) + Math.Pow(dj, 2));
        }

        private float getGScore(Node pt)
        {
            float score = float.MaxValue;
            gScore.TryGetValue(pt, out score);
            return score;
        }


        private double getFScore(Node pt)
        {
            double score = double.MaxValue;
            fScore.TryGetValue(pt, out score);
            return score;
        }

        public IEnumerable<Node> Neighbors(bool[,] graph, Node node)
        {
            // Controls
            var controls = new List<float>() { -1f, 0f, 1f };
            foreach (var steer in controls)
            {
                // Move in trajectory using rear axel mid
                var v = max_v;
                var phi = steer * max_steering;
                var theta = node.theta;
                var fa_pos = node.fa_pos;
                var center_start_pos = fa_pos - Quaternion.Euler(0, theta, 0) * Vector3.forward * (L / 2);
                var center_pos = center_start_pos;
                var ra_start_pos = center_pos - Quaternion.Euler(0, theta, 0) * Vector3.forward * (L / 2);
                var ra_pos = ra_start_pos;
                var trajectory = new List<Vector3>();
                while (Vector3.Distance(center_start_pos, center_pos) < d)
                {
                    var dx = dt * v * Mathf.Sin(theta);
                    var dz = dt * v * Mathf.Cos(theta);
                    var dTheta = dt * (v / L) * Mathf.Tan(phi);
                    ra_pos += new Vector3(dx, 0, dz);
                    center_pos = ra_pos + Quaternion.Euler(0, theta, 0) * Vector3.forward * (L / 2);
                    fa_pos = center_pos + Quaternion.Euler(0, theta, 0) * Vector3.forward * (L / 2);
                    theta += dTheta;
                    // Log front axel position in trajectory
                    trajectory.Add(fa_pos);
                }
                // Check traversability using center pos
                var i = traversability.get_i_index(center_pos.x);
                var j = traversability.get_j_index(center_pos.z);
                var tick = mod(Convert.ToInt32(theta / heading_resolution), ticks);
                // Log front axel position
                var new_pt = new Node(new DiscreteConfig(i, j, tick), fa_pos, theta, v, steer, trajectory);
                if (IsValidNeighbor(graph, new_pt))
                {
                    yield return new_pt;
                }
            }
        }

        int mod(int x, int m)
        {
            int r = x % m;
            return r < 0 ? r + m : r;
        }

        public bool IsValidNeighbor(bool[,] matrix, Node pt)
        {
            int i = pt.config.i;
            int j = pt.config.j;
            if (i < 0 || i >= matrix.GetLength(1))
                return false;

            if (j < 0 || j >= matrix.GetLength(0))
                return false;

            return matrix[i, j];

        }

        private List<Vector3> Reconstruct(Node current)
        {
            List<Vector3> path = new List<Vector3>();
            while (nodeLinks.ContainsKey(current))
            {
                path.Add(current.fa_pos);
                current = nodeLinks[current];
            }

            path.Reverse();
            return path;
        }

        private Node nextBest()
        {
            double best = double.MaxValue;
            Node bestPt = openSet.Keys.First();
            foreach (var node in openSet.Keys)
            {
                var score = getFScore(node);
                if (score < best)
                {
                    bestPt = node;
                    best = score;
                }
            }

            return bestPt;
        }

    }
}