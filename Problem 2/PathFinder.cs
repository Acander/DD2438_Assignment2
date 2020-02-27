using System;
using System.Collections.Generic;
using System.Linq;
using System.Runtime.CompilerServices;
using UnityEngine;

namespace Scrips
{
     public class PathFinder
     {
         private Dictionary<Node, bool> closedSet;
         private Dictionary<Node, bool> openSet;

        //cost of start to this key node
         private Dictionary<Node, float> gScore;
        //cost of start to goal, passing through key node
         private Dictionary<Node, double> fScore;

         private Dictionary<Node, Node> nodeLinks;

        // Heading resolution
        double heading_resolution = Math.PI / 4;

         // Traversability manager
        TraversabilityManager traversability;

        public List<Vector3> FindPath(bool[,] graph, Vector3 s, Vector3 g, TraversabilityManager traversabilityManager, float max_steering, float L, float max_v)
        {
            closedSet = new Dictionary<Node, bool>();
            openSet = new Dictionary<Node, bool>();

            //cost of start to this key node
            gScore = new Dictionary<Node, float>();
            //cost of start to goal, passing through key node
            fScore = new Dictionary<Node, double>();

            nodeLinks = new Dictionary<Node, Node>();
            
            var start = new Node(new DiscreteConfig(traversabilityManager.get_i_index(s.x), traversabilityManager.get_j_index(s.z))); 
            var goal = new Node(new DiscreteConfig(traversabilityManager.get_i_index(g.x), traversabilityManager.get_j_index(g.z)));
            
            traversability = traversabilityManager;

            openSet[start] = true;
            gScore[start] = 0;
            fScore[start] = Heuristic(start, goal);

            while (openSet.Count > 0)
            {
                var current = nextBest();
                if (current.config.i == goal.config.i && current.config.j == goal.config.j)
                {
                    var path = Reconstruct(current);
                    path.Insert(0, s);
                    return path; 
                }


                openSet.Remove(current);
                closedSet[current] = true;


                foreach (var neighbor in Neighbors(current))
                {
                    if (closedSet.ContainsKey(neighbor))
                        continue;

                    var projectedG = getGScore(current);

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


         public IEnumerable<Node> Neighbors(Node node)
         {
             List<Node> neighbours = new List<Node>();
             neighbours.Add(new Node(new DiscreteConfig(node.config.i, node.config.j + 1)));
             neighbours.Add(new Node(new DiscreteConfig(node.config.i + 1, node.config.j + 1)));
             neighbours.Add(new Node(new DiscreteConfig(node.config.i + 1, node.config.j)));
             neighbours.Add(new Node(new DiscreteConfig(node.config.i + 1, node.config.j - 1)));
             neighbours.Add(new Node(new DiscreteConfig(node.config.i, node.config.j - 1)));
             neighbours.Add(new Node(new DiscreteConfig(node.config.i - 1, node.config.j - 1)));
             neighbours.Add(new Node(new DiscreteConfig(node.config.i - 1, node.config.j)));
             neighbours.Add(new Node(new DiscreteConfig(node.config.i - 1, node.config.j + 1)));
             for (int i = 0; i < neighbours.Count; i++)
             {
                 if (!IsValidNeighbor(neighbours[i]))
                 {
                     Debug.Log(
                         "---------------------Invalid Neighbour!!!-----------------------------------------------");
                     neighbours.RemoveAt(i);
                     i--;
                 }
             }
             Debug.LogFormat("Length of neighbours: {0}", neighbours.Count);

    return neighbours;
         }
         
         public bool IsValidNeighbor(Node pt)
         {
             /*float x_pos = traversability.get_x_pos(pt.config.i);
             float z_pos = traversability.get_z_pos(pt.config.j);*/
             return traversability.traversableCS[pt.config.i, pt.config.j];
         }

        private List<Vector3> Reconstruct(Node current)
        {
            List<Vector3> path = new List<Vector3>();
            while (nodeLinks.ContainsKey(current))
            {
                float x_pos = traversability.get_x_pos(current.config.i);
                float z_pos = traversability.get_z_pos(current.config.j);
                path.Add(new Vector3(x_pos, 0f, z_pos));
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