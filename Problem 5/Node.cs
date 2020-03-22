using System;
using System.Collections.Generic;
using UnityEngine;

namespace Scrips
{
    public class Node
    {
        // Discrete configuration
        public DiscreteConfig config;

        // Continuous configuration
        // Car front axel center position
        //public Vector3 fa_pos;
        //public float theta, v;

        // Controls
        //public float steer;

        //public List<Vector3> trajectory;

        public Node(DiscreteConfig config)
        {
            this.config = config;
        }

        public override int GetHashCode()
        {
            return config.GetHashCode();
        }

        public override bool Equals(object obj)
        {
            Node other = obj as Node;
            return other != null && other.config.Equals(config);
        }

        /*public override string ToString()
        {
            return String.Format("(Node: (Discrete Config: {0}, {1}, {2}) (Continuous Config: {3}, {4}, {5}), (Control: {6}))", config.i, config.j, config.tick, fa_pos.ToString(), theta, v, steer);
        }*/
    }

    public struct DiscreteConfig
    {
        public int i, j;
        public DiscreteConfig(int i, int j) { this.i = i; this.j = j;}
    }
}
