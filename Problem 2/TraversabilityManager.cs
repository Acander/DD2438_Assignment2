using System;
using UnityEngine;

namespace Scrips
{
    public class TraversabilityManager
    {
        TerrainManager terrain_manager;

        // CS grid and x-axis, z-axis
        public bool[,] traversableCS;
        float x_low;
        float x_high;
        float z_low;
        float z_high;
        float x_range;
        float z_range;
        public int n_x;
        public int n_z;

        // CS grid parameters
        public float grid_x;
        public float grid_z;
        public float grid_hype;

        float car_radius;

        // Traversable point closest to goal_pos
        public Vector3 traversable_goal_pos;

        public TraversabilityManager(TerrainManager terrain_manager, float car_radius)
        {
            this.terrain_manager = terrain_manager;
            this.car_radius = car_radius;

            // Get variables from TerrainManager
            x_low = terrain_manager.myInfo.x_low;
            x_high = terrain_manager.myInfo.x_high;
            z_low = terrain_manager.myInfo.z_low;
            z_high = terrain_manager.myInfo.z_high;
            x_range = x_high - x_low;
            z_range = z_high - z_low;

            // Determine appropriate grid_size
            var x_res = x_range / terrain_manager.myInfo.x_N;
            var z_res = z_range / terrain_manager.myInfo.z_N;
            // We want grid of to be ≈ 3 x 3
            grid_x = determine_grid_size(x_res, 2);
            grid_z = determine_grid_size(z_res, 2);

            // Get grid hypotenuse
            grid_hype = Convert.ToSingle(Math.Sqrt(Math.Pow(grid_x, 2) + Math.Pow(grid_z, 2)));


            // Create traversable points
            n_x = (int)Mathf.Floor(x_range / grid_x);
            n_z = (int)Mathf.Floor(z_range / grid_z);

            // Build CS traversability matrix
            traversableCS = new bool[n_x, n_z];
            for (int i = 0; i < n_x; i++)
            {
                for (int j = 0; j < n_z; j++)
                {
                    float x = get_x_pos(i), z = get_z_pos(j);
                    if (PointTraversableCS(x, z))
                    {
                        traversableCS[i, j] = true;
                    }
                }
            }
        }

        public float determine_grid_size(float res, float desired_grid_size)
        {
            var upper = Mathf.Ceil(res / desired_grid_size);
            var lower = Mathf.Floor(res / desired_grid_size);
            var larger = res / lower;
            var smaller = res / upper;
            var diff_larger = Mathf.Abs(desired_grid_size - larger);
            var diff_smaller = Mathf.Abs(desired_grid_size - smaller);
            return (diff_larger > diff_smaller) ? smaller : larger;
        }

        public Vector3 get_closest_traversable(Vector3 point)
        {
            Vector3 traversable = new Vector3(get_x_pos(0), 0, get_z_pos(0));
            var min_dist = double.MaxValue;
            for (int i = 0; i < n_x; i++)
            {
                for (int j = 0; j < n_z; j++)
                {
                    float x = get_x_pos(i), z = get_z_pos(j);
                    var _point = new Vector3(x, 0, z);
                    var dist = Vector3.Distance(_point, point);
                    if (PointTraversableCS(x, z) && dist < min_dist)
                    {
                        min_dist = dist;
                        traversable = _point;
                    }
                }
            }
            return traversable;
        }

        public float get_x_pos(int i)
        {
            return x_low + (grid_x * i) + (grid_x / 2f);
        }

        public float get_z_pos(int j)
        {
            return z_low + (grid_z * j) + (grid_z / 2f);
        }

        public int get_i_index(float x)
        {
            int index = (int)Mathf.Floor(n_x * (x - x_low) / x_range);
            if (index < 0)
            {
                index = 0;
            }
            else if (index > n_x - 1)
            {
                index = n_x - 1;
            }
            return index;

        }

        public int get_j_index(float z)
        {
            int index = (int)Mathf.Floor(n_z * (z - z_low) / z_range);
            if (index < 0)
            {
                index = 0;
            }
            else if (index > n_z - 1)
            {
                index = n_z - 1;
            }
            return index;
        }

        public bool PointTraversableCS(float x, float z)
        {
            return terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index(x), terrain_manager.myInfo.get_j_index(z)] == 0
                && terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index(x), terrain_manager.myInfo.get_j_index(z + car_radius)] == 0
                && terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index(x + car_radius), terrain_manager.myInfo.get_j_index(z + car_radius)] == 0
                && terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index(x + car_radius), terrain_manager.myInfo.get_j_index(z)] == 0
                && terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index(x + car_radius), terrain_manager.myInfo.get_j_index(z - car_radius)] == 0
                && terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index(x), terrain_manager.myInfo.get_j_index(z - car_radius)] == 0
                && terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index(x - car_radius), terrain_manager.myInfo.get_j_index(z - car_radius)] == 0
                && terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index(x - car_radius), terrain_manager.myInfo.get_j_index(z)] == 0
                && terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index(x - car_radius), terrain_manager.myInfo.get_j_index(z + car_radius)] == 0;
        }
    }
}