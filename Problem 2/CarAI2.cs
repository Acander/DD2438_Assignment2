using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI2 : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        public GameObject[] friends;
        public GameObject[] enemies;

        private void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            // note that both arrays will have holes when objects are destroyed
            // but for initial planning they should work
            friends = GameObject.FindGameObjectsWithTag("Player");
            // Note that you are not allowed to check the positions of the turrets in this problem

            // Plan your path here

            // get size of the terrian 
            int xn = terrain_manager.myInfo.x_N;
            int zn = terrain_manager.myInfo.z_N;

            //Init Data Structures
            int[][] map = new int[zn][];
            int[][] occupancy_map = new int[zn][];
            List<Tuple<int, int, int, int>> convex_cover_boundary = new List<Tuple<int, int, int, int>>();

            createNewIntGridMap(xn, zn, map, occupancy_map);
            printGridMap(xn, zn, map);
            storeTheFourBoundriesOfAllConvexCover(occupancy_map, map, convex_cover_boundary);
            checkThatConvexCoversAreNotPartOfObstacles(convex_cover_boundary, map);
            drawConvexSet(convex_cover_boundary, zn);
        }

        private void createNewIntGridMap(int xn, int zn, int[][] map, int[][] occupancy_map)
        {
            for (int i = 0; i < zn; i++)
            {
                map[zn - 1 - i] = new int[xn];
                occupancy_map[zn - 1 - i] = new int[xn];
                for (int j = 0; j < xn; j++)
                {
                    map[zn - 1 - i][j] = (int) terrain_manager.myInfo.traversability[j, i];
                    occupancy_map[zn - 1 - i][j] = (int) terrain_manager.myInfo.traversability[j, i];
                }
            }
        }


        private void printGridMap(int xn, int zn, int[][] map)
        {
            int free_spaces = 0;
            Debug.Log("original map");
            string original_map = "";
            for (int i = 0; i < zn; i++)
            {
                for (int j = 0; j < xn; j++)
                {
                    original_map += map[i][j].ToString() + ",";
                    if (map[i][j] == 0)
                    {
                        free_spaces += 1;
                    }
                }
                original_map += "\n";
            }

            Debug.Log(original_map);
            Debug.LogFormat("total free spaces: {0}", free_spaces);
        }

        private void storeTheFourBoundriesOfAllConvexCover(int[][] occupancy_map, int[][] map,
            List<Tuple<int, int, int, int>> convex_cover_boundary)
        {
            // store the four boundaries for convex covers
            // up, down, left, right
            int n_cover = 2;
            while (!check_free_spaces(occupancy_map))
            {
                Debug.LogFormat("current convex cover index is {0}", n_cover);
                List<Tuple<int, int>> free_ones = get_free_ij_pair(occupancy_map);
                int rnd_free_p_index = UnityEngine.Random.Range(0, free_ones.Count);
                int p_i = free_ones[rnd_free_p_index].Item1;
                int p_j = free_ones[rnd_free_p_index].Item2;
                int up = p_i;
                int down = p_i;
                int left = p_j;
                int right = p_j;
                while (true)
                {
                    var around = look_around(map, up, down, left, right);
                    if ((around.Item1 != 0) && (around.Item2 != 0) && (around.Item3 != 0) && (around.Item4 != 0))
                    {
                        print_update_map(map, occupancy_map, up, down, left, right, n_cover);
                        break;
                    }
                    bool changed = false;
                    if (around.Item1 == 0 && !changed)
                    {
                        changed = true;
                        up -= 1;
                    }
                    if (around.Item2 == 0 && !changed)
                    {
                        changed = true;
                        down += 1;
                    }
                    if (around.Item3 == 0 && !changed)
                    {
                        changed = true;
                        left -= 1;
                    }
                    if (around.Item4 == 0 && !changed)
                    {
                        changed = true;
                        right += 1;
                    }
                }
                Debug.LogFormat("current convex cover boundaries, up:{0}, down:{1}, left:{2}, right:{3}", up, down, left,
                    right);
                convex_cover_boundary.Add(Tuple.Create(up, down, left, right));
                n_cover += 1;
            }
        }

        public static bool check_free_spaces(int[][] occupancy_map)
        {
            // return true if no free spaces
            // return false if still free spaces 
            int zn = occupancy_map.Length;
            int xn = occupancy_map[0].Length;
            int sum = 0;
            for (int i = 0; i < zn; i++)
            {
                for (int j = 0; j < xn; j++)
                {
                    sum += occupancy_map[i][j];
                }
            }
            return sum == zn * xn;
        }

        public static Tuple<int, int, int, int> look_around(int[][] map, int up, int down, int left, int right)
        {
            int up_sum = 0;
            for (int j = left; j <= right; j++)
            {
                up_sum += map[up - 1][j];
            }
            int down_sum = 0;
            for (int j = left; j <= right; j++)
            {
                down_sum += map[down + 1][j];
            }
            int left_sum = 0;
            for (int i = up; i <= down; i++)
            {
                left_sum += map[i][left - 1];
            }
            int right_sum = 0;
            for (int i = up; i <= down; i++)
            {
                right_sum += map[i][right + 1];
            }
            return Tuple.Create(up_sum, down_sum, left_sum, right_sum);
        }

        public static List<Tuple<int, int>> get_free_ij_pair(int[][] map)
        {
            List<Tuple<int, int>> result = new List<Tuple<int, int>>();
            int zn = map.Length;
            int xn = map[0].Length;
            for (int i = 0; i < zn; i++)
            {
                for (int j = 0; j < xn; j++)
                {
                    if (map[i][j] == 0)
                    {
                        result.Add(Tuple.Create(i, j));
                    }
                }
            }
            return result;
        }

        public static void print_update_map(int[][] map, int[][] occupancy_map, int up, int down, int left, int right,
            int value)
        {
            int zn = map.Length;
            int xn = map[0].Length;
            string update_map = "";
            for (int i = 0; i < zn; i++)
            {
                for (int j = 0; j < xn; j++)
                {
                    if (i >= up && i <= down && j >= left && j <= right)
                    {
                        update_map += value.ToString() + ",";
                        occupancy_map[i][j] = 1;
                    }
                    else
                    {
                        update_map += map[i][j].ToString() + ",";
                    }
                }
                update_map += "\n";
            }
            Debug.Log(update_map);
        }

        private void checkThatConvexCoversAreNotPartOfObstacles(List<Tuple<int, int, int, int>> convex_cover_boundary,
            int[][] map)
        {
            Debug.LogFormat("convex cover boundary has size:{0}", convex_cover_boundary.Count);
            for (int i = 0; i < convex_cover_boundary.Count; i++)
            {
                var hehe = convex_cover_boundary[i];
                if (map[hehe.Item1][hehe.Item3] == 1 || map[hehe.Item1][hehe.Item4] == 1 ||
                    map[hehe.Item2][hehe.Item3] == 1 || map[hehe.Item2][hehe.Item4] == 1)
                {
                    Debug.LogFormat("error with index:{0}", i);
                }
            }
        }

        private void drawConvexSet(List<Tuple<int, int, int, int>> convex_cover_boundary, int zn)
        {
            float x_step = (terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low) / terrain_manager.myInfo.x_N;
            float z_step = (terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low) / terrain_manager.myInfo.z_N;
            for (int i = 0; i < convex_cover_boundary.Count; i++)
            {
                var hehe = convex_cover_boundary[i];
                var color_list = new List<Color> {Color.black, Color.cyan, Color.red, Color.yellow, Color.white};
                int random_index = UnityEngine.Random.Range(0, color_list.Count);
                Vector3 up_left = new Vector3(terrain_manager.myInfo.get_x_pos(hehe.Item3) - x_step / 2, 0f,
                    terrain_manager.myInfo.get_x_pos(zn - 1 - hehe.Item1) + z_step / 2);
                Vector3 up_right = new Vector3(terrain_manager.myInfo.get_x_pos(hehe.Item4) + x_step / 2, 0f,
                    terrain_manager.myInfo.get_x_pos(zn - 1 - hehe.Item1) + z_step / 2);
                Vector3 down_left = new Vector3(terrain_manager.myInfo.get_x_pos(hehe.Item3) - x_step / 2, 0f,
                    terrain_manager.myInfo.get_x_pos(zn - 1 - hehe.Item2) - z_step / 2);
                Vector3 down_right = new Vector3(terrain_manager.myInfo.get_x_pos(hehe.Item4) + x_step / 2, 0f,
                    terrain_manager.myInfo.get_x_pos(zn - 1 - hehe.Item2) - z_step / 2);
                Debug.DrawLine(up_left, up_right, color_list[random_index], 100f);
                Debug.DrawLine(up_right, down_right, color_list[random_index], 100f);
                Debug.DrawLine(down_right, down_left, color_list[random_index], 100f);
                Debug.DrawLine(down_left, up_left, color_list[random_index], 100f);
            }
        }


        private void FixedUpdate()
        {
            // Execute your path here
            // ...


            // this is how you access information about the terrain
            int i = terrain_manager.myInfo.get_i_index(transform.position.x);
            int j = terrain_manager.myInfo.get_j_index(transform.position.z);
            float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
            float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

            Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z));
            // this is how you control the car
            //m_Car.Move(0f, -1f, 1f, 0f);
        }
    }
}