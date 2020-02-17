using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using KruskalMinimumSpanningTree;
using System.Linq;

namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI1 : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use
        
        private PIDController pid_controller; // Used for making course corrections
        // Car information
        float car_radius = 3f;
        float L = 2.870426f;
        float L_f;
        float L_b;
        float max_throttle = 1f;
        float max_v = 8;
        Vector3 fa_mid;
        Vector3 ra_mid;
        
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

            int position_i = terrain_manager.myInfo.get_i_index(transform.position.x);
            int position_j = terrain_manager.myInfo.get_j_index(transform.position.z);
            float grid_center_x = terrain_manager.myInfo.get_x_pos(position_i);
            float grid_center_z = terrain_manager.myInfo.get_z_pos(position_j);

            Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z), Color.white, 100f);


            // Plan your path here
            // ...
            int xn = terrain_manager.myInfo.x_N;
            int zn = terrain_manager.myInfo.z_N;
            int[][] gridmap = new int[zn][];
            for (int i = 0; i < zn; i++)
            {
                gridmap[zn - 1 - i] = new int[xn];
                for (int j = 0; j < xn; j++)
                {
                    gridmap[zn - 1 - i][j] = (int)terrain_manager.myInfo.traversability[j, i];
                }
            }

            // below can be used to visualize the grip map: 1 means obstacle, 0 means no obstacle
            string lala = "";
            for (int i = 0; i < zn; i++)
            {
                for (int j = 0; j < xn; j++)
                {
                    if(position_j==zn-i-1 && position_i == j)
                    {
                        // the current position is red 
                        lala += "<color=#800000ff>"+ gridmap[i][j].ToString() + "</color>" + ",";
                    }
                    else
                    {
                        lala += gridmap[i][j].ToString() + ",";
                    }
                }
                lala += "\n";
            }
            Debug.Log(lala);

            int[,] vertex_mapping = new int[zn, xn];
            int sum = 0;
            for (int i = 0; i < zn; i++)
            {
                for (int j = 0; j < xn; j++)
                {
                    if (gridmap[i][j] == 0)
                    {
                        sum += 1;
                        vertex_mapping[i, j] = sum;
                    }
                }
            }
            Debug.LogFormat("total free spaces: {0}",sum);
            //all edges
            List<Edge> edges = new List<Edge>();
            for (int i = 0; i < zn; i++)
            {
                for (int j = 0; j < xn; j++)
                {
                    if (vertex_mapping[i, j] != 0)
                    {
                        // look right
                        if (vertex_mapping[i, j + 1] != 0)
                        {
                            edges.Add(new Edge() { Vertex1 = vertex_mapping[i, j], Vertex2 = vertex_mapping[i, j + 1], Weight = 1 });
                        }
                        // look down 
                        if (vertex_mapping[i + 1, j] != 0)
                        {
                            edges.Add(new Edge() { Vertex1 = vertex_mapping[i, j], Vertex2 = vertex_mapping[i + 1, j], Weight = 1 });
                        }
                    }
                }
            }

            //set of vertices
            List<int> vertices = new List<int>();
            for (int i = 0; i < sum; i++)
            {
                vertices.Add(i + 1);
            }

            // get minimum spanning tree
            List<Edge> MinimumSpanningTree = Kruskals_MST(edges, vertices);
            int total_weight = 0;
            foreach(Edge edge in MinimumSpanningTree)
            {
                total_weight += 1;
                int v1_i = CoordinatesOf<int>(vertex_mapping, edge.Vertex1).Item1;
                int v1_j = CoordinatesOf<int>(vertex_mapping, edge.Vertex1).Item2;
                int v2_i = CoordinatesOf<int>(vertex_mapping, edge.Vertex2).Item1;
                int v2_j = CoordinatesOf<int>(vertex_mapping, edge.Vertex2).Item2;
                float v1_x = terrain_manager.myInfo.get_x_pos(v1_j);
                float v1_z = terrain_manager.myInfo.get_z_pos(zn-1-v1_i);
                float v2_x = terrain_manager.myInfo.get_x_pos(v2_j);
                float v2_z = terrain_manager.myInfo.get_z_pos(zn-1-v2_i);
                Debug.DrawLine(new Vector3(v1_x, 0f, v1_z), new Vector3(v2_x, 0f, v2_z), Color.blue, 100f);
                //Debug.LogFormat("Vertex ({0},{1}) to Vertex ({2},{3}) weight is: {4}", v1_i, v1_j, v2_i, v2_j, edge.Weight);
            }
            Debug.LogFormat("minimum spanning tree weight: {0}", total_weight);

            

            int[,] final_mapping = new int[3 * zn, 3 * xn];
            for (int i = 0; i < zn; i++)
            {
                for (int j = 0; j < xn; j++)
                {
                    if (gridmap[i][j] == 1)
                    {
                        for (int m = 3 * i; m <= 3 * i + 2; m++)
                        {
                            for (int n = 3 * j; n <= 3 * j + 2; n++)
                            {
                                final_mapping[m, n] = 1;
                            }
                        }
                    }
                    else
                    {
                        final_mapping[3 * i + 1, 3 * j + 1] = 1;
                        final_mapping[3 * i, 3 * j] = 0;
                        final_mapping[3 * i, 3 * j + 2] = 0;
                        final_mapping[3 * i + 2, 3 * j] = 0;
                        final_mapping[3 * i + 2, 3 * j + 2] = 0;
                        var hoho = judgement(MinimumSpanningTree, vertex_mapping, i, j);
                        final_mapping[3 * i, 3 * j + 1] = hoho.Item1; //up
                        final_mapping[3 * i + 2, 3 * j + 1] = hoho.Item2; //down
                        final_mapping[3 * i + 1, 3 * j] = hoho.Item3; //left
                        final_mapping[3 * i + 1, 3 * j + 2] = hoho.Item4; //right
                    }
                }
            }

            int total_step = 0;
            for (int i = 0; i < 3 * zn; i++)
            {
                for(int j = 0; j < 3 * xn; j++)
                {
                    if (final_mapping[i, j] == 0)
                    {
                        total_step += 1;
                    }
                }
            }
            Debug.LogFormat("total step is {0}", total_step);
            List<Vector3> final_path = new List<Vector3>();
            print_map(final_mapping, zn, xn);


            int start_i = 3*(zn-1-position_j);
            int start_j = 3*(position_i);
            if(transform.position.x < grid_center_x && transform.position.z < grid_center_z)
            {
                start_i += 1;
                start_j -= 1;
            }
            if (transform.position.x < grid_center_x && transform.position.z > grid_center_z)
            {
                start_i -= 1;
                start_j -= 1;
            }
            if (transform.position.x > grid_center_x && transform.position.z < grid_center_z)
            {
                start_i += 1;
                start_j += 1;
            }
            if (transform.position.x > grid_center_x && transform.position.z > grid_center_z)
            {
                start_i -= 1;
                start_j += 1;
            }
            int step = 2;
            int[,] trajectory = new int[3 * zn, 3 * xn];
            for (int i = 0; i < 3 * zn; i++)
            {
                for (int j = 0; j < 3 * xn; j++)
                {
                    if (i == start_i && j == start_j)
                    {
                        trajectory[i, j] = step;
                    }
                    else
                    {
                        trajectory[i, j] = final_mapping[i, j];
                    }
                }
            }
            print_map(trajectory, zn, xn);

            int current_i = start_i + 1;
            int current_j = start_j;
            Debug.LogFormat("start index: ({0}, {1})", start_i, start_j);
            Debug.LogFormat("next index: ({0}, {1})", current_i, current_j);

            int old_i = 0;
            int old_j = 0;
            while (true)
            {
                bool changed = false;
                step += 1;
                if (at_corner_or_not(current_i, current_j))
                {
                    old_i = current_i;
                    old_j = current_j;
                }
                
                trajectory[current_i, current_j] = step;
                //Debug.LogFormat("trajectory with step: {0}", step);

                int up = trajectory[current_i - 1, current_j] == 0 ? 1 : 0;
                int down = trajectory[current_i + 1, current_j] == 0 ? 1 : 0;
                int left = trajectory[current_i, current_j - 1] == 0 ? 1 : 0;
                int right = trajectory[current_i, current_j + 1] == 0 ? 1 : 0;
                if (up + down + left + right == 0)
                {
                    break;
                }
                if (up + down + left + right == 1)
                {
                    if (up == 1)
                    {
                        current_i -= 1;
                        changed = true;
                    }
                    if (down == 1)
                    {
                        changed = true;
                        current_i += 1;
                    }
                    if (left == 1)
                    {
                        changed = true;
                        current_j -= 1;
                    }
                    if (right == 1)
                    {
                        changed = true;
                        current_j += 1;
                    }
                }
                else
                {
                    var hehe = judgement(MinimumSpanningTree, vertex_mapping, current_i / 3, current_j / 3);
                    if (up == 1 && hehe.Item1 == 1)
                    {
                        changed = true;
                        current_i -= 1;
                    }
                    if (down == 1 && hehe.Item2 == 1)
                    {
                        changed = true;
                        current_i += 1;
                    }
                    if (left == 1 && hehe.Item3 == 1)
                    {
                        changed = true;
                        current_j -= 1;
                    }
                    if (right == 1 && hehe.Item4 == 1)
                    {
                        changed = true;
                        current_j += 1;
                    }
                }
                if (!changed)
                {
                    if (up == 1 && (current_i - 1) / 3 == current_i / 3)
                    {
                        changed = true;
                        current_i -= 1;
                    }
                    if (down == 1 && (current_i + 1) / 3 == current_i / 3)
                    {
                        changed = true;
                        current_i += 1;
                    }
                    if (left == 1 && (current_j - 1) / 3 == current_j / 3)
                    {
                        changed = true;
                        current_j -= 1;
                    }
                    if (right == 1 && (current_j + 1) / 3 == current_j / 3)
                    {
                        changed = true;
                        current_j += 1;
                    }
                }
                //Debug.LogFormat("changed: {0}", changed);
                if (!changed)
                {
                    break;
                }
                Vector3 old_p = get_position_in_map_from_ij(old_i, old_j, zn);
                if (at_corner_or_not(current_i, current_j) && step != total_step-1)
                {
                    Vector3 new_p = get_position_in_map_from_ij(current_i, current_j, zn);
                    Debug.DrawLine(old_p, new_p, Color.red, 100f);
                }
                final_path.Add(old_p);
            }
            Debug.Log("trajectory");
            print_map(trajectory, zn, xn);
            Debug.LogFormat("final path length: {0}", final_path.Count);
            
            var theta = transform.eulerAngles.y;
            fa_mid = transform.position + Quaternion.Euler(0, theta, 0) * Vector3.forward * (L / 2);
            // Start on ground
            fa_mid.y = 0;
            pid_controller = new PIDController(final_path, m_Car.m_MaximumSteerAngle);
        }

        public static bool at_corner_or_not(int i,int j)
        {
            if (i%3 == 1 || j%3 == 1)
            {
                return false;
            }
            else
            {
                return true;
            }
        }

        public Vector3 get_position_in_map_from_ij(int i,int j,int zn)
        {
            Vector3 result = new Vector3();
            int map_i = j / 3;
            int map_j = zn - 1 - i / 3;
            float grid_center_x = terrain_manager.myInfo.get_x_pos(map_i);
            float grid_center_z = terrain_manager.myInfo.get_z_pos(map_j);
            int top_left_i = 3 * (zn - 1 - map_j);
            int top_left_j = 3 * map_i;
            float x_step = (terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low) / terrain_manager.myInfo.x_N;
            float z_step = (terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low) / terrain_manager.myInfo.z_N;
            

            if (top_left_i == i && top_left_j == j)
            {
                // top left
                result.Set(grid_center_x - x_step / 4, 0f, grid_center_z + z_step / 4);
            }
            if (top_left_i+2 == i && top_left_j == j)
            {
                // down left
                result.Set(grid_center_x - x_step / 4, 0f, grid_center_z - z_step / 4);
            }
            if (top_left_i == i && top_left_j+2 == j)
            {
                // top right
                result.Set(grid_center_x + x_step / 4, 0f, grid_center_z + z_step / 4);
            }
            if (top_left_i+2 == i && top_left_j+2 == j)
            {
                // down right
                result.Set(grid_center_x + x_step / 4, 0f, grid_center_z - z_step / 4);
            }

            return result;
        }

        public static void print_map(int [,] map, int zn,int xn)
        {
            string output = "";
            for (int i = 0; i < 3 * zn; i++)
            {
                for (int j = 0; j < 3 * xn; j++)
                {
                    output += map[i, j].ToString()+" ";
                }
                output += "\n";
            }
            Debug.Log(output);
        }

        public static Tuple<int, int, int, int> judgement(List<Edge> tree, int[,] map, int i, int j)
        {
            int up = 0, down = 0, left = 0, right = 0;
            foreach (Edge edge in tree)
            {
                int v1_i = CoordinatesOf<int>(map, edge.Vertex1).Item1;
                int v1_j = CoordinatesOf<int>(map, edge.Vertex1).Item2;
                int v2_i = CoordinatesOf<int>(map, edge.Vertex2).Item1;
                int v2_j = CoordinatesOf<int>(map, edge.Vertex2).Item2;

                if ((i == v1_i && j == v1_j && i - 1 == v2_i && j == v2_j) || (i - 1 == v1_i && j == v1_j && i == v2_i && j == v2_j))
                {
                    up = 1;
                    break;
                }
            }

            foreach (Edge edge in tree)
            {
                int v1_i = CoordinatesOf<int>(map, edge.Vertex1).Item1;
                int v1_j = CoordinatesOf<int>(map, edge.Vertex1).Item2;
                int v2_i = CoordinatesOf<int>(map, edge.Vertex2).Item1;
                int v2_j = CoordinatesOf<int>(map, edge.Vertex2).Item2;

                if ((i == v1_i && j == v1_j && i + 1 == v2_i && j == v2_j) || (i + 1 == v1_i && j == v1_j && i == v2_i && j == v2_j))
                {
                    down = 1;
                    break;
                }
            }

            foreach (Edge edge in tree)
            {
                int v1_i = CoordinatesOf<int>(map, edge.Vertex1).Item1;
                int v1_j = CoordinatesOf<int>(map, edge.Vertex1).Item2;
                int v2_i = CoordinatesOf<int>(map, edge.Vertex2).Item1;
                int v2_j = CoordinatesOf<int>(map, edge.Vertex2).Item2;

                if ((i == v1_i && j == v1_j && i == v2_i && j - 1 == v2_j) || (i == v1_i && j - 1 == v1_j && i == v2_i && j == v2_j))
                {
                    left = 1;
                    break;
                }
            }

            foreach (Edge edge in tree)
            {
                int v1_i = CoordinatesOf<int>(map, edge.Vertex1).Item1;
                int v1_j = CoordinatesOf<int>(map, edge.Vertex1).Item2;
                int v2_i = CoordinatesOf<int>(map, edge.Vertex2).Item1;
                int v2_j = CoordinatesOf<int>(map, edge.Vertex2).Item2;

                if ((i == v1_i && j == v1_j && i == v2_i && j + 1 == v2_j) || (i == v1_i && j + 1 == v1_j && i == v2_i && j == v2_j))
                {
                    right = 1;
                    break;
                }
            }
            return Tuple.Create(up, down, left, right);
        }


        public static Tuple<int, int> CoordinatesOf<T>(T[,] matrix, T value)
        {
            int w = matrix.GetLength(0); // width
            int h = matrix.GetLength(1); // height

            for (int x = 0; x < w; ++x)
            {
                for (int y = 0; y < h; ++y)
                {
                    if (matrix[x, y].Equals(value))
                        return Tuple.Create(x, y);
                }
            }

            return Tuple.Create(-1, -1);
        }

        static List<Edge> Kruskals_MST(List<Edge> edges, List<int> vertices)
        {
            //empty result list
            List<Edge> result = new List<Edge>();

            //making set
            DisjointSet.Set set = new DisjointSet.Set(1000);
            foreach (int vertex in vertices)
                set.MakeSet(vertex);

            //sorting the edges order by weight ascending
            var sortedEdge = edges.OrderBy(x => x.Weight).ToList();

            foreach (Edge edge in sortedEdge)
            {
                //adding edge to result if both vertices do not belong to same set
                //both vertices in same set means it can have cycles in tree
                if (set.FindSet(edge.Vertex1) != set.FindSet(edge.Vertex2))
                {
                    result.Add(edge);
                    set.Union(edge.Vertex1, edge.Vertex2);
                }
            }
            return result;
        }

        private void FixedUpdate()
        {
            /* // friends.Length is the number of friends
            // GameObject friend in friends --> friend.transform.position --> get positions of friends 

            enemies = GameObject.FindGameObjectsWithTag("Enemy");

            // Execute your path here
            // ...
            

            // this is how you access information about the terrain
            int i = terrain_manager.myInfo.get_i_index(transform.position.x);
            int j = terrain_manager.myInfo.get_j_index(transform.position.z);
            float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
            float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

            Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z));


            // this is how you control the car
            //Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
            //m_Car.Move(0f, 0.5f, 0f, 0f);
            */

            /*
            // friends.Length is the number of friends
            // GameObject friend in friends --> friend.transform.position --> get positions of friends 

            enemies = GameObject.FindGameObjectsWithTag("Enemy");

            // Execute your path here
            // ...
            

            // this is how you access information about the terrain
            int i = terrain_manager.myInfo.get_i_index(transform.position.x);
            int j = terrain_manager.myInfo.get_j_index(transform.position.z);
            float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
            float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

            Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z));


            // this is how you control the car
            //Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
            //m_Car.Move(0f, 0.5f, 0f, 0f);
            */
            var theta = transform.eulerAngles.y;
            fa_mid = transform.position + Quaternion.Euler(0, theta, 0) * Vector3.forward * (L / 2);
            ra_mid = transform.position - Quaternion.Euler(0, theta, 0) * Vector3.forward * (L / 2);
            var act_fa_mid = m_Car.m_WheelMeshes[0].transform.position + (m_Car.m_WheelMeshes[1].transform.position - m_Car.m_WheelMeshes[0].transform.position) / 2;
            var act_ra_mid = m_Car.m_WheelMeshes[2].transform.position + (m_Car.m_WheelMeshes[3].transform.position - m_Car.m_WheelMeshes[2].transform.position) / 2;

            
            // Draw line from front axel mid to target
            var target = my_path[pid_controller.current];
            var target_vec = target.fa_pos;
            UnityEngine.Debug.DrawLine(fa_mid, target_vec, Color.blue);

            // Draw CTE line
            var progress = pid_controller.get_progress_point(fa_mid);
            UnityEngine.Debug.DrawLine(fa_mid, progress, Color.red);
            
            throttle = 0.5f
            /*var throttle = max_throttle;
            var target_vel = target.v;
            var velocity = GetComponent<Rigidbody>().velocity.magnitude;
            if (velocity > target_vel)
            {
                throttle = 0;
            }*/

            var steer = pid_controller.get_controls(fa_mid, transform.right);
            //UnityEngine.Debug.Log("steer = " + steer);
            m_Car.Move(steer, throttle, 0f, 0f);
        }
    }
}
