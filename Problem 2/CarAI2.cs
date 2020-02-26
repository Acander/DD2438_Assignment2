using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Diagnostics;
using System.Linq;
using KruskalMinimumSpanningTree;
using Scrips;
using Unity.Collections.LowLevel.Unsafe;
using Debug = UnityEngine.Debug;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI2 : MonoBehaviour
    {
        public CarController m_Car; // the car controller we want to use
        /*private CarController m_Car2;
        private CarController m_Car3;*/

        public PIDController pid_controller; // Used for making course corrections
        /*private PIDController pid_controller2;
        private PIDController pid_controller3;*/
        // Car information
        private float car_radius = 5f;
        private float L = 2.870426f;
        private float L_f;
        private float L_b;
        private float max_throttle = 1f;
        private float max_v = 8;
        private Vector3 fa_mid;
        private Vector3 ra_mid;

        private Boolean collision = false;
        /*private Boolean collision2 = false;
        private Boolean collision3 = false;
        
        Stopwatch stopwatch1 = new Stopwatch();*/
        private Stopwatch stopwatch = new Stopwatch();

        List<Vector3> convexPoints = new List<Vector3>();
        List<Vector3> tspPath = new List<Vector3>();
        public List<Vector3> finalPath = new List<Vector3>();
        //public List<Vector3> final_pathB = new List<Vector3>();
        //public List<Vector3> final_pathC = new List<Vector3>();
        
        //To do with planning final path
        private PathFinder pathFinder = new PathFinder();
        private TraversabilityManager traversabilityManager;

        public GameObject terrain_manager_game_object;
        public TerrainManager terrain_manager;

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
            
            //1) Creating a complete convex set________________________________________________________________________
            //Init Data Structures
            int[][] map = new int[zn][];
            int[][] occupancy_map = new int[zn][];
            List<Tuple<int, int, int, int>> convex_cover_boundary = new List<Tuple<int, int, int, int>>();
            
            createNewIntGridMap(xn, zn, map, occupancy_map);
            printGridMap(xn, zn, map);
            storeTheFourBoundriesOfAllConvexCover(occupancy_map, map, convex_cover_boundary);
            checkThatConvexCoversAreNotPartOfObstacles(convex_cover_boundary, map);
            drawConvexSet(convex_cover_boundary, zn);
            
            
            //2) Create a set of minimum points that give maximum exposition___________________________________________
            //Init variables
            float xSteps = terrain_manager.myInfo.x_high / 3;
            float minimumX;

            minimumX = selectMinimumX(xSteps);
            selectPointsFromConvexCover(convex_cover_boundary, zn, minimumX, xSteps);
            Debug.LogFormat("Number of ConvexPoints: {0}", convexPoints.Count);
            Debug.Log("Number of ConvexPoints: ----------------------------------------");
            
            
            //3) Solve the travelling salesman problem_________________________________________________________________
            //Init variables
            var theta = transform.eulerAngles.y;
            fa_mid = transform.position + Quaternion.Euler(0, theta, 0) * Vector3.forward * (L / 2);
            // Start on ground
            fa_mid.y = 0;
            
            //Init data structures

            solveTSPNaive(convexPoints, tspPath, fa_mid);
            Debug.Log(tspPath.Count);
            
            //4) Plan a final path traversal___________________________________________________________________________
            traversabilityManager = new TraversabilityManager(terrain_manager, car_radius);
            planPath(tspPath);
            Debug.Log(finalPath.Count);
            //finalPath = tspPath;
            
            
            //5 Prepare for set of_____________________________________________________________________________________
            pid_controller = new PIDController(finalPath, m_Car.m_MaximumSteerAngle);
            
            //Debug.Log(pid_controller.current);
        }

        
        private void FixedUpdate()
        {
            
            if (pid_controller.check_Should_Reverse(transform.position, transform.forward) || collision)
            {
                if (collision)
                {
                    //Check if we have 'cleared'
                    if (stopwatch.ElapsedMilliseconds > 1000)
                    {
                        collision = false;
                        stopwatch.Stop();
                        stopwatch.Reset();
                    }
                }
                Debug.Log("Reversing!!!!!!!!");
                var localVel = transform.InverseTransformDirection(GetComponent<Rigidbody>().velocity);
                float forwardSpeed = localVel.z; //Negative speed means it is reversing
                NextMove nextMove = pid_controller.reverse_Routine(forwardSpeed, transform.position, transform.right);
                m_Car.Move(nextMove.steeringAngle, nextMove.throttle, nextMove.footBrake, nextMove.handBrake);
            }
            else
            {
                Debug.Log("Heading for next target!!!!!!!!!!!!");
                var theta = transform.eulerAngles.y;
                fa_mid = transform.position + Quaternion.Euler(0, theta, 0) * Vector3.forward * (L / 2);
                ra_mid = transform.position - Quaternion.Euler(0, theta, 0) * Vector3.forward * (L / 2);
                var act_fa_mid = m_Car.m_WheelMeshes[0].transform.position +
                                 (m_Car.m_WheelMeshes[1].transform.position -
                                  m_Car.m_WheelMeshes[0].transform.position) /
                                 2;
                var act_ra_mid = m_Car.m_WheelMeshes[2].transform.position +
                                 (m_Car.m_WheelMeshes[3].transform.position -
                                  m_Car.m_WheelMeshes[2].transform.position) /
                                 2;


                // Draw line from front axel mid to target
                var target = finalPath[pid_controller.current];
                var target_vec = target;
                Debug.DrawLine(fa_mid, target_vec, Color.blue);

                // Draw CTE line
                var progress = pid_controller.get_progress_point(fa_mid);
                //Debug.DrawLine(fa_mid, progress, Color.red);

                //Debug.DrawLine(fa_mid, final_path[pid_controller.current - 1], Color.green);

                var throttle = max_throttle;
                var target_vel = 10f; //2f
                var velocity = GetComponent<Rigidbody>().velocity.magnitude;
                if (velocity > target_vel)
                {
                    throttle = 0;
                }

                //Debug.LogFormat("Here am I now: {0}", transform.position);
                var steer = pid_controller.get_controls(fa_mid, transform.right);
                //UnityEngine.Debug.Log("steer = " + steer);
                m_Car.Move(steer, throttle, 0f, 0f);
            }
            //m_Car.Move(0f, 0f, 0f, 0f);
        }

        private void OnCollisionEnter()
        {
            collision = true;
            stopwatch.Start();
        }

        private void OnDrawGizmos()
        {
            // Draw traverability
            /*for (int i = 0; i < traversabilityManager.n_x; i++)
            {
                for (int j = 0; j < traversabilityManager.n_z; j++)
                {
                    float x = traversabilityManager.get_x_pos(i), z = traversabilityManager.get_z_pos(j);
                    Vector3 center = new Vector3(x, 0, z);
                    Gizmos.color = Color.blue;
                    //Gizmos.DrawWireCube(center, new Vector3(traversabilityManager.grid_x, 0, traversabilityManager.grid_z));
                    if (!traversabilityManager.traversableCS[i, j])
                    {
                        Gizmos.color = Color.red;
                    }
                    Gizmos.DrawSphere(center, 2f);
                }
            }*/
            
            // Draw path and trajectories along path

            if (friends[0].name.Equals(gameObject.name))
            {
                Gizmos.color = Color.yellow;
            }
            else if (friends[1].name.Equals(gameObject.name))
            {
                Gizmos.color = Color.cyan;
            }
            else if (friends[2].name.Equals(gameObject.name))
            {
                Gizmos.color = Color.red;
            }
            
            for (int i = 0; i < finalPath.Count-1; i++)
            {
                Gizmos.DrawSphere(finalPath[i], 2f);
                Gizmos.DrawLine(finalPath[i], finalPath[i+1]);
            }
            Gizmos.DrawSphere(finalPath[finalPath.Count-1], 5f);
            
            //Draw all mini goals
            Gizmos.color = Color.blue;
            for (int i = 0; i < tspPath.Count-1; i++)
            {
                //Gizmos.color = Color.blue;
                Gizmos.DrawSphere(tspPath[i], 5f);
                //Gizmos.DrawLine(tspPath[i], tspPath[i+1]);
            }
            Gizmos.DrawSphere(tspPath[tspPath.Count-1], 5f);
            
            //Draw all convex centers
            /*for (int i = 0; i < convexPoints.Count; i++)
            {
                Gizmos.color = Color.red;
                Gizmos.DrawSphere(convexPoints[i], 5f);
                //Gizmos.DrawLine(tspPath[i], tspPath[i+1]);
            }
            //Gizmos.DrawSphere(tspPath[tspPath.Count-1], 5f);*/
        }


        //Support fucntions for 1)_____________________________________________________________________________________
        
        
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

        //2) Support functions for finding min set of points___________________________________________________________

        private float selectMinimumX(float xSteps)
        {
            float minimumX = 0;
            //Debug.Log(friends[0].name.Equals(gameObject.name));
            if (friends[0].name.Equals(gameObject.name))
            {
                minimumX = 0;
            }
            else if (friends[1].name.Equals(gameObject.name))
            {
                minimumX = xSteps;
            }
            else if (friends[2].name.Equals(gameObject.name))
            {
                minimumX = xSteps * 2;
            }

            return minimumX;
        }
        
        private void selectPointsFromConvexCover(List<Tuple<int, int, int, int>> convex_cover_boundary, int zn, float minimumX, float xStep)
        {
            //Select centerpoint of each rectangle
            float x_step = (terrain_manager.myInfo.x_high - terrain_manager.myInfo.x_low) / terrain_manager.myInfo.x_N;
            float z_step = (terrain_manager.myInfo.z_high - terrain_manager.myInfo.z_low) / terrain_manager.myInfo.z_N;
            for (int i = 0; i < convex_cover_boundary.Count; i++)
            {
                var hehe = convex_cover_boundary[i];
                float x_center = (terrain_manager.myInfo.get_x_pos(hehe.Item3) - x_step / 2 +
                 terrain_manager.myInfo.get_x_pos(hehe.Item4) + x_step / 2) / 2;
                float z_center = (terrain_manager.myInfo.get_x_pos(zn - 1 - hehe.Item2) - z_step / 2 +
                 terrain_manager.myInfo.get_x_pos(zn - 1 - hehe.Item1) + z_step / 2) / 2;
                
                Debug.LogFormat("Center point x: {0}, y: {1}", x_center, z_center);
                Debug.LogFormat("Minimum X: {0}", minimumX);
                if (minimumX <= x_center && x_center < minimumX + xStep)
                {
                    convexPoints.Add(new Vector3(x_center, 0f, z_center));
                }
            }
            
        }
        
        //3) Support functions for solving traveling salesman problem__________________________________________________
        private void solveTSPNaive(List<Vector3> convexPoints, List<Vector3> tspPath, Vector3 startPos)
        {
            //tspPath.Add(startPos);
            Vector3 currentPoint = startPos;
            float currentDistance;
            int closetsPointIndex = 0;
            while (convexPoints.Count > 0)
            {
                float shortestPath = float.PositiveInfinity;
                for (int i = 0; i < convexPoints.Count; i++)
                {
                    currentDistance = (currentPoint - convexPoints[i]).magnitude;
                    if (currentDistance < shortestPath)
                    {
                        shortestPath = currentDistance;
                        closetsPointIndex = i;
                    }
                }
                Debug.LogFormat("Final path: Center point x: {0}, y: {1}", convexPoints[closetsPointIndex].x, convexPoints[closetsPointIndex].z);
                currentPoint = convexPoints[closetsPointIndex];
                tspPath.Add(convexPoints[closetsPointIndex]);
                convexPoints.RemoveAt(closetsPointIndex);
                
            }
        }
        
        //4 Support functions for path planning and construction_______________________________________________________
        private void planPath(List<Vector3> tspPath)
        {
            finalPath.AddRange(pathFinder.FindPath(traversabilityManager.traversableCS, fa_mid, tspPath[0], traversabilityManager, m_Car.m_MaximumSteerAngle, L, max_v));
            for(int i = 1; i < tspPath.Count; i++)
            {
                finalPath.AddRange(pathFinder.FindPath(traversabilityManager.traversableCS, tspPath[i-1], tspPath[i], traversabilityManager, m_Car.m_MaximumSteerAngle, L, max_v));
            }
        }
        
        //FixedUpdate support functions________________________________________________________________________________
    }
}