using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Diagnostics;
using KruskalMinimumSpanningTree;
using Scrips;
using Debug = UnityEngine.Debug;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI3 : MonoBehaviour
    {
        public CarController m_Car;

        public PIDController pid_controller;
        
        private float car_radius = 5f;
        private float L = 2.870426f;
        private float L_f;
        private float L_b;
        private float max_throttle = 1f;
        private float max_v = 8;
        private Vector3 fa_mid;
        //private Vector3 ra_mid;

        private Boolean collision;

        private Stopwatch stopwatch = new Stopwatch();

        List<Vector3> enemyPositions = new List<Vector3>();
        List<Vector3> tspPath = new List<Vector3>();
        public List<Vector3> finalPath = new List<Vector3>();

        private PathFinder pathFinder = new PathFinder();
        private TraversabilityManager traversabilityManager;

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;

        public GameObject[] friends;
        public GameObject[] enemies;

        private void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            //Divides the terrain into three steps (or areas) for each car
            float xStep = terrain_manager.myInfo.x_high / 3;

            // note that both arrays will have holes when objects are destroyed
            // but for initial planning they should work
            friends = GameObject.FindGameObjectsWithTag("Player");
            enemies = GameObject.FindGameObjectsWithTag("Enemy");
            Debug.LogFormat("Total number of enemy positions: {0}", enemies.Length);
            foreach (GameObject obj in enemies)
            {
                //Debug.DrawLine(transform.position, obj.transform.position, Color.black, 10f);
                float minimumX = selectMinimumX(xStep);
                float posX = obj.transform.position.x;
                if (minimumX <= posX && posX < minimumX + xStep)
                {
                    enemyPositions.Add(obj.transform.position);
                }
            }
            
            Debug.LogFormat("Number of enemy positions {0}", enemyPositions);


            // Plan your path here
            //3) Solve the travelling salesman problem_________________________________________________________________
            //Init variables
            var theta = transform.eulerAngles.y;
            fa_mid = transform.position + Quaternion.Euler(0, theta, 0) * Vector3.forward * (L / 2);
            // Start on ground
            fa_mid.y = 0;
            
            //Init data structures

            solveTSPNaive(enemyPositions, tspPath, fa_mid);
            Debug.LogFormat("Car {0}: Length of path {1}", gameObject.name, tspPath.Count);
            
            //4) Plan a final path traversal___________________________________________________________________________
            traversabilityManager = new TraversabilityManager(terrain_manager, car_radius);
            planPath(tspPath);
            Debug.Log(finalPath.Count);
            Debug.Log(finalPath[finalPath.Count-1]);
            //finalPath = tspPath;
            
            Debug.LogFormat("Finalpath[0] {0} vs fa_mid {1}", finalPath[0], fa_mid);
            
            //5 Prepare for set of_____________________________________________________________________________________
            finalPath.Insert(0, fa_mid);
            pid_controller = new PIDController(finalPath, m_Car.m_MaximumSteerAngle);

            Debug.LogFormat("Finalpath[0] {0} vs fa_mid {1}", finalPath[0], fa_mid);
        }


        private void FixedUpdate()
        {


            // Execute your path here
            // ...
            Debug.LogFormat("Current node: {0}", pid_controller.current);
            
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

                var target = finalPath[pid_controller.current];
                var target_vec = target;
                Debug.DrawLine(fa_mid, target_vec, Color.blue);

                // Draw CTE line
                var progress = pid_controller.get_progress_point(fa_mid);
                //Debug.DrawLine(fa_mid, progress, Color.red);

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

            /*Vector3 avg_pos = Vector3.zero;

            foreach (GameObject friend in friends)
            {
                avg_pos += friend.transform.position;
            }
            avg_pos = avg_pos / friends.Length;
            Vector3 direction = (avg_pos - transform.position).normalized;

            bool is_to_the_right = Vector3.Dot(direction, transform.right) > 0f;
            bool is_to_the_front = Vector3.Dot(direction, transform.forward) > 0f;

            float steering = 0f;
            float acceleration = 0;

            if (is_to_the_right && is_to_the_front)
            {
                steering = 1f;
                acceleration = 1f;
            }
            else if (is_to_the_right && !is_to_the_front)
            {
                steering = -1f;
                acceleration = -1f;
            }
            else if (!is_to_the_right && is_to_the_front)
            {
                steering = -1f;
                acceleration = 1f;
            }
            else if (!is_to_the_right && !is_to_the_front)
            {
                steering = 1f;
                acceleration = -1f;
            }

            // this is how you access information about the terrain
            int i = terrain_manager.myInfo.get_i_index(transform.position.x);
            int j = terrain_manager.myInfo.get_j_index(transform.position.z);
            float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
            float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

            Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z));


            // this is how you control the car
            Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
            m_Car.Move(steering, acceleration, acceleration, 0f);
            //m_Car.Move(0f, -1f, 1f, 0f);*/


        }
        
        private void OnDrawGizmos()
        {
            // Draw traversabillity
            /*
            for (int i = 0; i < traversabilityManager.n_x; i++)
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
            
            Debug.Log(finalPath.Count); 
            for (int i = 0; i < finalPath.Count-1; i++)
            {
                Gizmos.DrawSphere(finalPath[i], 2f);
                Gizmos.DrawLine(finalPath[i], finalPath[i+1]);
            }
            Gizmos.DrawSphere(finalPath[finalPath.Count-1], 5f);
            
            
            //Draw all mini goals
            Gizmos.color = Color.green;
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
        
        //1) Support functions for solving traveling salesman problem__________________________________________________
        private void solveTSPNaive(List<Vector3> enemyPositions, List<Vector3> tspPath, Vector3 startPos)
        {
            //tspPath.Add(startPos);
            Vector3 currentPoint = startPos;
            float currentDistance;
            int closetsPointIndex = 0;
            while (enemyPositions.Count > 0)
            {
                float shortestPath = float.PositiveInfinity;
                for (int i = 0; i < enemyPositions.Count; i++)
                {
                    currentDistance = (currentPoint - enemyPositions[i]).magnitude;
                    if (currentDistance < shortestPath)
                    {
                        shortestPath = currentDistance;
                        closetsPointIndex = i;
                    }
                }
                Debug.LogFormat("Final path: Center point x: {0}, y: {1}", enemyPositions[closetsPointIndex].x, enemyPositions[closetsPointIndex].z);
                currentPoint = enemyPositions[closetsPointIndex];
                tspPath.Add(enemyPositions[closetsPointIndex]);
                enemyPositions.RemoveAt(closetsPointIndex);
                
            }
        }
        
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
        
        //2) Support functions for path planning and construction_______________________________________________________
        private void planPath(List<Vector3> tspPath)
        {
            finalPath.AddRange(pathFinder.FindPath(traversabilityManager.traversableCS, fa_mid, tspPath[0], traversabilityManager, m_Car.m_MaximumSteerAngle, L, max_v));
            for(int i = 1; i < tspPath.Count; i++)
            {
                Debug.LogFormat("Nr goal {0}/{1}, {2} to {3}", i+1, tspPath.Count, tspPath[i-1], tspPath[i]);
                finalPath.AddRange(pathFinder.FindPath(traversabilityManager.traversableCS, tspPath[i-1], tspPath[i], traversabilityManager, m_Car.m_MaximumSteerAngle, L, max_v));
                Debug.LogFormat("Length of final path: {0}", finalPath.Count);
            }
        }
    }
}
