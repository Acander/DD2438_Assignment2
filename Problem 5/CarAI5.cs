using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Diagnostics;
using System.Numerics;
using KruskalMinimumSpanningTree;
using Scrips;
using UnityEngine.UIElements;
using Debug = UnityEngine.Debug;
using Quaternion = UnityEngine.Quaternion;
using Vector3 = UnityEngine.Vector3;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI5 : MonoBehaviour
    {
        private CarController m_Car;
        private float car_radius = 5f;
        private float L = 2.870426f;
        private float L_f;
        private float L_b;
        private float max_throttle = 1f;
        private float max_v = 8;
        private Vector3 fa_mid;
        private PIDController pidController;

        private Boolean collision;

        private Stopwatch stopwatch = new Stopwatch();

        private CarFormation vShape;
        private CarFormation slimShape;
        private CarFormation currentFormation;
        private float sweepLength = 70f;

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;
        private TraversabilityManager traversabilityManager;

        public GameObject[] friends;
        public GameObject[] enemies;

        private int[] turretScores;
        private List<Vector3> finalRoute = new List<Vector3>();
        public List<Vector3> finalPath = new List<Vector3>();
        PathFinder pathFinder = new PathFinder();


        private void Start()
        {
            // get the car controller
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();
            traversabilityManager = new TraversabilityManager(terrain_manager, car_radius);

            // note that both arrays will have holes when objects are destroyed
            // but for initial planning they should work
            friends = GameObject.FindGameObjectsWithTag("Player");
            enemies = GameObject.FindGameObjectsWithTag("Enemy");
            foreach (GameObject obj in enemies)
            {
                Debug.DrawLine(transform.position, obj.transform.position, Color.black, 10f);
            }

            if (friends[0])
            {
                //Init leader
                calculateTurretScores();
                sortTurretList();
                planPath(finalRoute);
                finalPath.Insert(0, fa_mid);
                pidController = new PIDController(finalPath, m_Car.m_MaximumSteerAngle);
            }
            else
            {
                //Init wingmen
                initCarFormation();
            }

        }

        private void FixedUpdate()
        {

            if (friends[0].name == name)
            {
                leaderControls();
            }
            else
            {
                wingmanControls();
            }
        }

        private void OnCollisionEnter()
        {
            collision = true;
            stopwatch.Start();
        }
        
        private void OnDrawGizmos()
        {
            //Draw formation
            Gizmos.color = Color.cyan;
            Debug.Log(currentFormation.innerLeft.formationPosition.position);
            Gizmos.DrawSphere(currentFormation.innerLeft.formationPosition.position, 3f);
            Gizmos.DrawSphere(currentFormation.innerRight.formationPosition.position, 3f);
            Gizmos.DrawSphere(currentFormation.outerLeft.formationPosition.position, 3f);
            Gizmos.DrawSphere(currentFormation.outerRight.formationPosition.position, 3f);
            
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
            for (int i = 0; i < finalRoute.Count-1; i++)
            {
                //Gizmos.color = Color.blue;
                Gizmos.DrawSphere(finalRoute[i], 5f);
                //Gizmos.DrawLine(tspPath[i], tspPath[i+1]);
            }
            Gizmos.DrawSphere(finalRoute[finalRoute.Count-1], 5f);
        }

        //1) Plan leader path_______________________________________________________________________________________________
        private void calculateTurretScores()
        {
            turretScores = new int[enemies.Length];

            for (int current = 0; current < turretScores.Length - 1; current++)
            {
                for (int interest = current + 1; interest < turretScores.Length; interest++)
                {
                    if (!outOfsight(friends[current].transform.position, friends[interest].transform.position))
                    {
                        turretScores[current]++;
                        turretScores[interest]++;
                    }
                }
            }
        }

        private void sortTurretList()
        {
            for (int i = 0; i < turretScores.Length; i++)
            {
                int lowestIndex = 0;
                int lowest = int.MaxValue;
                for (int j = 0; j < turretScores.Length; j++)
                {
                    if (turretScores[j] < lowest)
                    {
                        lowestIndex = j;
                        lowest = turretScores[j];
                    }
                }
                turretScores[lowestIndex] = int.MaxValue;
                finalRoute.Add(enemies[lowestIndex].transform.position);
            }
        }

        private bool outOfsight(Vector3 currentPoint, Vector3 pointOfinterest)
        {
            Vector3 between = currentPoint - pointOfinterest;
            Vector3 betweenNorm = between.normalized;
            Vector3 currentBetweenPoint = currentPoint + (betweenNorm * car_radius);
            int i = 1;
            while (!finishedIterating(currentBetweenPoint, pointOfinterest))
            {
                currentBetweenPoint = currentPoint + (betweenNorm * i * car_radius);
                if (!traversabilityManager.PointTraversableCS(currentBetweenPoint.x, currentBetweenPoint.z))
                {
                    return true;
                }
                i++;
            }
            return false;
        }

        private bool finishedIterating(Vector3 point, Vector3 pointOfInterest)
        {
            return (point - pointOfInterest).magnitude < 3;
        }

        private void planPath(List<Vector3> tspPath)
        {
            finalPath.AddRange(pathFinder.FindPath(traversabilityManager.traversableCS, fa_mid, tspPath[0],
                traversabilityManager, m_Car.m_MaximumSteerAngle, L, max_v));
            for (int i = 1; i < tspPath.Count; i++)
            {
                Debug.LogFormat("Nr goal {0}/{1}, {2} to {3}", i + 1, tspPath.Count, tspPath[i - 1], tspPath[i]);
                finalPath.AddRange(pathFinder.FindPath(traversabilityManager.traversableCS, tspPath[i - 1], tspPath[i],
                    traversabilityManager, m_Car.m_MaximumSteerAngle, L, max_v));
                Debug.LogFormat("Length of final path: {0}", finalPath.Count);
            }
        }

        //2) Support code for controlling the leader car
        private void leaderControls()
        {
            Debug.LogFormat("Current node: {0}", pidController.current);
            if (pidController.check_Should_Reverse(transform.position, transform.forward) || collision)
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
                NextMove nextMove = pidController.reverse_Routine(forwardSpeed, transform.position, transform.right);
                m_Car.Move(nextMove.steeringAngle, nextMove.throttle, nextMove.footBrake, nextMove.handBrake);
            }
            else
            {
                Debug.Log("Heading for next target!!!!!!!!!!!!");
                var theta = transform.eulerAngles.y;
                fa_mid = transform.position + Quaternion.Euler(0, theta, 0) * Vector3.forward * (L / 2);

                var target = finalPath[pidController.current];
                var target_vec = target;
                Debug.DrawLine(fa_mid, target_vec, Color.blue);

                // Draw CTE line
                var progress = pidController.get_progress_point(fa_mid);
                //Debug.DrawLine(fa_mid, progress, Color.red);

                var throttle = max_throttle;
                var target_vel = 10f; //2f
                var velocity = GetComponent<Rigidbody>().velocity.magnitude;
                if (velocity > target_vel)
                {
                    throttle = 0;
                }

                //Debug.LogFormat("Here am I now: {0}", transform.position);
                var steer = pidController.get_controls(fa_mid, transform.right);
                //UnityEngine.Debug.Log("steer = " + steer);
                m_Car.Move(steer, throttle, 0f, 0f);
            }
        }


        //3) Support code for the wingmen______________________________________________________________________________________
        private void wingmanControls()
        {
            Vector3 goalPos = new Vector3();
            if (friends[1].name == gameObject.name)
            {
                goalPos = currentFormation.innerLeft.formationPosition.position;
            } 
            else if (friends[2].name == gameObject.name)
            {
                goalPos = currentFormation.innerRight.formationPosition.position;
            }
            
            
            CarControls carControls;
            Vector3 leaderPosition = friends[0].transform.position;
            Vector3 leftSidePosition = leaderPosition - friends[0].transform.right*sweepLength/2;
            Vector3 rightSidePosition = leaderPosition + friends[0].transform.right*sweepLength/2;
            if (!traversabilityManager.PointTraversableCS(rightSidePosition.x, rightSidePosition.z) || !traversabilityManager.PointTraversableCS(leftSidePosition.x, leftSidePosition.z))
            {
                currentFormation = slimShape;
            }
            else
            {
                currentFormation = vShape;
            }
            carControls = currentFormation.getCarControls(gameObject);
            m_Car.Move(carControls.steering, carControls.acceleration, carControls.acceleration, 0f);
        }
        
        public float steer_dir(Vector3 pos, Vector3 right, Vector3 end_pos)
        {
            var dir = end_pos - pos;
            float dot = Vector3.Dot(right, dir);
            return dot > 0f ? 1f : -1f;
        }

        private void initCarFormation()
        {
            //Init V-formation 

            //Car 1 (Inner left)
            CarPosition innerLeft = new CarPosition(friends[0].transform.position, new Vector3(-sweepLength / 4, 0, -12));

            //Car 2 (Inner right)
            CarPosition innerRight = new CarPosition(friends[0].transform.position, new Vector3(sweepLength / 4, 0, -12));

            //Car 3 (Outer left)
            CarPosition outerLeft = new CarPosition(friends[0].transform.position, new Vector3(-sweepLength / 2, 0, -24));

            //Car 4 (Outer right)
            CarPosition outerRight = new CarPosition(friends[0].transform.position, new Vector3(sweepLength / 2, 0, -24));

            vShape = new CarFormation(friends, innerLeft, innerRight, outerLeft, outerRight);

            //Init Sweep-formation

            //Car 1 (Inner left)
            innerLeft = new CarPosition(friends[0].transform.position, new Vector3(-sweepLength / 8, 0, -12));

            //Car 2 (Inner right)
            innerRight = new CarPosition(friends[0].transform.position, new Vector3(sweepLength / 8, 0, -12));

            //Car 3 (Outer left)
            outerLeft = new CarPosition(friends[0].transform.position, new Vector3(-sweepLength / 4, 0, -18));

            //Car 4 (Outer right)
            outerRight = new CarPosition(friends[0].transform.position, new Vector3(sweepLength / 4, 0, -18));

            slimShape = new CarFormation(friends, innerLeft, innerRight, outerLeft, outerRight);

            currentFormation = vShape;
        }

    }
}
