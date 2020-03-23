using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Diagnostics;
using System.Numerics;
using Scrips;
using UnityEditor;
using Debug = UnityEngine.Debug;
using Vector3 = UnityEngine.Vector3;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI4 : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use
        private float car_radius = 5f;
        private Stopwatch stopwatch = new Stopwatch();
        private bool collision = false;
        public List<Vector3> pathToPosition = new List<Vector3>();
        
        private double inner_height = 5;
        private double outer_height = 10;
        private double inner_width = 2;
        private double outer_width = 4;
        private float standardAcceleration = 0.1f;
        private GameObject parent;

        private CarFormation vShape;
        private CarFormation slimShape;
        private CarFormation currentFormation;
        private float sweepLength = 70f;

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;
        private TraversabilityManager traversabilityManager;

        public GameObject[] friends;
        public GameObject[] enemies;

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
            
            
            //Init V-formation 
            
            //Car 1 (Inner left)
            CarPosition innerLeft = new CarPosition(friends[0].transform.position, new Vector3(-sweepLength/4, 0,  -12));

            //Car 2 (Inner right)
            CarPosition innerRight = new CarPosition(friends[0].transform.position, new Vector3(sweepLength/4, 0, -12));
            
            //Car 3 (Outer left)
            CarPosition outerLeft = new CarPosition(friends[0].transform.position, new Vector3(-sweepLength/2, 0, -24));
            
            //Car 4 (Outer right)
            CarPosition outerRight = new CarPosition(friends[0].transform.position, new Vector3(sweepLength/2, 0, -24));
            
            vShape = new CarFormation(friends, innerLeft, innerRight, outerLeft, outerRight, traversabilityManager);

            //Init Sweep-formation
            
            //Car 1 (Inner left)
            innerLeft = new CarPosition(friends[0].transform.position, new Vector3(-sweepLength/8, 0,  -12));

            //Car 2 (Inner right)
            innerRight = new CarPosition(friends[0].transform.position, new Vector3(sweepLength/8, 0,  -12));
            
            //Car 3 (Outer left)
            outerLeft = new CarPosition(friends[0].transform.position, new Vector3(-sweepLength/4, 0,  -18));
            
            //Car 4 (Outer right)
            outerRight = new CarPosition(friends[0].transform.position, new Vector3(sweepLength/4, 0,  -18));
            
            slimShape = new CarFormation(friends, innerLeft, innerRight, outerLeft, outerRight, traversabilityManager);

            currentFormation = vShape;
        }


        private void FixedUpdate()
        {
            Vector3 goalPos = new Vector3();
            if (friends[1].name == gameObject.name)
            {
                goalPos = currentFormation.innerLeft.formationPosition.position;
            } 
            else if (friends[3].name == gameObject.name)
            {
                goalPos = currentFormation.outerLeft.formationPosition.position;
            }
            else if (friends[2].name == gameObject.name)
            {
                goalPos = currentFormation.innerRight.formationPosition.position;
            }
            else if (friends[4].name == gameObject.name)
            {
                goalPos = currentFormation.outerRight.formationPosition.position;
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
            
            /*if (check_Should_Reverse(transform.position, transform.forward, goalPos) || collision)
            {
                currentFormation.getCarControls(gameObject);
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
                NextMove nextMove = reverse_Routine(forwardSpeed, transform.position, transform.right, goalPos);
                m_Car.Move(nextMove.steeringAngle, nextMove.throttle, nextMove.footBrake, nextMove.handBrake);
            }
            else
            {
                carControls = currentFormation.getCarControls(gameObject);
                m_Car.Move(carControls.steering, carControls.acceleration, carControls.acceleration, 0f);
            }*/
            
            carControls = currentFormation.getCarControls(gameObject);
            m_Car.Move(carControls.steering, carControls.acceleration, carControls.acceleration, 0f);



        }
        
        public NextMove reverse_Routine(float forwardVelocity, Vector3 currentPos, Vector3 right, Vector3 goalPos)
        {
            //Debug.Log("Reversing!!!!!!!!!!!!!!!!");
            float steeringAngle = steer_dir(currentPos, right, goalPos);
            if (forwardVelocity > 0)
            {
                return new NextMove(steeringAngle, 0, -1, 0f);
            }
            //Debug.Log("Reversing!!!!!!!!!!!!!!!!");
            return new NextMove(-steeringAngle, 0, -1, 0f);
            //return new NextMove(0, -1f, 0f, 0f);
        }
        
        public Boolean check_Should_Reverse(Vector3 currentPos, Vector3 carHeading, Vector3 goalPos)
        {
            Debug.DrawLine(currentPos, goalPos, Color.white);
            float reverseScore = Vector3.Dot(goalPos-currentPos, carHeading);
            //Debug.LogFormat("Reverse score: {0}", reverseScore);
            //Debug.LogFormat("Carheading: {0} ", carHeading);
            return reverseScore < 0;
        }
        
        public float steer_dir(Vector3 pos, Vector3 right, Vector3 end_pos)
        {
            var dir = end_pos - pos;
            float dot = Vector3.Dot(right, dir);
            return dot > 0f ? 1f : -1f;
        }
        
        private void OnCollisionEnter()
        {
            bool inCollisionSpace =
                !traversabilityManager.PointTraversableCS(transform.position.x, transform.position.z);

            if (inCollisionSpace)
            {
                if (friends[1].name == gameObject.name)
                {
                    currentFormation.innerLeft.collision = true;
                    currentFormation.innerLeft.stopwatch.Start();
                }
                else if (friends[3].name == gameObject.name)
                {
                    currentFormation.outerLeft.collision = true;
                    currentFormation.outerLeft.stopwatch.Start();
                }
                else if (friends[2].name == gameObject.name)
                {
                    currentFormation.innerRight.collision = true;
                    currentFormation.innerRight.stopwatch.Start();
                }
                else if (friends[4].name == gameObject.name)
                {
                    currentFormation.outerRight.collision = true;
                    currentFormation.outerRight.stopwatch.Start();
                }
            }
        }

        private void OnDrawGizmos()
        {
            Gizmos.color = Color.cyan;
            Debug.Log(currentFormation.innerLeft.formationPosition.position);
            Gizmos.DrawSphere(currentFormation.innerLeft.formationPosition.position, 3f);
            Gizmos.DrawSphere(currentFormation.innerRight.formationPosition.position, 3f);
            Gizmos.DrawSphere(currentFormation.outerLeft.formationPosition.position, 3f);
            Gizmos.DrawSphere(currentFormation.outerRight.formationPosition.position, 3f);
            
            //Draw path for innerLeft
            List<Vector3> path = new List<Vector3>();
            if (friends[1].name == gameObject.name)
            {
                path = currentFormation.innerLeft.pathToParent;
                Gizmos.color = Color.yellow;
            } 
            else if (friends[3].name == gameObject.name)
            {
                path = currentFormation.outerLeft.pathToParent;
                Gizmos.color = Color.red;
            }
            else if (friends[2].name == gameObject.name)
            {
                path = currentFormation.innerRight.pathToParent;
                Gizmos.color = Color.green;
            }
            else if (friends[4].name == gameObject.name)
            {
                path = currentFormation.outerRight.pathToParent;
                Gizmos.color = Color.black;
            }
            
            Debug.Log(path.Count); 
            for (int i = 0; i < path.Count-1; i++)
            {
                Gizmos.DrawSphere(path[i], 2f);
                Gizmos.DrawLine(path[i], path[i+1]);
            }
            Gizmos.DrawSphere(path[path.Count-1], 5f);
        }
        
    }
}
