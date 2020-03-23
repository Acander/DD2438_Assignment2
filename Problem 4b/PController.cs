using System.Collections.Generic;
using System.Diagnostics;
using KruskalMinimumSpanningTree;
using UnityEditor;
using UnityEngine;
using UnityStandardAssets.Vehicles.Car;
using Debug = UnityEngine.Debug;

namespace Scrips
{
    public class PController
    {
        //Position Controller for individual car
        private GameObject car;
        private GameObject leader;
        public CarPosition formationPosition;
        private PathFinder pathFinder = new PathFinder();
        private TraversabilityManager traversabilityManager;
        //private GameObject[] friends;

        private PIDController pidController;
        public List<Vector3> pathToParent = new List<Vector3>();
        public Stopwatch stopwatch = new Stopwatch();

        private float L = 2.870426f;
        private float theta;
        private Vector3 fa_mid;
        private double speedTolerance = 1;
        public bool collision = false;

        public PController(GameObject car, GameObject leader, CarPosition initPosition, TraversabilityManager traversabilityManager, GameObject[] friends)
        {
            this.car = car;
            theta = car.transform.eulerAngles.y;
            fa_mid = car.transform.position + Quaternion.Euler(0, theta, 0) * Vector3.forward * (L / 2);
            this.leader = leader;
            formationPosition = initPosition;
            this.traversabilityManager = traversabilityManager;
            
        }
        
        public CarControls getCarControls()
        {
            //plan path to parent
            fa_mid = car.transform.position + Quaternion.Euler(0, theta, 0) * Vector3.forward * (L / 2);
            pathToParent = pathFinder.FindPath(car.transform.position, formationPosition.position,
                traversabilityManager);
            pathToParent.Add(fa_mid);
            pidController = new PIDController(pathToParent, car.GetComponent<CarController>().m_MaximumSteerAngle);

            if (passed(car.transform.position, leader.transform.forward, formationPosition.position))
            {
                return drivingRoutine();
            }
            if (pidController.check_Should_Reverse(car.transform.position, car.transform.forward) || collision)
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

                return reverse();
                //return new CarControls(0f, -1f, -1f, 0f);
                
            }

            return drivingRoutine();
            
            
            
        }
        
        private bool passed(Vector3 pos, Vector3 direction, Vector3 goal_pos)
        {
            /*var between = end_pos - start_pos;
            var progress = pos - start_pos;
            var projection = Vector3.Project(progress, between);*/

            Vector3 between = -(goal_pos - pos);

            return Vector3.Dot(between, direction) > 0;
        }

        private float matchCarControls()
        {
            if (withinToleranceLevel())
            {
                return 0.1f;
            }
            if (faster())
            {
                return -1f;
            }

            return 1f;
        }

        private bool withinToleranceLevel()
        {
            double leaderSpeed = leader.GetComponent<CarController>().CurrentSpeed;
            return leaderSpeed - speedTolerance < car.GetComponent<CarController>().CurrentSpeed && car.GetComponent<CarController>().CurrentSpeed < leaderSpeed + speedTolerance;
        }

        private bool faster()
        {
            double leaderSpeed = leader.GetComponent<CarController>().CurrentSpeed;
            return leaderSpeed < car.GetComponent<CarController>().CurrentSpeed;
        }

        private CarControls drivingRoutine()
        {
            float magnitude;
            bool passed;
            bool is_to_the_right;

            passed = this.passed(car.transform.position, leader.transform.forward, formationPosition.position);
            Vector3 between = formationPosition.position - car.transform.position;
            is_to_the_right = Vector3.Dot(between, car.transform.right) < 0;
            magnitude = between.magnitude;
            Debug.DrawLine(formationPosition.position, car.transform.position);
            
            
            float acceleration = 0.05f*magnitude;
            float steering = 0.05f*magnitude;
            
            if (passed)
            {
                acceleration = 0;
                is_to_the_right = Vector3.Dot(leader.transform.forward, car.transform.right) < 0;
                if (is_to_the_right)
                {
                    steering = -steering;
                }
            }
            else if (magnitude < 10)
            {
                acceleration = 0f;
                steering = 0.1f;
                is_to_the_right = Vector3.Dot(between, car.transform.right) < 0;
                if (is_to_the_right)
                {
                    steering = -steering;
                }
            }
            else
            {
                steering = pidController.get_controls(fa_mid, car.transform.right);
            }

            return new CarControls(steering, acceleration, acceleration, 0);
        }

        private CarControls reverse()
        {
            Debug.Log("Reversing!!!!!!!!");
            var localVel = car.transform.InverseTransformDirection(car.GetComponent<Rigidbody>().velocity);
            float forwardSpeed = localVel.z; //Negative speed means it is reversing
            NextMove nextMove = pidController.reverse_Routine(forwardSpeed, car.transform.position, car.transform.right);
            return new CarControls(nextMove.steeringAngle, nextMove.throttle, nextMove.footBrake, nextMove.handBrake);
        }
        

    }
}