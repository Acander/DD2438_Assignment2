using System;
using UnityEngine;
using UnityEngine.SocialPlatforms.Impl;
using UnityStandardAssets.Vehicles.Car;

namespace Scrips
{
    //Formation class for 4 cars (not including a replay/leader car)
    
    public class CarFormation
    {
        //Parameters calculated as dot products based on the leaders position
        
        //Leader
        private GameObject leader;
        private CarPosition lastLeaderPosition;
        private Vector3 oldForwardVector;
        
        //Other Cars
        private GameObject[] friends;
        
        //Car 1 (Inner left)
        public CarPosition innerLeft;

        //Car 2 (Inner right)
        public CarPosition innerRight;
        
        //Car 3 (Outer left)
        public CarPosition outerLeft;
        
        //Car 4 (Outer right)
        public CarPosition outerRight;

        public CarFormation(GameObject[] friends, CarPosition innerLeft, CarPosition innerRight, CarPosition outerLeft, CarPosition outerRight)
        {
            leader = friends[0];
            lastLeaderPosition = new CarPosition(leader.transform.position, new Vector3(0, 0, 0));
            oldForwardVector = leader.transform.forward;
            Debug.LogFormat("Initial position {0}", lastLeaderPosition.position);
            this.innerLeft = innerLeft;
            this.innerRight = innerRight;
            this.outerLeft = outerLeft;
            this.outerRight = outerRight;
            this.friends = friends;
        }


        public CarControls getCarControls(GameObject car)
        {
            //Vector3 update = leader.transform.position - lastLeaderPosition.position;
            float dot = Vector3.Dot(leader.transform.right, oldForwardVector);
            float dir = dot > 0f ? -1f : 1f;
            float angle = dir * Vector3.Angle(oldForwardVector, leader.transform.forward);
            Vector3 leaderPosition = leader.transform.position; 
            Debug.LogFormat("Update angle: {0}", angle);
            
            //Save old positions
            innerLeft.oldPos = innerLeft.position;
            innerRight.oldPos = innerRight.position;
            outerLeft.oldPos = outerLeft.position;
            outerRight.oldPos = outerRight.position;
            
            innerLeft.differenceVector = Quaternion.Euler(0, angle, 0) * innerLeft.differenceVector;
            innerLeft.position = leaderPosition - innerLeft.differenceVector;;
            innerRight.differenceVector = Quaternion.Euler(0, angle, 0) * innerRight.differenceVector;
            innerRight.position = leaderPosition - innerRight.differenceVector;
            outerLeft.differenceVector = Quaternion.Euler(0, angle, 0) * outerLeft.differenceVector;
            outerLeft.position = leaderPosition - outerLeft.differenceVector;
            outerRight.differenceVector = Quaternion.Euler(0, angle, 0) * outerRight.differenceVector;
            outerRight.position = leaderPosition - outerRight.differenceVector;
            
            lastLeaderPosition.position = leader.transform.position;
            oldForwardVector = leader.transform.forward;

            /*foreach(var friend in friends)
            {
                parentPosition += friend.transform.position;
            }

            parentPosition = parentPosition / friends.Length;*/
            
            /*if (isInnerCar(car))
            {
                parentPosition = leader.transform.position;
            }
            else if (friends[3].name == car.name)
            {
                leader = friends[1];
                parentPosition = friends[1].transform.position;
            }
            else if (friends[4].name == car.name)
            {
                leader = friends[2];
                parentPosition = friends[2].transform.position;
            }
            
            myPosition = car.transform.position;
            Debug.DrawLine(myPosition, parentPosition);
            dirToLeader = myPosition - parentPosition;
            //dirNorm = dirToLeader.normalized;*/

            Vector3 myPosition = car.transform.position;
            bool passed = false;
            bool is_to_the_right = true;
            //bool tooFarAway = false;
            
            //double steeringError 

            if (friends[1].name == car.name)
            {
                //Left
                passed = this.passed(car.transform.position, innerLeft.oldPos, innerLeft.position);
                is_to_the_right = Vector3.Dot(innerLeft.position - car.transform.position, car.transform.right) < 0;
                Debug.DrawLine(innerLeft.position, car.transform.position);
            } 
            else if (friends[3].name == car.name)
            {
                passed = this.passed(car.transform.position, outerLeft.oldPos, outerLeft.position);
                is_to_the_right = Vector3.Dot(outerLeft.position - car.transform.position, car.transform.right) < 0;
                Debug.DrawLine(outerLeft.position, car.transform.position);
            }
            else if (friends[2].name == car.name)
            {
                passed = this.passed(car.transform.position, innerRight.oldPos, innerRight.position);
                is_to_the_right = Vector3.Dot(innerRight.position - car.transform.position, car.transform.right) < 0;
                Debug.DrawLine(innerRight.position, car.transform.position);
            }
            else if (friends[4].name == car.name)
            {
                passed = this.passed(car.transform.position, outerRight.oldPos, outerRight.position);
                is_to_the_right = Vector3.Dot(outerRight.position - car.transform.position, car.transform.right) < 0;
                Debug.DrawLine(outerRight.position, car.transform.position);
            }
            
            float steering;
            float acceleration = 0.8f;

            if (is_to_the_right)
            {
                steering = -1f;
            }
            else
            {
                steering = 1f;
            }
            /*if (passed)
            {
                acceleration = -1f;
            }
            else
            {
                acceleration = 1f;
            }*/

            CarControls controls = new CarControls(steering, acceleration, acceleration, 0);
            return controls;
        }

        private bool isInnerCar(GameObject car)
        {
            return friends[1].name == car.name || friends[2].name == car.name;
        }
        
        private bool passed(Vector3 pos, Vector3 start_pos, Vector3 end_pos)
        {
            var between = end_pos - start_pos;
            var progress = pos - start_pos;
            var projection = Vector3.Project(progress, between);
            
            return projection.magnitude >= between.magnitude;
        }

    }

    public class CarPosition
    {
        //Old solution
        //Optimal Scalar product values
        //public double distanceFromLeader;
        //public double angleProduct;
        //public double angle;
        
        //New Solution
        //Projection magnitude
        //public double max_height;
        //public double min_height;
        //public double max_width;
        //public double min_width;
        public Vector3 position;
        public Vector3 differenceVector;
        public Vector3 oldPos;
        public Vector3 currentPos;
        
        public CarPosition(Vector3 leaderPosition, Vector3 difference)
        {
            /*this.max_height = max_height;
            this.min_height = min_height;
            this.angle = angle;
            this.angleProduct = angleProduct;*/

            position = leaderPosition + difference;
            differenceVector = -difference;
            oldPos = position;
        }
    }

    public class CarControls
    {
        public float steering;
        public float acceleration;
        public float footBrake;
        public float handBrake;

        public CarControls(float steering, float acceleration, float footBrake, float handBrake)
        {
            this.steering = steering;
            this.acceleration = acceleration;
            this.footBrake = footBrake;
            this.handBrake = handBrake;
        }
    }
    
    
    
}