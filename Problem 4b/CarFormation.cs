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
        public PController innerLeft;

        //Car 2 (Inner right)
        public PController innerRight;
        
        //Car 3 (Outer left)
        public PController outerLeft;
        
        //Car 4 (Outer right)
        public PController outerRight;

        public CarFormation(GameObject[] friends, CarPosition innerLeft, CarPosition innerRight, CarPosition outerLeft, CarPosition outerRight, TraversabilityManager traversabilityManager)
        {
            leader = friends[0];
            lastLeaderPosition = new CarPosition(leader.transform.position, new Vector3(0, 0, 0));
            oldForwardVector = leader.transform.forward;
            Debug.LogFormat("Initial position {0}", lastLeaderPosition.position);
            this.innerLeft = new PController(friends[1], leader, innerLeft, traversabilityManager, friends);
            this.innerRight = new PController(friends[2], leader, innerRight, traversabilityManager, friends);
            this.outerLeft = new PController(friends[3], leader, outerLeft, traversabilityManager, friends);
            this.outerRight = new PController(friends[4], leader, outerRight, traversabilityManager, friends);
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
            innerLeft.formationPosition.oldPos = innerLeft.formationPosition.position;
            innerRight.formationPosition.oldPos = innerRight.formationPosition.position;
            outerLeft.formationPosition.oldPos = outerLeft.formationPosition.position;
            outerRight.formationPosition.oldPos = outerRight.formationPosition.position;
            
            innerLeft.formationPosition.differenceVector = Quaternion.Euler(0, angle, 0) * innerLeft.formationPosition.differenceVector;
            innerLeft.formationPosition.position = leaderPosition - innerLeft.formationPosition.differenceVector;;
            innerRight.formationPosition.differenceVector = Quaternion.Euler(0, angle, 0) * innerRight.formationPosition.differenceVector;
            innerRight.formationPosition.position = leaderPosition - innerRight.formationPosition.differenceVector;
            outerLeft.formationPosition.differenceVector = Quaternion.Euler(0, angle, 0) * outerLeft.formationPosition.differenceVector;
            outerLeft.formationPosition.position = leaderPosition - outerLeft.formationPosition.differenceVector;
            outerRight.formationPosition.differenceVector = Quaternion.Euler(0, angle, 0) * outerRight.formationPosition.differenceVector;
            outerRight.formationPosition.position = leaderPosition - outerRight.formationPosition.differenceVector;
            
            lastLeaderPosition.position = leader.transform.position;
            oldForwardVector = leader.transform.forward;

            

            CarControls controls = new CarControls(0f, 0f, 0f, 0f);
            if (friends[1].name == car.name)
            {
                controls = innerLeft.getCarControls();
            } 
            else if (friends[3].name == car.name)
            {
                controls = outerLeft.getCarControls();
            }
            else if (friends[2].name == car.name)
            {
                controls = innerRight.getCarControls();
            }
            else if (friends[4].name == car.name)
            {
                controls = outerRight.getCarControls();
            }

            //leader.GetComponent<CarController>().CurrentSpeed;
            
            return controls;
        }

        private bool isInnerCar(GameObject car)
        {
            return friends[1].name == car.name || friends[2].name == car.name;
        }
        

    }

}