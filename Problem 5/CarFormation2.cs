using System;
using UnityEngine;
using UnityEngine.SocialPlatforms.Impl;
using UnityStandardAssets.Vehicles.Car;

namespace Scrips
{
    //Formation class for 4 cars (not including a replay/leader car)
    
    public class CarFormation2
    {
        //Parameters calculated as dot products based on the leaders position
        
        //Leader
        private GameObject leader;
        private CarPosition lastLeaderPosition;
        private Vector3 oldForwardVector;
        
        //Other Cars
        private GameObject[] friends;
        
        //Car 1 (Inner left)
        public PController left;

        //Car 2 (Inner right)
        public PController right;

        public CarFormation2(GameObject[] friends, CarPosition left, CarPosition right)
        {
            leader = friends[0];
            lastLeaderPosition = new CarPosition(leader.transform.position, new Vector3(0, 0, 0));
            oldForwardVector = leader.transform.forward;
            Debug.LogFormat("Initial position {0}", lastLeaderPosition.position);
            this.left = new PController(friends[1], leader, left);
            this.right= new PController(friends[2], leader, right);
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
            left.formationPosition.oldPos = left.formationPosition.position;
            right.formationPosition.oldPos = right.formationPosition.position;

            left.formationPosition.differenceVector = Quaternion.Euler(0, angle, 0) * left.formationPosition.differenceVector;
            left.formationPosition.position = leaderPosition - left.formationPosition.differenceVector;;
            right.formationPosition.differenceVector = Quaternion.Euler(0, angle, 0) * right.formationPosition.differenceVector;
            right.formationPosition.position = leaderPosition - right.formationPosition.differenceVector;

            lastLeaderPosition.position = leader.transform.position;
            oldForwardVector = leader.transform.forward;

            CarControls controls = new CarControls(0f, 0f, 0f, 0f);
            if (friends[1].name == car.name)
            {
                controls = left.getCarControls();
            }
            else if (friends[2].name == car.name)
            {
                controls = right.getCarControls();
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