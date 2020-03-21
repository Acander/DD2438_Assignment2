using UnityEngine;
using UnityStandardAssets.Vehicles.Car;

namespace Scrips
{
    public class PController
    {
        //Position Controller for individual car
        private GameObject car;
        private GameObject leader;
        public CarPosition formationPosition;

        private double speedTolerance = 1;

        public PController(GameObject car, GameObject leader, CarPosition initPosition)
        {
            this.car = car;
            this.leader = leader;
            formationPosition = initPosition;
        }
        
        public CarControls getCarControls()
        {
            float magnitude;
            bool passed;
            bool is_to_the_right;

            
            passed = this.passed(car.transform.position, leader.transform.forward, formationPosition.position);
            Vector3 between = formationPosition.position - car.transform.position;
            is_to_the_right = Vector3.Dot(between, car.transform.right) < 0;
            magnitude = between.magnitude;
            Debug.DrawLine(formationPosition.position, car.transform.position);

            //float magnitudeCoefficient = 1f / magnitude;
            
            float acceleration = 0.05f*magnitude;
            float steering = 0.05f*magnitude;
            
            /*float acceleration = 1f;
            float steering = 1f;*/
            
            //CarControls controls = new CarControls(0, 0, 0, 0);

            if (magnitude < 10)
            {
                //acceleration = matchCarControls();
                acceleration = 0f;
                steering = 0.1f;
            }

            if (passed)
            {
                //acceleration = -acceleration;
                acceleration = 0;
                is_to_the_right = Vector3.Dot(leader.transform.forward, car.transform.right) < 0;
            }
            if (is_to_the_right)
            {
                steering = -steering;
            }

            return new CarControls(steering, acceleration, acceleration, 0);
            
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
        
        
    }
}