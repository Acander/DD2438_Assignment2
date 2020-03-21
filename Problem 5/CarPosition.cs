using UnityEngine;

namespace Scrips
{
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
}