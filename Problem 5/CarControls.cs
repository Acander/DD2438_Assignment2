namespace Scrips
{
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