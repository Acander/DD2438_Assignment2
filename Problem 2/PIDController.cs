using System;
using System.Collections.Generic;
using System.Runtime.InteropServices;
using Scrips;
using TMPro;
using UnityEngine;

namespace KruskalMinimumSpanningTree
{
    public class PIDController
    {
        // Path
        List<Vector3> path;
        public int current = 1;

        public Vector3 oldPos; 

        // Keep track
        float CTE_old = 0f;
        float CTE_sum = 0f;

        //PID parameters
        public float tau_P = 70f;
        public float tau_I = 0.01f;
        public float tau_D = 20f;

        // CTE tolerance for no steering
        float CTE_tol = 2f;

        // Max steering angle in degrees
        float max_steering;

        private const float FullThrottle = 1f;
        private const float FullBrake = -1f; 
        private const float FullHandbrake = 1f;

        private float tolerance = 70f;

        public PIDController(List<Vector3> path, float max_steering)
        {
            this.path = path;
            this.max_steering = max_steering;
            oldPos = path[0];
        }

        public float get_controls(Vector3 pos, Vector3 right)
        {
            // Reached end of path
            if (current == path.Count)
            {
                return 2;
            }

            // Check if passed current goal
            var start_pos = path[current - 1];
            var end_pos = path[current];
            //Debug.LogFormat("start_pos: {0}", start_pos);
            //Debug.LogFormat("end_pos: {0}", end_pos);
            if (passed(pos, start_pos, end_pos))
            {
                //Debug.Log("Passed a point");
                //Debug.Log(current);
                current += 1;
                
                //Debug.LogFormat("Current waypoint: {0}", path[current]);
                return 0;
            }

            // CTE
            float CTE = get_CTE(pos, start_pos, end_pos);
            //UnityEngine.Debug.Log("CTE = " + CTE);
            // Steer direction
            var dir = steer_dir(pos, right, end_pos);
            CTE *= dir;
            // Steer angle
            float steer_angle = get_steer_angle(CTE);
            // Clamp and normalise to [-1, 1]
            steer_angle = Mathf.Clamp(steer_angle, -max_steering, max_steering) / max_steering;

            return steer_angle;
        }

        public float get_steer_angle(float CTE)
        {
            // P
            float P = tau_P * CTE;

            //I
            CTE_sum += Time.fixedDeltaTime * CTE;
            //Sometimes better to just sum and average the last errors instead of integrating
            // TODO: CTE_sum -> inf? Perhaps use linked-list of length 20 instead
            float averageAmount = 20f;
            CTE_sum += (CTE - CTE_sum) / averageAmount;
            float I = tau_I * CTE_sum;

            //D
            float d_dt_CTE = (CTE - CTE_old) / Time.fixedDeltaTime;
            float D = tau_D * d_dt_CTE;
            // For D calculation
            CTE_old = CTE;

            //UnityEngine.Debug.Log("P + D + I = " + (P + D + I));
            //Debug.LogFormat("PID score: {0}", Mathf.Abs(P+D+I));
            if (Mathf.Abs(P + D + I) < tolerance)
            {
                return 0;
            }
            
            return P + D + I;
            
        }

        public float steer_dir(Vector3 pos, Vector3 right, Vector3 end_pos)
        {
            var dir = end_pos - pos;
            float dot = Vector3.Dot(right, dir);
            return dot > 0f ? 1f : -1f;
        }

        public bool passed(Vector3 pos, Vector3 start_pos, Vector3 end_pos)
        {
            var between = end_pos - start_pos;
            var progress = pos - start_pos;
            var projection = Vector3.Project(progress, between);
            return projection.magnitude >= between.magnitude;
        }

        public float get_CTE(Vector3 pos, Vector3 start_pos, Vector3 end_pos)
        {
            var between = end_pos - start_pos;
            var progress = pos - start_pos;
            var projection = Vector3.Project(progress, between);
            return (projection - progress).magnitude;
        }

        public Vector3 get_progress_point(Vector3 pos)
        {
            var start_pos = path[current - 1];
            var end_pos = path[current];
            var between = end_pos - start_pos;
            var progress_pos = pos - start_pos;
            return start_pos + Vector3.Project(progress_pos, between);
        }

        public NextMove reverse_Routine(float forwardVelocity, Vector3 currentPos, Vector3 right)
        {
            //Debug.Log("Reversing!!!!!!!!!!!!!!!!");
            float steeringAngle = steer_dir(currentPos, right, path[current]);
            if (forwardVelocity > 0)
            {
                return new NextMove(steeringAngle, 0, FullBrake, 0f);
            }
            //Debug.Log("Reversing!!!!!!!!!!!!!!!!");
            return new NextMove(-steeringAngle, 0, FullBrake, 0f);
            //return new NextMove(0, -1f, 0f, 0f);
        }
        
        public Boolean check_Should_Reverse(Vector3 currentPos, Vector3 carHeading)
        {
            Debug.DrawLine(currentPos, path[current], Color.white);
            float reverseScore = Vector3.Dot(path[current]-currentPos, carHeading);
            //Debug.LogFormat("Reverse score: {0}", reverseScore);
            //Debug.LogFormat("Carheading: {0} ", carHeading);
            return reverseScore < 0;
        }
    }
}