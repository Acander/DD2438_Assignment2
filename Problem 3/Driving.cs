using System.Diagnostics;
using KruskalMinimumSpanningTree;
using UnityEngine;
using Debug = System.Diagnostics.Debug;

namespace Scrips
{
    public class Driving
    {
        /*public static void driveCar(Stopwatch stopwatch, PIDController pid_controller, Transform transform)
        {
            if (pid_controller.check_Should_Reverse(transform.position, transform.forward) || collision)
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
                NextMove nextMove = pid_controller.reverse_Routine(forwardSpeed, transform.position, transform.right);
                m_Car.Move(nextMove.steeringAngle, nextMove.throttle, nextMove.footBrake, nextMove.handBrake);
            }
            else
            {
                Debug.Log("Heading for next target!!!!!!!!!!!!");
                var theta = transform.eulerAngles.y;
                fa_mid = transform.position + Quaternion.Euler(0, theta, 0) * Vector3.forward * (L / 2);
                ra_mid = transform.position - Quaternion.Euler(0, theta, 0) * Vector3.forward * (L / 2);
                var act_fa_mid = m_Car.m_WheelMeshes[0].transform.position +
                                 (m_Car.m_WheelMeshes[1].transform.position -
                                  m_Car.m_WheelMeshes[0].transform.position) /
                                 2;
                var act_ra_mid = m_Car.m_WheelMeshes[2].transform.position +
                                 (m_Car.m_WheelMeshes[3].transform.position -
                                  m_Car.m_WheelMeshes[2].transform.position) /
                                 2;


                // Draw line from front axel mid to target
                var target = final_path[pid_controller.current];
                var target_vec = target;
                Debug.DrawLine(fa_mid, target_vec, Color.blue);

                // Draw CTE line
                var progress = pid_controller.get_progress_point(fa_mid);
                //Debug.DrawLine(fa_mid, progress, Color.red);

                //Debug.DrawLine(fa_mid, final_path[pid_controller.current - 1], Color.green);

                var throttle = max_throttle;
                var target_vel = 10f; //2f
                var velocity = GetComponent<Rigidbody>().velocity.magnitude;
                if (velocity > target_vel)
                {
                    throttle = 0;
                }

                //Debug.LogFormat("Here am I now: {0}", transform.position);
                var steer = pid_controller.get_controls(fa_mid, transform.right);
                //UnityEngine.Debug.Log("steer = " + steer);
                m_Car.Move(steer, throttle, 0f, 0f);
            }
        }*/
    }
}