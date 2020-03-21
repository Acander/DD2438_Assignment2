using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Numerics;
using Scrips;
using Vector3 = UnityEngine.Vector3;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI5 : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use
        private float car_radius = 5f;

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;
        private TraversabilityManager traversabilityManager;

        public GameObject[] friends;
        public GameObject[] enemies;

        private int[] turretScores;
        private Vector3[] finalRoute;
        
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
            foreach (GameObject obj in enemies)
            {
                Debug.DrawLine(transform.position, obj.transform.position, Color.black, 10f);
            }

            calculateTurretScores();
            sortTurretList();
            
        }

        private void calculateTurretScores()
        {
            turretScores = new int[enemies.Length];
            
            for (int current = 0; current < turretScores.Length-1; current++)
            {
                for (int interest = current+1; interest < turretScores.Length; interest++)
                {
                    if (!outOfsight(friends[current].transform.position, friends[interest].transform.position))
                    {
                        turretScores[current]++;
                        turretScores[interest]++;
                    }
                }
            }
        }

        private void sortTurretList()
        {
            finalRoute = new Vector3[enemies.Length];
            for (int i = 0; i < turretScores.Length; i++)
            {
                int lowestIndex = 0;
                int lowest = int.MaxValue;
                for (int j = 0; j < turretScores.Length; j++)
                {
                    if (turretScores[j] < lowest)
                    {
                        lowestIndex = j;
                        lowest = turretScores[j];
                    }
                }
                turretScores[lowestIndex] = int.MaxValue;
                finalRoute[i] = enemies[lowestIndex].transform.position;
            }
        }

        private bool outOfsight(Vector3 currentPoint, Vector3 pointOfinterest)
        {
            Vector3 between = currentPoint - pointOfinterest;
            Vector3 betweenNorm = between.normalized;
            Vector3 currentBetweenPoint = currentPoint + (betweenNorm * car_radius);
            int i = 1;
            while (!finishedIterating(currentBetweenPoint, pointOfinterest))
            {
                currentBetweenPoint = currentPoint + (betweenNorm * i*car_radius);
                if (!traversabilityManager.PointTraversableCS(currentBetweenPoint.x, currentBetweenPoint.z))
                {
                    return true;
                }
                i++;
            }
            return false;
        }

        private bool finishedIterating(Vector3 point, Vector3 pointOfInterest)
        {
            return (point - pointOfInterest).magnitude < 3;
        }

        private void FixedUpdate()
        {


            /*// Execute your path here
            // ...

            Vector3 avg_pos = Vector3.zero;

            foreach (GameObject friend in friends)
            {
                avg_pos += friend.transform.position;
            }
            avg_pos = avg_pos / friends.Length;
            Vector3 direction = (avg_pos - transform.position).normalized;

            bool is_to_the_right = Vector3.Dot(direction, transform.right) > 0f;
            bool is_to_the_front = Vector3.Dot(direction, transform.forward) > 0f;

            float steering = 0f;
            float acceleration = 0;

            if (is_to_the_right && is_to_the_front)
            {
                steering = 1f;
                acceleration = 1f;
            }
            else if (is_to_the_right && !is_to_the_front)
            {
                steering = -1f;
                acceleration = -1f;
            }
            else if (!is_to_the_right && is_to_the_front)
            {
                steering = -1f;
                acceleration = 1f;
            }
            else if (!is_to_the_right && !is_to_the_front)
            {
                steering = 1f;
                acceleration = -1f;
            }

            // this is how you access information about the terrain
            int i = terrain_manager.myInfo.get_i_index(transform.position.x);
            int j = terrain_manager.myInfo.get_j_index(transform.position.z);
            float grid_center_x = terrain_manager.myInfo.get_x_pos(i);
            float grid_center_z = terrain_manager.myInfo.get_z_pos(j);

            Debug.DrawLine(transform.position, new Vector3(grid_center_x, 0f, grid_center_z));


            // this is how you control the car
            Debug.Log("Steering:" + steering + " Acceleration:" + acceleration);
            m_Car.Move(steering, acceleration, acceleration, 0f);
            //m_Car.Move(0f, -1f, 1f, 0f);*/


        }
    }
}
