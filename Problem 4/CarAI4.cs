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
    public class CarAI4 : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use
        private float car_radius = 5f;

        private double inner_height = 5;
        private double outer_height = 10;
        private double inner_width = 2;
        private double outer_width = 4;
        private float standardAcceleration = 0.1f;
        private GameObject parent;

        private CarFormation vShape;
        private CarFormation sweepShape;
        private CarFormation currentFormation;

        public GameObject terrain_manager_game_object;
        TerrainManager terrain_manager;
        private TraversabilityManager traversabilityManager;

        public GameObject[] friends;
        public GameObject[] enemies;

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
            
            
            //Init V-formation 
            
            //Car 1 (Inner left)
            CarPosition innerLeft = new CarPosition(friends[0].transform.position, new Vector3(-8, 0,  -12));

            //Car 2 (Inner right)
            CarPosition innerRight = new CarPosition(friends[0].transform.position, new Vector3(8, 0, -12));
            
            //Car 3 (Outer left)
            CarPosition outerLeft = new CarPosition(friends[0].transform.position, new Vector3(-16, 0, -24));
            
            //Car 4 (Outer right)
            CarPosition outerRight = new CarPosition(friends[0].transform.position, new Vector3(16, 0, -24));
            
            vShape = new CarFormation(friends, innerLeft, innerRight, outerLeft, outerRight);

            //Init Sweep-formation
            //Car 1 (Inner left)
            innerLeft = new CarPosition(friends[0].transform.position, new Vector3(-16, 0,  0));

            //Car 2 (Inner right)
            innerRight = new CarPosition(friends[0].transform.position, new Vector3(16, 0,  0));
            
            //Car 3 (Outer left)
            outerLeft = new CarPosition(friends[0].transform.position, new Vector3(-32, 0,  0));
            
            //Car 4 (Outer right)
            outerRight = new CarPosition(friends[0].transform.position, new Vector3(32, 0,  0));
            
            sweepShape = new CarFormation(friends, innerLeft, innerRight, outerLeft, outerRight);

            currentFormation = vShape;
        }
        
        
        private void FixedUpdate()
        {

            CarControls carControls = currentFormation.getCarControls(gameObject);
            
            m_Car.Move(carControls.steering, carControls.acceleration, carControls.acceleration, 0f);


        }

        private void OnDrawGizmos()
        {
            /*Gizmos.color = Color.cyan;
            Debug.Log(currentFormation.innerLeft.formationPosition.position);
            Gizmos.DrawSphere(currentFormation.innerLeft.formationPosition.position, 3f);
            Gizmos.DrawSphere(currentFormation.innerRight.formationPosition.position, 3f);
            Gizmos.DrawSphere(currentFormation.outerLeft.formationPosition.position, 3f);
            Gizmos.DrawSphere(currentFormation.outerRight.formationPosition.position, 3f);*/
        }
    }
}
