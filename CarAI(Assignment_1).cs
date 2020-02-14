using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System;
using System.Linq;
using System.Text;
using System.Diagnostics;


namespace UnityStandardAssets.Vehicles.Car
{
    [RequireComponent(typeof(CarController))]
    public class CarAI : MonoBehaviour
    {
        private CarController m_Car; // the car controller we want to use

        public GameObject terrain_manager_game_object;

        // Get information about terrain
        TerrainManager terrain_manager;

        // Get information about terrain
        TraversabilityManager traversability_manager;

        // Car information
        float car_radius = 3f;
        float L = 2.870426f;
        float L_f;
        float L_b;
        float max_throttle = 1f;
        float max_v = 8;
        Vector3 fa_mid;
        Vector3 ra_mid;

        // Start and goal positions
        Vector3 start_pos;
        Vector3 goal_pos;

        // Path
        List<Node> my_path;

        // PID Controller
        PIDController pid_controller;

        // Timing
        Stopwatch stopwatch;

        void OnDrawGizmos()
        {
            // Draw traverability
            for (int i = 0; i < traversability_manager.n_x; i++)
            {
                for (int j = 0; j < traversability_manager.n_z; j++)
                {
                    float x = traversability_manager.get_x_pos(i), z = traversability_manager.get_z_pos(j);
                    Vector3 center = new Vector3(x, 0, z);
                    Gizmos.color = Color.blue;
                    Gizmos.DrawWireCube(center, new Vector3(traversability_manager.grid_x, 0, traversability_manager.grid_z));
                    if (!traversability_manager.traversableCS[i, j])
                    {
                        Gizmos.color = Color.red;
                    }
                    Gizmos.DrawSphere(center, 0.3f);
                }
            }

            // Draw path and trajectories along path
            foreach (var node in my_path)
            {
                Gizmos.color = Color.yellow;
                foreach (var t in node.trajectory)
                {
                    Gizmos.DrawSphere(t, 0.2f);
                }
                Gizmos.color = Color.green;
                Gizmos.DrawWireSphere(node.fa_pos, 0.3f);
            }

            // Draw car collider rectangle used for CS
            Gizmos.color = Color.yellow;
            Gizmos.DrawWireCube(transform.position, new Vector3(2 * car_radius, 0, 2 * car_radius));

            // Draw car center
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(transform.position, 0.2f);

            //Draw front axel mid
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(fa_mid, 0.2f);

            // Draw rear axel mid
            Gizmos.color = Color.red;
            Gizmos.DrawSphere(ra_mid, 0.2f);

            // Draw start and goal positions
            //Gizmos.color = Color.yellow;
            //Gizmos.DrawSphere(start_pos, 0.5f);
            //Gizmos.DrawSphere(goal_pos, 0.5f);
        }

        private void Start()
        {
            // Get Car and TerrainManager
            m_Car = GetComponent<CarController>();
            terrain_manager = terrain_manager_game_object.GetComponent<TerrainManager>();

            // Start and goal position vectors
            start_pos = terrain_manager.myInfo.start_pos;
            goal_pos = terrain_manager.myInfo.goal_pos;

            // Create traversability manager from TerrainManager
            traversability_manager = new TraversabilityManager(terrain_manager, car_radius, goal_pos);

            // Make sure goal position doesn't overlap with CSO
            goal_pos = traversability_manager.get_closest_traversable(goal_pos);

            if (!traversability_manager.PointTraversableCS(start_pos.x, start_pos.z))
            {
                UnityEngine.Debug.Log("Start overlaps with CSO!");
            }

            if (!traversability_manager.PointTraversableCS(goal_pos.x, goal_pos.z))
            {
                UnityEngine.Debug.Log("Goal overlaps with CSO!");
            }

            var theta = transform.eulerAngles.y;
            fa_mid = transform.position + Quaternion.Euler(0, theta, 0) * Vector3.forward * (L / 2);
            // Start on ground
            fa_mid.y = 0;

            // Create start and goal Nodes 
            var start = new Node(new DiscreteConfig(traversability_manager.get_i_index(fa_mid.x), traversability_manager.get_j_index(fa_mid.z), 0), fa_mid, 0f, 0f, 0f, new List<Vector3>()); ;
            var goal = new Node(new DiscreteConfig(traversability_manager.get_i_index(goal_pos.x), traversability_manager.get_j_index(goal_pos.z), 0), goal_pos, 0f, 0f, 0f, new List<Vector3>());

            // Acceleration used for model 
            my_path = new PathFinder().FindPath(traversability_manager.traversableCS, start, goal, traversability_manager, m_Car.m_MaximumSteerAngle * Mathf.Deg2Rad, L, max_v);

            if (my_path.Count == 0)
            {
                UnityEngine.Debug.Log("Unable to find path");
            }

            // Create PID controller for charted path
            pid_controller = new PIDController(my_path, m_Car.m_MaximumSteerAngle);

            // Time lap
            stopwatch = new Stopwatch();
            stopwatch.Start();
        }


        private void FixedUpdate()
        {
            var theta = transform.eulerAngles.y;
            fa_mid = transform.position + Quaternion.Euler(0, theta, 0) * Vector3.forward * (L / 2);
            ra_mid = transform.position - Quaternion.Euler(0, theta, 0) * Vector3.forward * (L / 2);
            var act_fa_mid = m_Car.m_WheelMeshes[0].transform.position + (m_Car.m_WheelMeshes[1].transform.position - m_Car.m_WheelMeshes[0].transform.position) / 2;
            var act_ra_mid = m_Car.m_WheelMeshes[2].transform.position + (m_Car.m_WheelMeshes[3].transform.position - m_Car.m_WheelMeshes[2].transform.position) / 2;

            if (pid_controller.current < my_path.Count)
            {
                // Draw line from front axel mid to target
                var target = my_path[pid_controller.current];
                var target_vec = target.fa_pos;
                UnityEngine.Debug.DrawLine(fa_mid, target_vec, Color.blue);

                // Draw CTE line
                var progress = pid_controller.get_progress_point(fa_mid);
                UnityEngine.Debug.DrawLine(fa_mid, progress, Color.red);

                var throttle = max_throttle;
                var target_vel = target.v;
                var velocity = GetComponent<Rigidbody>().velocity.magnitude;
                if (velocity > target_vel)
                {
                    throttle = 0;
                }

                var steer = pid_controller.get_controls(fa_mid, transform.right);
                //UnityEngine.Debug.Log("steer = " + steer);
                m_Car.Move(steer, throttle, 0f, 0f);
            }
            else
            {
                if (stopwatch.IsRunning)
                {
                    stopwatch.Stop();
                    UnityEngine.Debug.Log(String.Format("Finished in {0:00} seconds", stopwatch.Elapsed.Seconds));
                }
                m_Car.Move(0, 0, 1f, 1f);
            }
        }
    }

    public class PIDController
    {
        // Path
        List<Vector3> path;
        public int current = 1;

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

        public PIDController(List<Node> nodes, float max_steering)
        {
            path = nodes.Select(node => node.fa_pos).ToList();
            this.max_steering = max_steering;
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
            if (passed(pos, start_pos, end_pos))
            {
                current++;
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
            return projection.magnitude > between.magnitude;
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
    }

    public class TraversabilityManager
    {
        TerrainManager terrain_manager;

        // CS grid and x-axis, z-axis
        public bool[,] traversableCS;
        float x_low;
        float x_high;
        float z_low;
        float z_high;
        float x_range;
        float z_range;
        public int n_x;
        public int n_z;

        // CS grid parameters
        public float grid_x;
        public float grid_z;
        public float grid_hype;

        float car_radius;

        // Traversable point closest to goal_pos
        public Vector3 traversable_goal_pos;

        public TraversabilityManager(TerrainManager terrain_manager, float car_radius, Vector3 goal_pos)
        {
            this.terrain_manager = terrain_manager;
            this.car_radius = car_radius;

            // Get variables from TerrainManager
            x_low = terrain_manager.myInfo.x_low;
            x_high = terrain_manager.myInfo.x_high;
            z_low = terrain_manager.myInfo.z_low;
            z_high = terrain_manager.myInfo.z_high;
            x_range = x_high - x_low;
            z_range = z_high - z_low;

            // Determine appropriate grid_size
            var x_res = x_range / terrain_manager.myInfo.x_N;
            var z_res = z_range / terrain_manager.myInfo.z_N;
            // We want grid of to be ≈ 3 x 3
            grid_x = determine_grid_size(x_res, 2);
            grid_z = determine_grid_size(z_res, 2);

            // Get grid hypotenuse
            grid_hype = Convert.ToSingle(Math.Sqrt(Math.Pow(grid_x, 2) + Math.Pow(grid_z, 2)));


            // Create traversable points
            n_x = (int)Mathf.Floor(x_range / grid_x);
            n_z = (int)Mathf.Floor(z_range / grid_z);

            // Build CS traversability matrix
            traversableCS = new bool[n_x, n_z];
            for (int i = 0; i < n_x; i++)
            {
                for (int j = 0; j < n_z; j++)
                {
                    float x = get_x_pos(i), z = get_z_pos(j);
                    if (PointTraversableCS(x, z))
                    {
                        traversableCS[i, j] = true;
                    }
                }
            }
        }

        public float determine_grid_size(float res, float desired_grid_size)
        {
            var upper = Mathf.Ceil(res / desired_grid_size);
            var lower = Mathf.Floor(res / desired_grid_size);
            var larger = res / lower;
            var smaller = res / upper;
            var diff_larger = Mathf.Abs(desired_grid_size - larger);
            var diff_smaller = Mathf.Abs(desired_grid_size - smaller);
            return (diff_larger > diff_smaller) ? smaller : larger;
        }

        public Vector3 get_closest_traversable(Vector3 point)
        {
            Vector3 traversable = new Vector3(get_x_pos(0), 0, get_z_pos(0));
            var min_dist = double.MaxValue;
            for (int i = 0; i < n_x; i++)
            {
                for (int j = 0; j < n_z; j++)
                {
                    float x = get_x_pos(i), z = get_z_pos(j);
                    var _point = new Vector3(x, 0, z);
                    var dist = Vector3.Distance(_point, point);
                    if (PointTraversableCS(x, z) && dist < min_dist)
                    {
                        min_dist = dist;
                        traversable = _point;
                    }
                }
            }
            return traversable;
        }

        public float get_x_pos(int i)
        {
            return x_low + (grid_x * i) + (grid_x / 2f);
        }

        public float get_z_pos(int j)
        {
            return z_low + (grid_z * j) + (grid_z / 2f);
        }

        public int get_i_index(float x)
        {
            int index = (int)Mathf.Floor(n_x * (x - x_low) / x_range);
            if (index < 0)
            {
                index = 0;
            }
            else if (index > n_x - 1)
            {
                index = n_x - 1;
            }
            return index;

        }

        public int get_j_index(float z)
        {
            int index = (int)Mathf.Floor(n_z * (z - z_low) / z_range);
            if (index < 0)
            {
                index = 0;
            }
            else if (index > n_z - 1)
            {
                index = n_z - 1;
            }
            return index;
        }

        public bool PointTraversableCS(float x, float z)
        {
            return terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index(x), terrain_manager.myInfo.get_j_index(z)] == 0
                && terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index(x), terrain_manager.myInfo.get_j_index(z + car_radius)] == 0
                && terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index(x + car_radius), terrain_manager.myInfo.get_j_index(z + car_radius)] == 0
                && terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index(x + car_radius), terrain_manager.myInfo.get_j_index(z)] == 0
                && terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index(x + car_radius), terrain_manager.myInfo.get_j_index(z - car_radius)] == 0
                && terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index(x), terrain_manager.myInfo.get_j_index(z - car_radius)] == 0
                && terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index(x - car_radius), terrain_manager.myInfo.get_j_index(z - car_radius)] == 0
                && terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index(x - car_radius), terrain_manager.myInfo.get_j_index(z)] == 0
                && terrain_manager.myInfo.traversability[terrain_manager.myInfo.get_i_index(x - car_radius), terrain_manager.myInfo.get_j_index(z + car_radius)] == 0;
        }
    }

    public class PathFinder
    {

        Dictionary<Node, bool> closedSet = new Dictionary<Node, bool>();
        Dictionary<Node, bool> openSet = new Dictionary<Node, bool>();

        //cost of start to this key node
        Dictionary<Node, float> gScore = new Dictionary<Node, float>();
        //cost of start to goal, passing through key node
        Dictionary<Node, double> fScore = new Dictionary<Node, double>();

        Dictionary<Node, Node> nodeLinks = new Dictionary<Node, Node>();

        // Heading resolution
        double heading_resolution = Math.PI / 4;
        int ticks;

        // Traversability manager
        TraversabilityManager traversability;
        double d;

        // Car
        float L;
        float max_steering;
        float dt = Time.fixedDeltaTime;
        float max_v;

        public List<Node> FindPath(bool[,] graph, Node start, Node goal, TraversabilityManager traversability, float max_steering, float L, float max_v)
        {
            this.traversability = traversability;
            this.max_steering = max_steering * 0.9f;
            this.L = L;
            this.max_v = max_v;
            d = traversability.grid_hype + 0.01;
            ticks = Convert.ToInt32(Math.Round(2 * Math.PI / heading_resolution));

            openSet[start] = true;
            gScore[start] = 0;
            fScore[start] = Heuristic(start, goal);

            while (openSet.Count > 0)
            {
                var current = nextBest();
                if (current.config.i == goal.config.i && current.config.j == goal.config.j)
                {
                    var path = Reconstruct(current);
                    path.Insert(0, start);
                    return path; 
                }


                openSet.Remove(current);
                closedSet[current] = true;


                foreach (var neighbor in Neighbors(graph, current))
                {
                    if (closedSet.ContainsKey(neighbor))
                        continue;

                    // Penalise changing steering
                    var penalise_steer = Convert.ToSingle(neighbor.steer != 0f) * 10;
                    var projectedG = getGScore(current) + penalise_steer;

                    if (!openSet.ContainsKey(neighbor))
                        openSet[neighbor] = true;
                    else if (projectedG >= getGScore(neighbor))
                        continue;

                    //record it
                    nodeLinks[neighbor] = current;
                    gScore[neighbor] = projectedG;
                    fScore[neighbor] = projectedG + Heuristic(neighbor, goal);

                }
            }


            return new List<Node>();
        }

        private double Heuristic(Node start, Node goal)
        {
            var di = goal.config.i - start.config.i;
            var dj = goal.config.j - start.config.j;
            return Math.Sqrt(Math.Pow(di, 2) + Math.Pow(dj, 2));
        }

        private float getGScore(Node pt)
        {
            float score = float.MaxValue;
            gScore.TryGetValue(pt, out score);
            return score;
        }


        private double getFScore(Node pt)
        {
            double score = double.MaxValue;
            fScore.TryGetValue(pt, out score);
            return score;
        }

        public IEnumerable<Node> Neighbors(bool[,] graph, Node node)
        {
            // Controls
            var controls = new List<float>() { -1f, 0f, 1f };
            foreach (var steer in controls)
            {
                // Move in trajectory using rear axel mid
                var v = max_v;
                var phi = steer * max_steering;
                var theta = node.theta;
                var fa_pos = node.fa_pos;
                var center_start_pos = fa_pos - Quaternion.Euler(0, theta, 0) * Vector3.forward * (L / 2);
                var center_pos = center_start_pos;
                var ra_start_pos = center_pos - Quaternion.Euler(0, theta, 0) * Vector3.forward * (L / 2);
                var ra_pos = ra_start_pos;
                var trajectory = new List<Vector3>();
                while (Vector3.Distance(center_start_pos, center_pos) < d)
                {
                    var dx = dt * v * Mathf.Sin(theta);
                    var dz = dt * v * Mathf.Cos(theta);
                    var dTheta = dt * (v / L) * Mathf.Tan(phi);
                    ra_pos += new Vector3(dx, 0, dz);
                    center_pos = ra_pos + Quaternion.Euler(0, theta, 0) * Vector3.forward * (L / 2);
                    fa_pos = center_pos + Quaternion.Euler(0, theta, 0) * Vector3.forward * (L / 2);
                    theta += dTheta;
                    // Log front axel position in trajectory
                    trajectory.Add(fa_pos);
                }
                // Check traversability using center pos
                var i = traversability.get_i_index(center_pos.x);
                var j = traversability.get_j_index(center_pos.z);
                var tick = mod(Convert.ToInt32(theta / heading_resolution), ticks);
                // Log front axel position
                var new_pt = new Node(new DiscreteConfig(i, j, tick), fa_pos, theta, v, steer, trajectory);
                if (IsValidNeighbor(graph, new_pt))
                {
                    yield return new_pt;
                }
            }
        }

        int mod(int x, int m)
        {
            int r = x % m;
            return r < 0 ? r + m : r;
        }

        public bool IsValidNeighbor(bool[,] matrix, Node pt)
        {
            int i = pt.config.i;
            int j = pt.config.j;
            if (i < 0 || i >= matrix.GetLength(1))
                return false;

            if (j < 0 || j >= matrix.GetLength(0))
                return false;

            return matrix[i, j];

        }

        private List<Node> Reconstruct(Node current)
        {
            List<Node> path = new List<Node>();
            while (nodeLinks.ContainsKey(current))
            {
                path.Add(current);
                current = nodeLinks[current];
            }

            path.Reverse();
            return path;
        }

        private Node nextBest()
        {
            double best = double.MaxValue;
            Node bestPt = openSet.Keys.First();
            foreach (var node in openSet.Keys)
            {
                var score = getFScore(node);
                if (score < best)
                {
                    bestPt = node;
                    best = score;
                }
            }

            return bestPt;
        }

    }

    public class Node
    {
        // Discrete configuration
        public DiscreteConfig config;

        // Continuous configuration
        // Car front axel center position
        public Vector3 fa_pos;
        public float theta, v;

        // Controls
        public float steer;

        public List<Vector3> trajectory;

        public Node(DiscreteConfig config, Vector3 fa_pos, float theta, float v, float steer, List<Vector3> trajectory)
        {
            this.config = config;
            this.fa_pos = fa_pos;
            this.theta = theta;
            this.v = v;
            this.steer = steer;
            this.trajectory = trajectory;
        }

        public override int GetHashCode()
        {
            return config.GetHashCode();
        }

        public override bool Equals(object obj)
        {
            Node other = obj as Node;
            return other != null && other.config.Equals(config);
        }

        public override string ToString()
        {
            return String.Format("(Node: (Discrete Config: {0}, {1}, {2}) (Continuous Config: {3}, {4}, {5}), (Control: {6}))", config.i, config.j, config.tick, fa_pos.ToString(), theta, v, steer);
        }
    }

    public struct DiscreteConfig
    {
        public int i, j, tick;
        public DiscreteConfig(int i, int j, int tick) { this.i = i; this.j = j; this.tick = tick; }
    }
}
