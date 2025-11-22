import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry, Path
import casadi as ca
import numpy as np
import math

class NMPCController(Node):
    def __init__(self):
        super().__init__('nmpc_controller')
        
        # ROS Publishers & Subscribers
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.path_pub = self.create_publisher(Path, '/global_plan', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        
        self.timer = self.create_timer(0.1, self.control_loop) # 10Hz

        # Robot State
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        
        # NMPC Parameters
        self.T = 0.1   # Time step
        self.N = 20    # Prediction horizon steps
        self.v_max = 0.5
        self.w_max = 1.0
        
        self.setup_optimization()
        self.get_logger().info("NMPC Solver Initialized with CasADi (Obstacle Avoidance Enabled)")

    def setup_optimization(self):
        # 1. State variables
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        states = ca.vertcat(x, y, theta)
        n_states = states.numel()

        # 2. Control variables
        v = ca.SX.sym('v')
        omega = ca.SX.sym('omega')
        controls = ca.vertcat(v, omega)
        n_controls = controls.numel()

        # 3. System Dynamics (Unicycle Model)
        # x' = v * cos(theta)
        # y' = v * sin(theta)
        # theta' = omega
        rhs = ca.vertcat(v * ca.cos(theta), v * ca.sin(theta), omega)
        f = ca.Function('f', [states, controls], [rhs])

        # 4. Optimization Variables
        U = ca.SX.sym('U', n_controls, self.N) # Control inputs over horizon
        P = ca.SX.sym('P', n_states + n_states * self.N) # Parameters (Initial State + Reference Trajectory)
        X = ca.SX.sym('X', n_states, self.N+1) # Predicted states

        # 5. Cost Function & Constraints
        obj = 0  # Objective function (minimize this)
        g = []   # Constraints vector

        st = X[:, 0]
        g = ca.vertcat(g, st - P[:3]) # Initial state constraint

        # Weight matrices
        Q = np.diag([5.0, 5.0, 0.5]) # Penalize x, y, theta error
        R = np.diag([0.1, 0.05])     # Penalize control effort

        for k in range(self.N):
            st = X[:, k]
            con = U[:, k]
            
            # Reference state for this step (from Parameters P)
            # P structure: [Init_X, Init_Y, Init_Th, Ref_X0, Ref_Y0, Ref_Th0, Ref_X1...]
            ref_idx = n_states + k*n_states
            st_ref = P[ref_idx : ref_idx + n_states]

            # Cost Calculation (Tracking Error + Control Effort)
            state_err = st - st_ref
            obj = obj + ca.mtimes([state_err.T, Q, state_err]) + ca.mtimes([con.T, R, con])

            # --- OBSTACLE AVOIDANCE LOGIC ---
            # Define obstacle at x=1.0, y=0.0
            obs_x, obs_y = 1.0, 0.0
            
            # Calculate distance squared from robot (st[0]=x, st[1]=y) to obstacle
            dist_sq = (st[0] - obs_x)**2 + (st[1] - obs_y)**2
            
            # Add a "repulsive" cost. The closer the robot, the higher the cost.
            # 0.1 is added to prevent division by zero.
            obj = obj + 20.0 / (dist_sq + 0.1)
            # --------------------------------

            # Dynamics Constraint (Next state must follow physics)
            st_next = X[:, k+1]
            k1 = f(st, con)
            st_next_euler = st + self.T * k1
            g = ca.vertcat(g, st_next - st_next_euler)

        # 6. Define NLP Solver
        OPT_variables = ca.vertcat(ca.reshape(X, -1, 1), ca.reshape(U, -1, 1))
        nlp_prob = {'f': obj, 'x': OPT_variables, 'g': g, 'p': P}
        
        opts = {
            'ipopt.print_level': 0, 
            'print_time': 0,
            'ipopt.sb': 'yes',
            'ipopt.max_iter': 2000
        }
        self.solver = ca.nlpsol('solver', 'ipopt', nlp_prob, opts)
        
        # Limits
        self.lbg = [0.0] * ((n_states)*(self.N+1)) # Equality constraints = 0
        self.ubg = [0.0] * ((n_states)*(self.N+1))
        
        # Variable bounds (Velocities)
        # Structure of vars: [X0...Xn, U0...Un]
        n_x_vars = n_states * (self.N + 1)
        n_u_vars = n_controls * self.N
        
        self.lbx = [-ca.inf] * n_x_vars + [-self.v_max, -self.w_max] * self.N
        self.ubx = [ca.inf] * n_x_vars + [self.v_max, self.w_max] * self.N

    def odom_callback(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y
        # Quaternion to Euler (Yaw)
        q = msg.pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        self.theta = math.atan2(siny_cosp, cosy_cosp)

    def get_reference_trajectory(self):
        # Generate a Figure-8 trajectory shifting based on time
        t_start = self.get_clock().now().nanoseconds / 1e9
        refs = []
        
        # Create N reference points ahead
        for k in range(self.N):
            t = t_start + k * self.T
            # Figure 8 equations
            ref_x = 2.0 * math.sin(t * 0.5)
            ref_y = 2.0 * math.sin(t * 0.5) * math.cos(t * 0.5)
            
            # Approximate tangent angle
            dx = 2.0 * 0.5 * math.cos(t * 0.5)
            dy = 2.0 * 0.5 * (math.cos(t)*math.cos(0.5*t) - math.sin(t)*math.sin(0.5*t)) # rough approx
            ref_theta = math.atan2(dy, dx)
            
            refs.extend([ref_x, ref_y, ref_theta])
            
        return refs

    def control_loop(self):
        # 1. Build Parameter Vector (Current State + Reference Trajectory)
        current_state = [self.x, self.y, self.theta]
        traj_refs = self.get_reference_trajectory()
        
        # Visualize Reference Path (Optional, for Rviz)
        path_msg = Path()
        path_msg.header.frame_id = 'odom'
        for i in range(0, len(traj_refs), 3):
            pose = PoseStamped()
            pose.pose.position.x = float(traj_refs[i])
            pose.pose.position.y = float(traj_refs[i+1])
            path_msg.poses.append(pose)
        self.path_pub.publish(path_msg)

        # 2. Solve
        params = current_state + traj_refs
        args = {
            'p': params,
            'lbg': self.lbg,
            'ubg': self.ubg,
            'lbx': self.lbx,
            'ubx': self.ubx,
            'x0': [0]*((3*(self.N+1)) + (2*self.N)) # Warm start with zeros
        }
        
        try:
            sol = self.solver(**args)
            # Extract first control input
            u_opt = sol['x'][3*(self.N+1):] # Skip state vars
            v_cmd = float(u_opt[0])
            w_cmd = float(u_opt[1])
            
            # 3. Publish
            twist = Twist()
            twist.linear.x = v_cmd
            twist.angular.z = w_cmd
            self.cmd_pub.publish(twist)
            
        except Exception as e:
            self.get_logger().error(f"Solver failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = NMPCController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
