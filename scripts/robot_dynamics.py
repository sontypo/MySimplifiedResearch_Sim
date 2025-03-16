import numpy as np
import math

class RobotDynamics:
    # def __init__(self, total_mass=20.0, v_max_robot=2.5, tau_d=4.2953125, sigma=88.8623, Lambda=0.04375,
    #              alpha=83.518066, beta=2.806738, alpha_bound=90.0, beta_bound=2.5, boundary_dis=6.3, cam2com=0.2):
    def __init__(self, total_mass=20.0, v_max_robot=2.5, tau_d=1.2953125, sigma=48.8623, Lambda=0.04375,
                 alpha=200.518066, beta=1.806738, alpha_bound=100.0, beta_bound=3.5, boundary_dis=6.3, cam2com=0.2):
        # Robot state
        self.robot_pose = np.array([0.0, 0.0])  # [x, y]
        self.robot_velocity = np.array([0.0, 0.0])  # [v_x, v_y]
        self.theta = 0.0  # Orientation (radians)

        # Parameters
        self.total_mass = total_mass
        self.v_max_robot = v_max_robot
        self.tau_d = tau_d
        self.sigma = sigma
        self.Lambda = Lambda
        self.alpha = alpha
        self.beta = beta
        self.alpha_bound = alpha_bound
        self.beta_bound = beta_bound
        self.boundary_dis = boundary_dis
        self.cam2com = cam2com
        self.I_w = 0.5 * 1.0 * (0.08 ** 2)  # Inertia of the wheel
        self.I_b = 1.216  # Inertia of the base

        # Forces
        self.F_des = np.array([0.0, 0.0])
        self.F_soc = np.array([0.0, 0.0])
        self.F_bound = np.array([0.0, 0.0])
        self.F_total = np.array([0.0, 0.0])

        # Field of View (FOV) parameters
        self.fov_angle = np.radians(90)  # 90-degree FOV
        self.fov_range = 2.5  # Maximum detection range (meters)
        
        # Goal parameters
        self.goal_position = np.array([12.5, 0.0])  # Goal position
        self.goal_threshold = 0.5  # Stop when within 0.1 m of the goal
        self.goal_check = False # Whether the robot has reached the goal

    def gamma_function(self, Lambda, cos_phi_ij):
        """Compute anisotropic behavior factor."""
        return Lambda + 0.5 * (1 - Lambda) * (1 + cos_phi_ij)

    def calculate_distance(self, pos1, pos2):
        """Calculate Euclidean distance between two points."""
        return np.linalg.norm(pos2 - pos1)

    def is_in_fov(self, human_position):
        """Check if a human is within the robot's FOV."""
        # Vector from robot to human
        vec_to_human = human_position - self.robot_pose
        distance = self.calculate_distance(self.robot_pose, human_position)

        # Angle between robot's heading and vector to human
        angle_to_human = np.arctan2(vec_to_human[1], vec_to_human[0]) - self.theta
        angle_to_human = (angle_to_human + np.pi) % (2 * np.pi) - np.pi  # Normalize to [-pi, pi]

        # Check if human is within FOV
        return (abs(angle_to_human) <= self.fov_angle / 2) and (distance <= self.fov_range)

    def social_force_calc(self, human_positions, human_velocities):
        """Calculate social force based on humans within FOV."""
        self.F_soc = np.array([0.0, 0.0])
        for i, human_pose in enumerate(human_positions):
            if not self.is_in_fov(human_pose):
                continue  # Skip humans outside FOV

            vec_dis = human_pose - self.robot_pose
            distance = self.calculate_distance(self.robot_pose, human_pose)
            if distance < 0.2:  # Avoid division by zero
                continue
            vec_relative_vel = self.robot_velocity - human_velocities[i]
            if np.linalg.norm(vec_relative_vel) < 1e-6 or np.linalg.norm(-vec_dis) < 1e-6:
                cos_phi_ij = 0.0
            else:
                cos_phi_ij = np.dot(vec_relative_vel / np.linalg.norm(vec_relative_vel), vec_dis / np.linalg.norm(-vec_dis))
            Gamma = self.gamma_function(self.Lambda, cos_phi_ij)
            F_soc = Gamma * self.alpha * math.exp(-distance / self.beta) * (-vec_dis / distance) # to increase the magnitude of the social force, there are two ways to do it: 1. increase alpha, 2. increase beta
            self.F_soc += F_soc

    def boundary_force_calc(self):
        """Calculate boundary force based on distances to walls."""
        self.left_distance = self.boundary_dis / 2 - self.robot_pose[1]  # Distance to left wall
        self.right_distance = self.boundary_dis / 2 + self.robot_pose[1]  # Distance to right wall
        self.F_bound_L = np.array([0.0, self.alpha_bound * math.exp(-self.left_distance / self.beta_bound)])
        self.F_bound_R = np.array([0.0, self.alpha_bound * math.exp(-self.right_distance / self.beta_bound)])
        self.F_bound = self.F_bound_R - self.F_bound_L

    def update_dynamic(self, Fm, Fn, v_x, omega_z, dt):
        """Update robot dynamics based on applied forces."""
        Mn = np.array([[self.total_mass, 0], [0, self.total_mass * self.cam2com + self.I_w + self.I_b]])
        Cn = np.array([[0, -self.total_mass * self.cam2com * omega_z], [self.total_mass * self.cam2com * omega_z, 0]])
        Bn = np.array([[1, 0, 0], [0, self.cam2com, 1]])
        z_in = np.array([v_x, omega_z])
        Z_dot = np.dot(np.linalg.inv(Mn), -np.dot(Cn, z_in) + np.dot(Bn, np.array([Fm, Fn, 0])))
        z_in += Z_dot * dt
        return z_in

    def update(self, human_positions, human_velocities, dt):
        """Update the robot's state."""
        
         # Check if the robot is near the goal
        distance_to_goal = self.calculate_distance(self.robot_pose, self.goal_position)
        if distance_to_goal < self.goal_threshold:
            self.goal_check = True
            # Stop the robot
            self.robot_velocity = np.array([0.0, 0.0])
            return self.robot_pose, self.goal_check, np.array([0.0, 0.0]), {
                "F_des": np.array([0.0, 0.0]),
                "F_soc": np.array([0.0, 0.0]),
                "F_bound": np.array([0.0, 0.0]),
                "F_total": np.array([0.0, 0.0]),
            }
            
        # Calculate forces
        self.social_force_calc(human_positions, human_velocities)
        self.boundary_force_calc()

        # Compute desired velocity and force
        s_ = np.linalg.norm(self.F_soc) + np.linalg.norm(self.F_bound)
        if np.isnan(s_) or s_ < 1e-6:
            s_ = 0.0
        self.v_des = math.exp(-s_ / self.sigma) * np.array([self.v_max_robot, 0])
        self.F_des = self.total_mass * (self.v_des - self.robot_velocity) / self.tau_d

        # Total force
        self.F_total = self.F_des + self.F_soc + self.F_bound

        # Update dynamics
        new_velocity = self.update_dynamic(self.F_total[0], self.F_total[1], self.robot_velocity[0], 0, dt)
        self.robot_velocity = new_velocity
        self.robot_pose += self.robot_velocity * dt

        # Return robot position, force vector, and force components
        force_components = {
            "F_des": self.F_des,
            "F_soc": self.F_soc,
            "F_bound": self.F_bound,
            "F_total": self.F_total,
        }
        return self.robot_pose, self.goal_check, self.F_total, force_components