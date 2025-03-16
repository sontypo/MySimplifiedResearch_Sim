import time
from environment import Environment
from robot_dynamics import RobotDynamics

def main():
    # Initialize environment and robot
    env = Environment()
    robot = RobotDynamics()

    # Simulation loop
    dt = 0.025  # Time step
    
    time_elapsed = 0.0
    for _ in range(1000):  # Simulate for 1000 steps
        # Update humans
        env.update_humans(dt)

        # Update robot
        robot_position, goal_check, force_vector, force_components = robot.update(env.get_human_positions(), env.human_velocities, dt)

        # Update visualization
        env.update_visuals(robot_position, force_vector, force_components, time_elapsed)
        print ('robot_position:', robot_position)

        # Increment time
        time_elapsed += dt

        # Pause for real-time simulation
        time.sleep(dt)
        
        # Check if goal is reached
        if goal_check:
            break

if __name__ == "__main__":
    main()