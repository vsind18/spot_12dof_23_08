import solo12_kinematic
import numpy as np

leg_name = {'fl': 1, 'fr': 1, 'hl': 0, 'hr': 0}
kine = solo12_kinematic.Solo12Kinematic()


def compute_motor_angles(kinematic, leg_name_to_sol_branch_Solo12, x, y, z, leg_name):
    """
    Compute angles from x,y,z
    :param x: x coordinate
    :param y: y coordinate
    :param z: z coordinate
    :param leg_name: leg name
    :return: a list contain motor angles
    """
    return list(kinematic.inverse_kinematics(x, y, z, leg_name_to_sol_branch_Solo12[leg_name]))

def gen_signal(t, phase, frequency, step_length, step_height, x_init, y_init):
    """Generates a modified sinusoidal reference leg trajectory with half-circle shape.

    Args:
      t: Current time in simulation.
      phase: The phase offset for the periodic trajectory.
      frequency: The frequency of the gait.
      step_length: The length of the step.
      step_height: The height of the step.
      x_init: The initial x position.
      y_init: The initial y position.

    Returns:
      The desired leg x and y angle at the current time.
    """
    period = 1 / frequency
    theta = (2 * np.pi / period * t + phase) % (2 * np.pi)

    x = -step_length * np.cos(theta) + x_init
    if theta > np.pi:
        y = y_init
    else:
        y = step_height * np.sin(theta) + y_init
    return x, y

def signal(t, kinematic, leg_name_to_sol_branch_Solo12, frequency, step_length, step_height, x_init, y_init):
    """Generates the trotting gait for the robot.

    Args:
      t: Current time in simulation.
      kinematic: The kinematic model for inverse kinematics.
      leg_name_to_sol_branch_Solo12: Mapping from leg name to solution branch.
      frequency: The frequency of the gait.
      step_length: The length of the step.
      step_height: The height of the step.
      x_init: The initial x position.
      y_init: The initial y position.

    Returns:
      A numpy array of the reference leg positions.
    """
    # Generates the leg trajectories for the two diagonal pairs of legs.
    ext_first_pair, sw_first_pair = gen_signal(t, phase=0, frequency=frequency, step_length=step_length, step_height=step_height, x_init=x_init, y_init=y_init)
    ext_second_pair, sw_second_pair = gen_signal(t, phase=np.pi, frequency=frequency, step_length=step_length, step_height=step_height, x_init=x_init, y_init=y_init)

    motors_fl = compute_motor_angles(kinematic, leg_name_to_sol_branch_Solo12, ext_first_pair, sw_first_pair, 0, "fl")
    motors_hr = compute_motor_angles(kinematic, leg_name_to_sol_branch_Solo12, ext_first_pair, sw_first_pair, 0, "hr")
    motors_fr = compute_motor_angles(kinematic, leg_name_to_sol_branch_Solo12, ext_second_pair, sw_second_pair, 0, "fr")
    motors_hl = compute_motor_angles(kinematic, leg_name_to_sol_branch_Solo12, ext_second_pair, sw_second_pair, 0, "hl")

    motors_fl[0] += np.pi / 2
    motors_hr[0] += np.pi / 2
    motors_fr[0] += np.pi / 2
    motors_hl[0] += np.pi / 2

    trotting_signal = np.array([*motors_fl, *motors_hr, *motors_fr, *motors_hl])
    return trotting_signal

time = 0

for _ in range(200):
    print(signal(t=time, kinematic=kine,leg_name_to_sol_branch_Solo12=leg_name, frequency=2.5, step_length=0.08, step_height=0.06, x_init=0, y_init=-0.23))
    print("-------------------------------")
    time += 0.02
