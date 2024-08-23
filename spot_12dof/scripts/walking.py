#!/usr/bin/env python3
import rospy
from std_msgs.msg import Float64
import solo12_kinematic
import numpy as np

# Khởi tạo ROS node
rospy.init_node('robot_controller')

# Khai báo các Publisher
pub_fl_hip = rospy.Publisher('/spot_12dof/fl_hip_joint_position_controller/command', Float64, queue_size=10)
pub_fl_knee = rospy.Publisher('/spot_12dof/fl_knee_joint_position_controller/command', Float64, queue_size=10)
pub_fl_abduction = rospy.Publisher('/spot_12dof/fl_abduction_joint_position_controller/command', Float64, queue_size=10)

pub_fr_hip = rospy.Publisher('/spot_12dof/fr_hip_joint_position_controller/command', Float64, queue_size=10)
pub_fr_knee = rospy.Publisher('/spot_12dof/fr_knee_joint_position_controller/command', Float64, queue_size=10)
pub_fr_abduction = rospy.Publisher('/spot_12dof/fr_abduction_joint_position_controller/command', Float64, queue_size=10)

pub_hl_hip = rospy.Publisher('/spot_12dof/hl_hip_joint_position_controller/command', Float64, queue_size=10)
pub_hl_knee = rospy.Publisher('/spot_12dof/hl_knee_joint_position_controller/command', Float64, queue_size=10)
pub_hl_abduction = rospy.Publisher('/spot_12dof/hl_abduction_joint_position_controller/command', Float64, queue_size=10)

pub_hr_hip = rospy.Publisher('/spot_12dof/hr_hip_joint_position_controller/command', Float64, queue_size=10)
pub_hr_knee = rospy.Publisher('/spot_12dof/hr_knee_joint_position_controller/command', Float64, queue_size=10)
pub_hr_abduction = rospy.Publisher('/spot_12dof/hr_abduction_joint_position_controller/command', Float64, queue_size=10)

def send_commands(motor_angles):
    """
    Gửi lệnh điều khiển tới các joint của robot.
    :param motor_angles: Danh sách các góc động cơ cho các joint.
    """
    # Phân tách các góc động cơ cho từng joint
    fr_hip, fr_knee, fr_abduction, hr_hip, hr_knee, hr_abduction, fl_hip, fl_knee, fl_abduction, hl_hip, hl_knee, hl_abduction = motor_angles

    # Gửi lệnh tới từng joint
    pub_fl_hip.publish(Float64(fl_hip))
    pub_fl_knee.publish(Float64(fl_knee))
    pub_fl_abduction.publish(Float64(fl_abduction))

    pub_fr_hip.publish(Float64(fr_hip))
    pub_fr_knee.publish(Float64(fr_knee))
    pub_fr_abduction.publish(Float64(fr_abduction))

    pub_hl_hip.publish(Float64(hl_hip))
    pub_hl_knee.publish(Float64(hl_knee))
    pub_hl_abduction.publish(Float64(hl_abduction))

    pub_hr_hip.publish(Float64(hr_hip))
    pub_hr_knee.publish(Float64(hr_knee))
    pub_hr_abduction.publish(Float64(hr_abduction))

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

    motors_fl[0] += - 0.75 * np.pi 
    motors_fr[0] += - 0.75 * np.pi 
    motors_fl[1] += + 1.25 * np.pi 
    motors_fr[1] += + 1.25 * np.pi

    motors_hl[0] += 0.75 * np.pi 
    motors_hr[0] += 0.75 * np.pi 
    motors_hl[1] += - 0.25 * np.pi 
    motors_hr[1] += - 0.25 * np.pi
    
    # motors_hl[0] += - np.pi / 4
    # motors_hr[0] += - np.pi / 4
    # motors_fl[0] += + np.pi / 4
    # motors_fr[0] += + np.pi / 4
    # motors_hl[1] += np.pi / 4
    # motors_hr[1] += np.pi / 4
    # motors_fl[1] += 3 * np.pi / 4
    # motors_fr[1] += 3 * np.pi / 4
    # 0011

    trotting_signal = np.array([*motors_fl, *motors_hr, *motors_fr, *motors_hl])
    return trotting_signal
def main():
    rate = rospy.Rate(100)  # Tần suất cập nhật là 50 Hz
    time = rospy.get_time()
    kine = solo12_kinematic.Solo12Kinematic()  # Khởi tạo đối tượng kinematic
    leg_name_to_sol_branch_Solo12 = {'fl': 1, 'fr': 1, 'hl': 0, 'hr': 0}
    
    while not rospy.is_shutdown():
        # Tạo tín hiệu cho các joint
        motor_angles = signal(t=time, kinematic=kine, leg_name_to_sol_branch_Solo12=leg_name_to_sol_branch_Solo12, frequency=2.5, step_length=0.08, step_height=0.06, x_init=0, y_init=-0.23)
        
        # Gửi lệnh điều khiển tới robot
        send_commands(motor_angles)
        
        # Cập nhật thời gian và tần suất vòng lặp
        time = rospy.get_time() 
        rate.sleep()

if __name__ == '__main__':
    main()
