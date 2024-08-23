import rospy
from std_msgs.msg import Float64

# Tạo các publisher cho mỗi khớp của robot
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

def stand_up():
    rospy.init_node('stand_up_robot')

    rate = rospy.Rate(10)  # Tần số gửi lệnh, ở đây là 10Hz

    # Vị trí ban đầu
    motor_abdution_fl = 0.0
    motor_hip_fl = 0.0
    motor_knee_fl = 0.0
    motor_abdution_fr = 0.0
    motor_hip_fr = 0.0
    motor_knee_fr = 0.0
    motor_abdution_hl = 0.0
    motor_hip_hl = 0.0
    motor_knee_hl = 0.0
    motor_abdution_hr = 0.0
    motor_hip_hr = 0.0
    motor_knee_hr = 0.0
    # Mục tiêu cuối cùng
    target_position = 0.5
    step = 0.01  # Bước tăng dần mỗi lần

    while not rospy.is_shutdown():
        # Tăng dần vị trí khớp cho đến khi đạt vị trí mục tiêu
        if motor_abdution_fl < target_position:
            motor_abdution_fl += step
            motor_hip_fl += step
            motor_knee_fl += step
            motor_abdution_fr += step
            motor_hip_fr += step
            motor_knee_fr += step
            motor_abdution_hl += step
            motor_hip_hl += step
            motor_knee_hl += step
            motor_abdution_hr += step
            motor_hip_hr += step
            motor_knee_hr += step
            # Giới hạn giá trị không vượt quá mục tiêu
            joint1_position = min(joint1_position, target_position)
            joint2_position = min(joint2_position, target_position)
            joint3_position = min(joint3_position, target_position)
            joint4_position = min(joint4_position, target_position)

            # Gửi lệnh điều khiển đến các khớp
            pub_joint1.publish(joint1_position)
            pub_joint2.publish(joint2_position)
            pub_joint3.publish(joint3_position)
            pub_joint4.publish(joint4_position)

        rate.sleep()

if __name__ == '__main__':
    try:
        stand_up()
    except rospy.ROSInterruptException:
        pass