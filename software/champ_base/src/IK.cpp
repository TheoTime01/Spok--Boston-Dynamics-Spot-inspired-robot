#include <IK_Controller.h>
 // Include the PoseArray message

champ::PhaseGenerator::Time rosTimeToChampTime(const rclcpp::Time& time)
{
    return time.nanoseconds() / 1000ul;
}

IK_Controller::IK_Controller():
    Node("inverse_kinematics_node",rclcpp::NodeOptions()
                        .allow_undeclared_parameters(true)
                        .automatically_declare_parameters_from_overrides(true)),
    clock_(*this->get_clock()),
    body_controller_(base_),
    leg_controller_(base_, rosTimeToChampTime(clock_.now())),
    kinematics_(base_)
{
    std::string joint_control_topic = "joint_group_position_controller/command";
    std::string knee_orientation;
    std::string urdf = "";

    double loop_rate = 200.0;

    this->get_parameter("gait.pantograph_leg",         gait_config_.pantograph_leg);
    this->get_parameter("gait.max_linear_velocity_x",  gait_config_.max_linear_velocity_x);
    this->get_parameter("gait.max_linear_velocity_y",  gait_config_.max_linear_velocity_y);
    this->get_parameter("gait.max_angular_velocity_z", gait_config_.max_angular_velocity_z);
    this->get_parameter("gait.com_x_translation",      gait_config_.com_x_translation);
    this->get_parameter("gait.swing_height",           gait_config_.swing_height);
    this->get_parameter("gait.stance_depth",           gait_config_.stance_depth);
    this->get_parameter("gait.stance_duration",        gait_config_.stance_duration);
    this->get_parameter("gait.nominal_height",         gait_config_.nominal_height);
    this->get_parameter("gait.knee_orientation",       knee_orientation);
    this->get_parameter("publish_foot_contacts",       publish_foot_contacts_);
    this->get_parameter("publish_joint_states",        publish_joint_states_);
    this->get_parameter("publish_joint_control",       publish_joint_control_);
    this->get_parameter("gazebo",                      in_gazebo_);
    this->get_parameter("joint_controller_topic",      joint_control_topic);
    this->get_parameter("loop_rate",                   loop_rate);
    this->get_parameter("urdf",                        urdf);

    cmd_foot_positions_subscription_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
        "target_foot_positions", 10, std::bind(&IK_Controller::cmdFootPositionsCallback_, this, std::placeholders::_1));

    if(publish_joint_control_)
    {
        joint_commands_publisher_ = this->create_publisher<trajectory_msgs::msg::JointTrajectory>(joint_control_topic, 10);
    }

    if(publish_joint_states_ && !in_gazebo_)
    {
        joint_states_publisher_ = this->create_publisher<sensor_msgs::msg::JointState>("joint_states", 10);
    }

    if(publish_foot_contacts_ && !in_gazebo_)
    {
        foot_contacts_publisher_   = this->create_publisher<champ_msgs::msg::ContactsStamped>("foot_contacts", 10);
    }

    gait_config_.knee_orientation = knee_orientation.c_str();
    
    base_.setGaitConfig(gait_config_);
    champ::URDF::loadFromString(base_, this->get_node_parameters_interface(), urdf);
    joint_names_ = champ::URDF::getJointNames(this->get_node_parameters_interface());
    std::chrono::milliseconds period(static_cast<int>(1000 / loop_rate));

    loop_timer_ = this->create_wall_timer(
         std::chrono::duration_cast<std::chrono::milliseconds>(period), std::bind(&IK_Controller::controlLoop_, this));
    req_pose_.position.z = gait_config_.nominal_height;
}

void IK_Controller::cmdFootPositionsCallback_(const geometry_msgs::msg::PoseArray::SharedPtr msg)
{
    // Assuming the message contains foot positions for all 4 feet
    for (size_t i = 0; i < 4 && i < msg->poses.size(); ++i)
    {
        target_foot_positions[i].p.X() = msg->poses[i].position.x;
        target_foot_positions[i].p.Y() = msg->poses[i].position.y;
        target_foot_positions[i].p.Z() = msg->poses[i].position.z;

        // If you need to apply orientation (rotation) as well
        tf2::Quaternion quat(
            msg->poses[i].orientation.x,
            msg->poses[i].orientation.y,
            msg->poses[i].orientation.z,
            msg->poses[i].orientation.w
        );
        tf2::Matrix3x3 m(quat);
        double roll, pitch, yaw;
        m.getRPY(roll, pitch, yaw);

        target_foot_positions[i].RotateX(roll);
        target_foot_positions[i].RotateY(pitch);
        target_foot_positions[i].RotateZ(yaw);
    }
}

void IK_Controller::controlLoop_()
{
    float target_joint_positions[12];
    
    bool foot_contacts[4];
    if(target_foot_positions[0].X() == 0.0 && target_foot_positions[0].Y() == 0.0 && target_foot_positions[0].Z() == 0.0 &&
       target_foot_positions[1].X() == 0.0 && target_foot_positions[1].Y() == 0.0 && target_foot_positions[1].Z() == 0.0 &&
       target_foot_positions[2].X() == 0.0 && target_foot_positions[2].Y() == 0.0 && target_foot_positions[2].Z() == 0.0 &&
       target_foot_positions[3].X() == 0.0 && target_foot_positions[3].Y() == 0.0 && target_foot_positions[3].Z() == 0.0
       ) {
        return;
    }

    if(target_foot_positions[0].X() == target_foot_positions_old[0].X() && 
       target_foot_positions[0].Y() == target_foot_positions_old[0].Y() && 
       target_foot_positions[0].Z() == target_foot_positions_old[0].Z() &&
       target_foot_positions[1].X() == target_foot_positions_old[1].X() && 
       target_foot_positions[1].Y() == target_foot_positions_old[1].Y() && 
       target_foot_positions[1].Z() == target_foot_positions_old[1].Z() &&
       target_foot_positions[2].X() == target_foot_positions_old[2].X() && 
       target_foot_positions[2].Y() == target_foot_positions_old[2].Y() && 
       target_foot_positions[2].Z() == target_foot_positions_old[2].Z() &&
       target_foot_positions[3].X() == target_foot_positions_old[3].X() && 
       target_foot_positions[3].Y() == target_foot_positions_old[3].Y() && 
       target_foot_positions[3].Z() == target_foot_positions_old[3].Z()
       ) {
        return;
    }
    


    kinematics_.inverse(target_joint_positions, target_foot_positions);

    // Log the foot positions
    RCLCPP_INFO(this->get_logger(), "target_foot_positions - P1: (%f, %f, %f), P2: (%f, %f, %f), P3: (%f, %f, %f), P4: (%f, %f, %f)",
                target_foot_positions[0].X(), target_foot_positions[0].Y(), target_foot_positions[0].Z(),
                target_foot_positions[1].X(), target_foot_positions[1].Y(), target_foot_positions[1].Z(),
                target_foot_positions[2].X(), target_foot_positions[2].Y(), target_foot_positions[2].Z(),
                target_foot_positions[3].X(), target_foot_positions[3].Y(), target_foot_positions[3].Z());

    publishFootContacts_(foot_contacts);
    publishJoints_(target_joint_positions);

    target_foot_positions_old[0].X() = target_foot_positions[0].X();
    target_foot_positions_old[0].Y() = target_foot_positions[0].Y();
    target_foot_positions_old[0].Z() = target_foot_positions[0].Z();
    target_foot_positions_old[1].X() = target_foot_positions[1].X();
    target_foot_positions_old[1].Y() = target_foot_positions[1].Y(); 
    target_foot_positions_old[1].Z() = target_foot_positions[1].Z();
    target_foot_positions_old[2].X() = target_foot_positions[2].X();
    target_foot_positions_old[2].Y() = target_foot_positions[2].Y();
    target_foot_positions_old[2].Z() = target_foot_positions[2].Z();
    target_foot_positions_old[3].X() = target_foot_positions[3].X();
    target_foot_positions_old[3].Y() = target_foot_positions[3].Y(); 
    target_foot_positions_old[3].Z() = target_foot_positions[3].Z();

}

void IK_Controller::publishJoints_(float target_joints[12])
{
    if(publish_joint_control_)
    {
        trajectory_msgs::msg::JointTrajectory joints_cmd_msg;
        joints_cmd_msg.header.stamp = clock_.now();

        joints_cmd_msg.joint_names = joint_names_;

        trajectory_msgs::msg::JointTrajectoryPoint point;
        point.positions.resize(12);

        for(size_t i = 0; i < 12; i++)
        {
            point.positions[i] = target_joints[i];
        }

        joints_cmd_msg.points.push_back(point);
        joint_commands_publisher_->publish(joints_cmd_msg);
    }

    if(publish_joint_states_ && !in_gazebo_)
    {
        sensor_msgs::msg::JointState joints_msg;

        joints_msg.header.stamp = clock_.now();
        joints_msg.name.resize(joint_names_.size());
        joints_msg.position.resize(joint_names_.size());
        joints_msg.name = joint_names_;

        for (size_t i = 0; i < joint_names_.size(); ++i)
        {
            joints_msg.position[i] = target_joints[i];
        }

        joint_states_publisher_->publish(joints_msg);
    }
}

void IK_Controller::publishFootContacts_(bool foot_contacts[4])
{
    if(publish_foot_contacts_ && !in_gazebo_)
    {
        champ_msgs::msg::ContactsStamped contacts_msg;
        contacts_msg.header.stamp = clock_.now();
        contacts_msg.contacts.resize(4);

        for(size_t i = 0; i < 4; i++)
        {
            contacts_msg.contacts[i] = base_.legs[i]->gait_phase();
        }
        foot_contacts_publisher_->publish(contacts_msg);
    }
}
