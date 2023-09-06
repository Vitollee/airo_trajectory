#include "airo_trajectory/airo_trajectory_server.h"

AIRO_TRAJECTORY_SERVER::AIRO_TRAJECTORY_SERVER(ros::NodeHandle& nh){
    nh.getParam("airo_control_node/fsm/pose_topic",POSE_TOPIC);
    nh.getParam("airo_control_node/fsm/twist_topic",TWIST_TOPIC);
    nh.getParam("airo_control_node/fsm/controller_type",CONTROLLER_TYPE);
    local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(POSE_TOPIC,5,&AIRO_TRAJECTORY_SERVER::pose_cb,this);
    local_twist_sub = nh.subscribe<geometry_msgs::TwistStamped>(TWIST_TOPIC,5,&AIRO_TRAJECTORY_SERVER::twist_cb,this);
    fsm_info_sub = nh.subscribe<airo_message::FSMInfo>("/airo_control/fsm_info",1,&AIRO_TRAJECTORY_SERVER::fsm_info_cb,this);
    command_pub = nh.advertise<airo_message::Reference>("/airo_control/setpoint",1);
    command_preview_pub = nh.advertise<airo_message::ReferencePreview>("/airo_control/setpoint_preview",1);
    takeoff_land_pub = nh.advertise<airo_message::TakeoffLandTrigger>("/airo_control/takeoff_land_trigger",1);
    if (CONTROLLER_TYPE == "mpc"){
        nh.getParam("airo_control_node/mpc/enable_preview",mpc_enable_preview);
        if (mpc_enable_preview){
            use_preview = true;
        }
    }
    std::cout<<"use_preview"<<use_preview<<std::endl;
}

void AIRO_TRAJECTORY_SERVER::pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pose.header = msg->header;
    local_pose.pose = msg->pose;
}

void AIRO_TRAJECTORY_SERVER::twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    current_twist = sqrt(pow(msg->twist.linear.x,2) + pow(msg->twist.linear.y, 2) + pow(msg->twist.linear.z,2));
}

void AIRO_TRAJECTORY_SERVER::fsm_info_cb(const airo_message::FSMInfo::ConstPtr& msg){
    fsm_info.header = msg->header;
    fsm_info.is_landed = msg->is_landed;
    fsm_info.is_waiting_for_command = msg->is_waiting_for_command;
}

void AIRO_TRAJECTORY_SERVER::pose_cmd(const geometry_msgs::Point& point, const double& yaw_angle){
    if(fsm_info.is_waiting_for_command){
        geometry_msgs::Twist twist;
        twist.linear.x = 0.0;
        twist.linear.y = 0.0;
        twist.linear.z = 0.0;
        pose_cmd(point,twist,yaw_angle);
    }
}

void AIRO_TRAJECTORY_SERVER::pose_cmd(const geometry_msgs::Point& point, const geometry_msgs::Twist& twist, const double& yaw_angle){
    if(fsm_info.is_waiting_for_command){
        geometry_msgs::Accel accel;
        accel.linear.x = 0.0;
        accel.linear.y = 0.0;
        accel.linear.z = 0.0;
        pose_cmd(point,twist,accel,yaw_angle);
    }
}

void AIRO_TRAJECTORY_SERVER::pose_cmd(const geometry_msgs::Point& point, const geometry_msgs::Twist& twist, const geometry_msgs::Accel& accel, const double& yaw_angle){
    if(fsm_info.is_waiting_for_command){
        airo_message::Reference reference;
        reference.header.stamp = ros::Time::now();
        reference.ref_pose.position = point;
        reference.ref_pose.orientation = AIRO_TRAJECTORY_SERVER::yaw_to_quaternion(yaw_angle);
        reference.ref_twist = twist;
        reference.ref_accel = accel;
        
        command_pub.publish(reference);
    }
}

bool AIRO_TRAJECTORY_SERVER::target_reached(const geometry_msgs::Point& msg){
    bool position_reached = sqrt(pow(msg.x - local_pose.pose.position.x,2)+pow(msg.y - local_pose.pose.position.y,2)
    +pow(msg.z - local_pose.pose.position.z,2)) < 0.5;
    bool twist_reached = current_twist < 0.5;
    return position_reached && twist_reached;
}

geometry_msgs::Quaternion AIRO_TRAJECTORY_SERVER::yaw_to_quaternion(double yaw){
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, yaw);
    return quaternion;
}

double AIRO_TRAJECTORY_SERVER::quaternion_to_yaw(const geometry_msgs::Quaternion& quaternion){
    double phi,theta,psi;
    tf::Quaternion tf_quaternion;
    tf::quaternionMsgToTF(quaternion,tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(phi, theta, psi);
    return psi;
}

void AIRO_TRAJECTORY_SERVER::file_traj_init(const std::string& file_name, std::vector<std::vector<double>>& traj){
    std::ifstream file(file_name);
    std::string line;
    int number_of_lines = 0;
    traj.clear();

    if(file.is_open()){
        while(getline(file, line)){
            number_of_lines++;
            std::istringstream linestream( line );
            std::vector<double> linedata;
            double number;
            while( linestream >> number ){
                linedata.push_back( number );
            }
            traj.push_back( linedata );
        }
        file.close();
    }
    else{
        ROS_ERROR("[AIRo Trajectory] Cannot open trajectory file!");
    }
}

geometry_msgs::Point AIRO_TRAJECTORY_SERVER::get_start_point(const std::vector<std::vector<double>>& traj){
    geometry_msgs::Point start_point;
    start_point.x = traj[0][0];
    start_point.y = traj[0][1];
    start_point.z = traj[0][2];
    return start_point;
}

geometry_msgs::Point AIRO_TRAJECTORY_SERVER::get_end_point(const std::vector<std::vector<double>>& traj){
    geometry_msgs::Point end_point;
    end_point.x = traj[traj.size()-1][0];
    end_point.y = traj[traj.size()-1][1];
    end_point.z = traj[traj.size()-1][2];
    return end_point;
}

bool AIRO_TRAJECTORY_SERVER::file_cmd(const std::vector<std::vector<double>>& traj, int& start_row){
    const int total_rows = traj.size();
    int path_ended = false;

    if (use_preview){
        std::vector<std::vector<double>> traj_matrix;
        if (start_row >= total_rows - 1) {
            // Construct all rows using the last row of traj
            std::vector<double> last_row = traj.back();
            traj_matrix.assign(preview_size, last_row);
            path_ended = true;
        }
        else{
            // Calculate the end row index
            int end_row = start_row + preview_size - 1;
            end_row = std::min(end_row, total_rows - 1);  // Make sure end_row doesn't exceed the maximum row index

            // Copy the desired rows into the trajectory matrix
            traj_matrix.assign(traj.begin() + start_row, traj.begin() + end_row + 1);

            // If there are fewer than num_rows available, fill the remaining rows with the last row
            while (traj_matrix.size() < preview_size){
                traj_matrix.push_back(traj.back());
            }

            // Update the start_row for the next call
            start_row++;
        }

        airo_message::ReferencePreview reference_preview;
        reference_preview.header.stamp = ros::Time::now();
        reference_preview.ref_pose.resize(preview_size);
        reference_preview.ref_twist.resize(preview_size);
        reference_preview.ref_accel.resize(preview_size);
        int column = traj_matrix[0].size();

        if (!path_ended){
            if (column == 3){
                assign_position(traj_matrix,reference_preview);
            }
            else if (column == 4){
                assign_position(traj_matrix,reference_preview);
                assign_yaw(traj_matrix,reference_preview);
            }
            else if (column == 6){
                assign_position(traj_matrix,reference_preview);
                assign_twist(traj_matrix,reference_preview);
            }
            else if (column == 7){
                assign_position(traj_matrix,reference_preview);
                assign_twist(traj_matrix,reference_preview);
                assign_yaw(traj_matrix,reference_preview);
            }
            else if (column == 9){
                assign_position(traj_matrix,reference_preview);
                assign_twist(traj_matrix,reference_preview);
                assign_accel(traj_matrix,reference_preview);
            }
            else if (column == 10){
                assign_position(traj_matrix,reference_preview);
                assign_twist(traj_matrix,reference_preview);
                assign_accel(traj_matrix,reference_preview);
                assign_yaw(traj_matrix,reference_preview);
            }
            else{
                ROS_ERROR("[AIRo Trajectory] Trajectory file dimension wrong!");
                return false;
            }
        }
        else{
            assign_position(traj_matrix,reference_preview);
            if (column == 4 || column == 6 || column == 7 || column == 9 || column == 10){
                assign_yaw(traj_matrix,reference_preview);
            }
        }

        command_preview_pub.publish(reference_preview);
    }
    else{
        std::vector<double> traj_row;
        if (start_row >= total_rows - 1) {
            traj_row = traj.back();
            path_ended = true;
        }
        else{
            traj_row = traj[start_row];
            // Update the start_row for the next call
            start_row++;
        }
        airo_message::Reference reference;
        reference.header.stamp = ros::Time::now();
        int column = traj_row.size();

        if (!path_ended){
            if (column == 3){
                assign_position(traj_row,reference);
            }
            else if (column == 4){
                assign_position(traj_row,reference);
                assign_yaw(traj_row,reference);
            }
            else if (column == 6){
                assign_position(traj_row,reference);
                assign_twist(traj_row,reference);
            }
            else if (column == 7){
                assign_position(traj_row,reference);
                assign_twist(traj_row,reference);
                assign_yaw(traj_row,reference);
            }
            else if (column == 9){
                assign_position(traj_row,reference);
                assign_twist(traj_row,reference);
                assign_accel(traj_row,reference);
            }
            else if (column == 10){
                assign_position(traj_row,reference);
                assign_twist(traj_row,reference);
                assign_accel(traj_row,reference);
                assign_yaw(traj_row,reference);
            }
            else{
                ROS_ERROR("[AIRo Trajectory] Trajectory file dimension wrong!");
                return false;
            }
        }
        else{
            assign_position(traj_row,reference);
            if (column == 4 || column == 6 || column == 7 || column == 9 || column == 10){
                assign_yaw(traj_row,reference);
            }
        }

        command_pub.publish(reference);
    }


    return path_ended;
}

void AIRO_TRAJECTORY_SERVER::assign_position(const std::vector<std::vector<double>>& traj_matrix, airo_message::ReferencePreview& reference_preview){
    for (int i = 0; i < preview_size; i++){
        reference_preview.ref_pose[i].position.x = traj_matrix[i][0];
        reference_preview.ref_pose[i].position.y = traj_matrix[i][1];
        reference_preview.ref_pose[i].position.z = traj_matrix[i][2];
        reference_preview.ref_pose[i].orientation = AIRO_TRAJECTORY_SERVER::yaw_to_quaternion(0.0);
        reference_preview.ref_twist[i].linear.x = 0.0;
        reference_preview.ref_twist[i].linear.y = 0.0;
        reference_preview.ref_twist[i].linear.z = 0.0;
        reference_preview.ref_accel[i].linear.x = 0.0;
        reference_preview.ref_accel[i].linear.y = 0.0;
        reference_preview.ref_accel[i].linear.z = 0.0;
    }
}

void AIRO_TRAJECTORY_SERVER::assign_position(const std::vector<double>& traj_row, airo_message::Reference& reference){
    reference.ref_pose.position.x = traj_row[0];
    reference.ref_pose.position.y = traj_row[1];
    reference.ref_pose.position.z = traj_row[2];
    reference.ref_pose.orientation = AIRO_TRAJECTORY_SERVER::yaw_to_quaternion(0.0);
    reference.ref_twist.linear.x = 0.0;
    reference.ref_twist.linear.y = 0.0;
    reference.ref_twist.linear.z = 0.0;
    reference.ref_accel.linear.x = 0.0;
    reference.ref_accel.linear.y = 0.0;
    reference.ref_accel.linear.z = 0.0;
}

void AIRO_TRAJECTORY_SERVER::assign_twist(const std::vector<std::vector<double>>& traj_matrix, airo_message::ReferencePreview& reference_preview){
    for (int i = 0; i < preview_size; i++){
        reference_preview.ref_twist[i].linear.x = traj_matrix[i][3];
        reference_preview.ref_twist[i].linear.y = traj_matrix[i][4];
        reference_preview.ref_twist[i].linear.z = traj_matrix[i][5];
    }
}

void AIRO_TRAJECTORY_SERVER::assign_twist(const std::vector<double>& traj_row, airo_message::Reference& reference){
    reference.ref_twist.linear.x = traj_row[3];
    reference.ref_twist.linear.y = traj_row[4];
    reference.ref_twist.linear.z = traj_row[5];
}

void AIRO_TRAJECTORY_SERVER::assign_accel(const std::vector<std::vector<double>>& traj_matrix, airo_message::ReferencePreview& reference_preview){
    for (int i = 0; i < preview_size; i++){
        reference_preview.ref_accel[i].linear.x = traj_matrix[i][6];
        reference_preview.ref_accel[i].linear.y = traj_matrix[i][7];
        reference_preview.ref_accel[i].linear.z = traj_matrix[i][8];
    }
}

void AIRO_TRAJECTORY_SERVER::assign_accel(const std::vector<double>& traj_row, airo_message::Reference& reference){
    reference.ref_accel.linear.x = traj_row[6];
    reference.ref_accel.linear.y = traj_row[7];
    reference.ref_accel.linear.z = traj_row[8];
}

void AIRO_TRAJECTORY_SERVER::assign_yaw(const std::vector<std::vector<double>>& traj_matrix, airo_message::ReferencePreview& reference_preview){
    for (int i = 0; i < preview_size; i++){
        reference_preview.ref_pose[i].orientation = AIRO_TRAJECTORY_SERVER::yaw_to_quaternion(traj_matrix[i].back());
    }
}

void AIRO_TRAJECTORY_SERVER::assign_yaw(const std::vector<double>& traj_row, airo_message::Reference& reference){
    reference.ref_pose.orientation = AIRO_TRAJECTORY_SERVER::yaw_to_quaternion(traj_row.back());
}

bool AIRO_TRAJECTORY_SERVER::takeoff(){
    if(fsm_info.is_landed == true){
        airo_message::TakeoffLandTrigger takeoff_trigger;
        ROS_INFO_THROTTLE(2.0, "[AIRo Trajectory] Sending takeoff trigger.");
        takeoff_trigger.takeoff_land_trigger = true; // Takeoff
        takeoff_trigger.header.stamp = ros::Time::now();
        takeoff_land_pub.publish(takeoff_trigger);
        return false;
    }
    else if(fsm_info.is_landed == false && fsm_info.is_waiting_for_command == true){
        ROS_INFO("[AIRo Trajectory] Vehicle already takeoff!");
        return true;
    }
    else{
        return false;
    }
}

bool AIRO_TRAJECTORY_SERVER::land(){
    if(fsm_info.is_landed == false && fsm_info.is_waiting_for_command == true){
        airo_message::TakeoffLandTrigger land_trigger;
        ROS_INFO_THROTTLE(2.0, "[AIRo Trajectory] Sending land trigger.");
        land_trigger.takeoff_land_trigger = false; // Land
        land_trigger.header.stamp = ros::Time::now();
        takeoff_land_pub.publish(land_trigger);
        return false;
    }
    else if(fsm_info.is_landed == true){
        ROS_INFO("[AIRo Trajectory] Vehicle has landed!");
        return true;
    }
    else{
        return false;
    }
}