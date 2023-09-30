#include <ros/ros.h>
#include <ros/package.h>
#include <tf/tf.h>

#include <fstream>
#include <eigen3/Eigen/Dense>

#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/CommandTOL.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/AttitudeTarget.h>
 
enum State{
    TAKEOFF,
    INIT,
    FILE_TRAJ,
    LAND
};

int FREQUENCY = 100;
double takeoff_height = 1.0, twist_norm = 0.0;
int log_counter,log_interval = 5,row_to_read;
mavros_msgs::State current_state;
mavros_msgs::AttitudeTarget target_attitude;
geometry_msgs::PoseStamped local_pose,takeoff_pose,init_pose;
geometry_msgs::TwistStamped local_twist;
std::string POSE_TOPIC,TRAJ_NAME,TRAJ_SCRIPT_PATH,TRAJ_FILE_PATH;
std::vector<std::vector<double>> log_data; // t,ref_x,x,ref_y,y,ref_z,z,ref_u,u,ref_v,v,ref_w,w,ref_phi,phi,ref_theta,theta,ref_psi,psi,thrust
bool local_pose_received = false, RESULT_SAVE = false;
ros::Publisher command_pub;

void state_cb(const mavros_msgs::State::ConstPtr& msg){
    current_state = *msg;
}

void pose_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
    local_pose.header = msg->header;
    local_pose.pose = msg->pose;
    local_pose_received = true;
}

void twist_cb(const geometry_msgs::TwistStamped::ConstPtr& msg){
    local_twist.twist.linear = msg->twist.linear;
    twist_norm = sqrt(pow(msg->twist.linear.x,2)+pow(msg->twist.linear.y,2)+pow(msg->twist.linear.z,2));
}

void target_attitude_cb(const mavros_msgs::AttitudeTarget::ConstPtr& msg){
    target_attitude.thrust = msg->thrust;
    target_attitude.orientation = msg->orientation;
    target_attitude.header = msg->header;
}

Eigen::Vector3d q2rpy(geometry_msgs::Quaternion q){
    Eigen::Vector3d euler;
    tf::Quaternion tf_quaternion;
    tf::quaternionMsgToTF(q,tf_quaternion);
    tf::Matrix3x3(tf_quaternion).getRPY(euler.x(), euler.y(), euler.z());
    return euler;
}

geometry_msgs::Quaternion rpy2q(Eigen::Vector3d euler){
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(euler.x(), euler.y(), euler.z());
    return quaternion;
}

geometry_msgs::Quaternion yaw2q(double yaw){
    geometry_msgs::Quaternion quaternion = tf::createQuaternionMsgFromRollPitchYaw(0.0, 0.0, yaw);
    return quaternion;
}


bool target_reached(const geometry_msgs::PoseStamped& msg){
    bool position_reached = sqrt(pow(msg.pose.position.x - local_pose.pose.position.x,2)+pow(msg.pose.position.y - local_pose.pose.position.y,2)
    +pow(msg.pose.position.z - local_pose.pose.position.z,2)) < 0.25;
    bool yaw_reached = abs(q2rpy(msg.pose.orientation).z() - q2rpy(local_pose.pose.orientation).z()) < 10.0 * M_PI / 180.0;
    return position_reached && yaw_reached;
}

void file_traj_init(const std::string& file_name, std::vector<std::vector<double>>& traj){
    std::ifstream file(file_name);
    std::string line;
    int number_of_lines = 0;
    traj.clear();
    if (RESULT_SAVE){
        log_data.clear();
        log_counter = log_interval;
    }

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

void update_log(const geometry_msgs::PoseStamped& ref){
    if (log_counter == log_interval){
        std::vector<double> line_to_push;
        Eigen::Vector3d local_euler,target_euler;
        local_euler = q2rpy(local_pose.pose.orientation);
        target_euler = q2rpy(target_attitude.orientation);

        line_to_push.push_back(static_cast<double>(log_data.size())*log_interval/FREQUENCY); // time
        line_to_push.push_back(ref.pose.position.x); // x position ref
        line_to_push.push_back(local_pose.pose.position.x); // x position
        line_to_push.push_back(ref.pose.position.y); // y position ref
        line_to_push.push_back(local_pose.pose.position.y); // y position
        line_to_push.push_back(ref.pose.position.z); // z position ref
        line_to_push.push_back(local_pose.pose.position.z); // z position
        line_to_push.push_back(0.0); // x velocity ref
        line_to_push.push_back(local_twist.twist.linear.x); // x velocity
        line_to_push.push_back(0.0); // y velocity ref
        line_to_push.push_back(local_twist.twist.linear.y); // y velocity
        line_to_push.push_back(0.0); // z velocity ref
        line_to_push.push_back(local_twist.twist.linear.z); // z velocity
        line_to_push.push_back(target_euler.x()); // phi ref
        line_to_push.push_back(local_euler.x()); // phi
        line_to_push.push_back(target_euler.y()); // theta ref
        line_to_push.push_back(local_euler.y()); // theta
        line_to_push.push_back(target_euler.z()); // psi ref
        line_to_push.push_back(local_euler.z()); // psi
        line_to_push.push_back(target_attitude.thrust); // thrust
        log_data.push_back(line_to_push);
        log_counter = 1;
    }
    else{
        log_counter++;
    }
}

bool file_cmd(const std::vector<std::vector<double>>& traj, int& start_row){
    const int total_rows = traj.size();
    int path_ended = false;
    geometry_msgs::PoseStamped ref_pose;

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

    double ref_yaw;
    if (traj_row.size() != 3){
        ref_yaw = traj_row[traj_row.size()-1];
    }
    else{
        ref_yaw = 0.0;
    }
    ref_pose.pose.position.x = traj_row[0];
    ref_pose.pose.position.y = traj_row[1];
    ref_pose.pose.position.z = traj_row[2];
    ref_pose.pose.orientation = yaw2q(ref_yaw);
    command_pub.publish(ref_pose);

    if (RESULT_SAVE){
        update_log(ref_pose);
    }

    if (!path_ended){
        ROS_INFO_STREAM_THROTTLE(2.0,"[AIRo Trajectory] Publishing file trajectory.");
    }
    else{
        ROS_INFO_STREAM_THROTTLE(2.0,"[AIRo Trajectory] File trajectory ended!");
    }
    
    return path_ended;
}

geometry_msgs::PoseStamped get_start_pose(const std::vector<std::vector<double>>& traj){
    geometry_msgs::PoseStamped start_point;
    double start_yaw;
    if (traj[0].size() != 3){
        start_yaw = traj[0][traj[0].size()-1];
    }
    else{
        start_yaw = 0.0;
    }
    start_point.header.stamp = ros::Time::now();
    start_point.pose.position.x = traj[0][0];
    start_point.pose.position.y = traj[0][1];
    start_point.pose.position.z = traj[0][2];
    start_point.pose.orientation = yaw2q(start_yaw);
    return start_point;
}

int save_result(){
    // Define column headers
    std::vector<std::string> column_headers = {"time","ref_x","x","ref_y","y","ref_z","z","ref_u","u","ref_v","v","ref_w","w","ref_phi","phi","ref_theta","theta","ref_psi","psi","thrust"};

    // Get the current time
    std::time_t now = std::time(nullptr);
    std::tm tm = *std::localtime(&now);

    // Format the time as "yyyy-mm-dd-hh-mm"
    std::ostringstream oss;
    oss << std::put_time(&tm, "%Y_%m_%d_%H_%M");
    std::string time_stamp = oss.str();

    // Get the path to the "airo_trajectory" package
    std::string package_path = ros::package::getPath("airo_trajectory");

    if (!package_path.empty()) {
        // Construct the full file path
        std::string log_path = package_path + "/results/" + time_stamp + ".csv";

        // Open the file for writing
        std::ofstream out_file(log_path);

        // Check if the file was successfully opened
        if (out_file.is_open()) {
            // Write column headers as the first row
            for (size_t i = 0; i < column_headers.size(); ++i) {
                out_file << column_headers[i];
                if (i < column_headers.size() - 1) {
                    out_file << ",";
                }
            }
            out_file << "\n";

            // Write the data to the CSV file
            for (const std::vector<double>& row : log_data) {
                for (size_t i = 0; i < row.size(); ++i) {
                    out_file << row[i];
                    if (i < row.size() - 1) {
                        out_file << ",";
                    }
                }
                out_file << "\n";
            }

            // Close the file
            out_file.close();

            ROS_INFO_STREAM("[AIRo Trajectory] Data saved to " << log_path);
        } else {
            ROS_ERROR_STREAM("[AIRo Trajectory] Failed to open the file for writing: " << log_path);
        }
    }

    return 0;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "position_setpoint_node");
    ros::NodeHandle nh;
    ros::Rate rate(FREQUENCY);
    State state = TAKEOFF;

    nh.getParam("/position_setpoint_node/pose_topic",POSE_TOPIC);
    nh.getParam("/position_setpoint_node/traj_name",TRAJ_NAME);
    nh.getParam("/position_setpoint_node/result_save",RESULT_SAVE);

    ros::Subscriber state_sub = nh.subscribe<mavros_msgs::State>("/mavros/state",10,state_cb);
    ros::Subscriber local_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>(POSE_TOPIC,100,pose_cb);
    ros::Subscriber local_twist_sub = nh.subscribe<geometry_msgs::TwistStamped>("/mavros/local_position/velocity_local",10,twist_cb);
    ros::Subscriber target_attitude_sub = nh.subscribe<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/target_attitude",10,target_attitude_cb);
    command_pub = nh.advertise<geometry_msgs::PoseStamped>("/mavros/setpoint_position/local",10);
    ros::ServiceClient arming_client = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    ros::ServiceClient landing_client = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros/cmd/land");
    ros::ServiceClient set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");

    // Read .txt file
    TRAJ_FILE_PATH = ros::package::getPath("airo_trajectory") + "/scripts/" + TRAJ_NAME + ".txt";
    std::vector<std::vector<double>> trajectory_data; // ref_x x ref_y y ref_z z ref_u u ref_v v ref_w w ref_phi phi ref_theta theta ref_psi psi thrust
    file_traj_init(TRAJ_FILE_PATH,trajectory_data);

    while(ros::ok() && (!current_state.connected || ! local_pose_received)){
        ros::spinOnce();
        rate.sleep();
        ROS_INFO_THROTTLE(1.0, "Waiting for connection.");
    }

    ROS_INFO("Connected!");

    takeoff_pose.pose.position.x = local_pose.pose.position.x;
    takeoff_pose.pose.position.y = local_pose.pose.position.y;
    takeoff_pose.pose.position.z = local_pose.pose.position.z + takeoff_height;
    takeoff_pose.pose.orientation.w = 1;
    takeoff_pose.pose.orientation.x = 0;
    takeoff_pose.pose.orientation.y = 0;
    takeoff_pose.pose.orientation.z = 0;

    for(int i = 100; ros::ok() && i > 0; --i){
        command_pub.publish(takeoff_pose);
        ros::spinOnce();
        rate.sleep();
    }

    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";

    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;

    mavros_msgs::CommandTOL land_cmd;
    land_cmd.request.yaw = 0;
    land_cmd.request.latitude = 0;
    land_cmd.request.longitude = 0;
    land_cmd.request.altitude = 0;

    ros::Time last_request = ros::Time::now();

    while(ros::ok()){
        switch(state){
            case TAKEOFF:{
                if( current_state.mode != "OFFBOARD" && (ros::Time::now() - last_request > ros::Duration(5.0))){
                    if( set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent){
                        ROS_WARN("Offboard enabled");
                    }
                    last_request = ros::Time::now();
                } else {
                    if( !current_state.armed && (ros::Time::now() - last_request > ros::Duration(5.0))){
                        if( arming_client.call(arm_cmd) && arm_cmd.response.success){
                            ROS_WARN("Vehicle armed");
                        }
                    last_request = ros::Time::now();
                    }
                }

                command_pub.publish(takeoff_pose);

                if(target_reached(takeoff_pose)){
                    state = INIT;
                    init_pose = get_start_pose(trajectory_data);
                }

                break;
            }

            case INIT:{
                command_pub.publish(init_pose);
                if(target_reached(init_pose) && twist_norm < 0.5){
                    state = FILE_TRAJ;
                    row_to_read = 0;
                }
                break;
            }

            case FILE_TRAJ:{
                if (file_cmd(trajectory_data,row_to_read) && twist_norm < 0.5){
                    state = LAND;
                }
                
                break;
            }

            case LAND:{
                landing_client.call(land_cmd);
                if (land_cmd.response.success){
                    ROS_INFO("Vehicle landed!");
                    if (RESULT_SAVE == true){
                        save_result();
                    }
                    return 0;
                }
                break;
            }
        }

        ros::spinOnce();
        ros::Duration(rate).sleep();
    }

    return 0;
}