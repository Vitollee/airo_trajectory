#include <airo_trajectory/airo_trajectory_server.h>

int i = 0;
 
enum State{
    TAKEOFF,
    POSE_YAW,
    POSE_TWIST,
    TRAJ_FILE_INIT,
    TRAJ_FILE,
    LAND
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_mission_node");
    ros::NodeHandle nh;
    ros::Rate rate(20.0);
    AIRO_TRAJECTORY_SERVER airo_trajectory_server(nh);

    std::string TRAJ_FILE_NAME,TRAJ_FILE_PATH;
    nh.getParam("/airo_trajectory/traj_file_name", TRAJ_FILE_NAME);
    TRAJ_FILE_PATH = ros::package::getPath("airo_trajectory") + "/traj/" + TRAJ_FILE_NAME;
    std::vector<std::vector<double>> trajectory_data;
    airo_trajectory_server.file_traj_init(TRAJ_FILE_PATH,trajectory_data);
    int row_to_read;

    State state = TAKEOFF;

    while(ros::ok()){
        switch(state){
            case TAKEOFF:{
                if(airo_trajectory_server.takeoff()){
                    state = TRAJ_FILE_INIT;
                }
                break;
            }

            case TRAJ_FILE_INIT:{
                airo_trajectory_server.pose_cmd(airo_trajectory_server.get_start_point(trajectory_data));
                if(airo_trajectory_server.target_reached(airo_trajectory_server.get_start_point(trajectory_data))){
                    state = TRAJ_FILE;
                    row_to_read = 0;
                }
                break;
            }

            case TRAJ_FILE:{
                if(airo_trajectory_server.file_cmd(trajectory_data, row_to_read) && // Reached end of file
                airo_trajectory_server.target_reached(airo_trajectory_server.get_end_point(trajectory_data))){
                    state = LAND;
                }
                break;
            }

            case LAND:{
                airo_trajectory_server.land();
                if(airo_trajectory_server.land()){
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