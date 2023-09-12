#include <airo_trajectory/airo_trajectory_server.h>

int i = 0;
 
enum State{
    TAKEOFF,
    POSE_YAW,
    POSE_TWIST,
    LAND
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_mission_node");
    ros::NodeHandle nh;
    int FSM_FREQUENCY;
    nh.getParam("/airo_control_node/fsm/fsm_frequency",FSM_FREQUENCY);
    ros::Rate rate(20.0);
    AIRO_TRAJECTORY_SERVER airo_trajectory_server(nh);

    State state = TAKEOFF;

    while(ros::ok()){
        switch(state){
            case TAKEOFF:{
                if(airo_trajectory_server.takeoff()){
                    state = POSE_YAW;
                }
                break;
            }

            case POSE_YAW:{
                airo_trajectory_server.pose_cmd(target_points[i],target_yaw[i]);
                if(airo_trajectory_server.target_reached(target_points[i])){
                    i += 1;
                    if(i == target_yaw.size()){
                        state = POSE_TWIST;
                    }
                }
                break;
            }

            case POSE_TWIST:{
                airo_trajectory_server.pose_cmd(target_points[0],target_twist);
                if(airo_trajectory_server.target_reached(target_points[0])){
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