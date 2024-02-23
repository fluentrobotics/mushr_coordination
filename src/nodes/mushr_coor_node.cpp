#include "mushr_coordination/mushr_coordination.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "mushr_coordination");
    ros::NodeHandle nh;

    ros::Duration(1.0).sleep(); //for debug attach

    node_mushr_coor::MushrCoordination node(nh);

    ros::spin();
    return 0;
}
