#include "remover/Remover.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "remover");
    
    Remover remover;
    remover.run();

    ros::spin();

    return 0;
}