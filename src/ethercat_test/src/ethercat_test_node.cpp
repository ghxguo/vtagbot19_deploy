
#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include "ethercat_test.h"


int main(int argc, char **argv)
{
    
    ros::init(argc, argv, "ethercat_test");
    ros::NodeHandle n;

    int iret1;
    printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

    if (argc > 1)
    {
        /* create thread to handle slave error handling in OP */
        iret1 = pthread_create(&thread1, NULL, ecatcheck, (void *)&ctime);
        /* start cyclic part */
        simpletest(argv[1]);
    }
    else
    {
        printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
    }

    printf("End program\n");

    return 0;
}