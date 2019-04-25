/** \file
* this is the ethercat master running in ros environment
* soem is the only required package
* root is required to run this node
* developed by: Hongxu Guo
* Date: 4-24-19
* Special Thanks to Han YI for the help
*
* version: 1.0
*/


#include <ros/ros.h>
#include <std_msgs/String.h>
#include <sstream>
#include "eCAT_ROS.h"
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <std_msgs/Int16MultiArray.h>
#include <vector>

int eCAT_Data[16];
void arrayCallback(const std_msgs::Int16MultiArray::ConstPtr& array);
int oloop[];
int iloop[];
int8_t masterOutArray[32];

int main(int argc, char **argv)
{
  ros::init(argc, argv, "ecat_ros");
  int iret1;
  printf("SOEM (Simple Open EtherCAT Master)\nSimple test\n");

  if (argc > 1)
  {
    /* create thread to handle slave error handling in OP */
    iret1 = pthread_create(&thread1, NULL, ecatcheck, (void *)&ctime);
    /* start cyclic part */
    mainTasks(argv[1]);
  }
  else
  {
    printf("Usage: simple_test ifname1\nifname = eth0 for example\n");
  }

  printf("End program\n");

  return 0;
}
void mainTasks(char *ifname)
{

  ros::NodeHandle n;
  ros::Publisher pub = n.advertise<std_msgs::Int16MultiArray>("ecat_ms_in", 1000);
  ros::Subscriber sub = n.subscribe("ecat_ms_out", 100, arrayCallback);

  ros::Rate loop_rate(100);
  std_msgs::Int16MultiArray masterInArray;
  int i, j, wkc_count, chk;
  needlf = FALSE;
  inOP = FALSE;

  printf("Starting mainTasks\n");

  /* initialise SOEM, bind socket to ifname */
  if (ec_init(ifname))
  {
    printf("ec_init on %s succeeded.\n", ifname);
    /* find and auto-config slaves */

    if (ec_config_init(FALSE) > 0)
    {
      printf("%d slaves found and configured.\n", ec_slavecount);

      ec_config_map(&IOmap);

      ec_configdc();

      printf("Slaves mapped, state to SAFE_OP.\n");
      /* wait for all slaves to reach SAFE_OP state */
      ec_statecheck(0, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE * 4);
      for(int k = 0; k<ec_slavecount; k++)
      {
        oloop[k] = ec_slave[k].Obytes;
        if ((oloop[k] == 0) && (ec_slave[k].Obits > 0))
        oloop[k] = 1;
        iloop[k] = ec_slave[k].Ibytes;
        if ((iloop[k] == 0) && (ec_slave[k].Ibits > 0))
        iloop[k] = 1;
        printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

        printf("Request operational state for all slaves\n");
        expectedWKC = (ec_group[k].outputsWKC * 2) + ec_group[k].inputsWKC;
        printf("Calculated workcounter %d\n", expectedWKC);
        ec_slave[k].state = EC_STATE_OPERATIONAL;
        /* send one valid process data to make outputs in slaves happy*/
        ec_send_processdata();
        ec_receive_processdata(EC_TIMEOUTRET);
        /* request OP state for all slaves */
        ec_writestate(k);
        chk = 40;
        /* wait for all slaves to reach OP state */
        do
        {
          ec_send_processdata();
          ec_receive_processdata(EC_TIMEOUTRET);
          ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
        } while (chk-- && (ec_slave[k].state != EC_STATE_OPERATIONAL));
      }
      while (ros::ok())
      {
        //for now we only use 1D array for tha autonomous cor. will change later if more slave presents

        masterInArray.data.clear();
        for (int k = 0; k < ec_slavecount; k++)
        {
          ec_statecheck(k, EC_STATE_OPERATIONAL, 50000);
          if (ec_slave[k].state == EC_STATE_OPERATIONAL)
          {
            printf("Operational state reached for all slaves.\n");
            wkc_count = 0;
            inOP = TRUE;
            /* cyclic loop */
            //ros message
            for (j = 0; j < oloop[k]; j++)
            {
              *(ec_slave[k].outputs+j) = masterOutArray[j];
            }
            ec_send_processdata();
            wkc = ec_receive_processdata(EC_TIMEOUTRET);
            //
            if (wkc >= expectedWKC)
            {
              for (j = 0; j < (iloop[k]/2); j++)
              {
                int16_t outdata = ((*(ec_slave[k].inputs + 2*j))<<8) | (*(ec_slave[k].inputs + (2*j + 1)));
                masterInArray.data.push_back(outdata);
              }
              needlf = TRUE;
              inOP = FALSE;
            }
          }
          else
          {
            printf("Not all slaves reached operational state.\n");
            ec_readstate();
            if (ec_slave[k].state != EC_STATE_OPERATIONAL)
            {
              printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
              k, ec_slave[k].state, ec_slave[k].ALstatuscode, ec_ALstatuscode2string(ec_slave[k].ALstatuscode));
              printf("\nRequest init state for all slaves\n");
              ec_slave[k].state = EC_STATE_INIT;

              /* request INIT state for all slaves */
              ec_writestate(k);
            }
          }

        }
        pub.publish(masterInArray);
        ros::spinOnce();
        loop_rate.sleep();
      }
    }
    else
    {
      printf("No slaves found!\n");
    }
    printf("End, close socket\n");
    /* stop SOEM, close socket */
    ec_close();
  }
  else
  {
    printf("No socket connection on %s\nExcecute as root\n", ifname);
  }
}

void arrayCallback(const std_msgs::Int16MultiArray::ConstPtr& array)
{
  std::fill( masterOutArray, masterOutArray + sizeof( masterOutArray ), 0 );
  int size = sizeof(array->data);
  int k = 0;
  for (int i = 0; i < size; i++)
  {
    masterOutArray[k] = (int8_t)(array->data[i] >> 8);
    k++;
    masterOutArray[k] = (int8_t)(array->data[i]);
    k++;
  }
  return;
}
