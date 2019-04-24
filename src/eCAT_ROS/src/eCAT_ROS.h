/** \file
 * this is the ethercat master running in ros environment
 * soem is the only required package
 * this node is modified from simple_test.c
 * root is required to run this node
 * developed by: Hongxu Guo
 * Date: 4-24-19
 * Special Thanks to Han YI for the help
 *
 * version: 1.0
 */

#include <stdio.h>
#include <string.h>
#include <sys/time.h>
#include <unistd.h>
#include <pthread.h>

#include "ethercattype.h"
#include "nicdrv.h"
#include "ethercatbase.h"
#include "ethercatmain.h"
#include "ethercatdc.h"
#include "ethercatcoe.h"
#include "ethercatfoe.h"
#include "ethercatconfig.h"
#include "ethercatprint.h"

#define EC_TIMEOUTMON 500

char IOmap[4096];
pthread_t thread1;
int expectedWKC;
boolean needlf;
volatile int wkc;
boolean inOP;
uint8 currentgroup = 0;

void mainTasks(char *ifname)
{
    int i, j, oloop, iloop, wkc_count, chk;
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
            for (int k = 0; k < ec_slavecount; i++)
            {
              oloop = ec_slave[k].Obytes;
              if ((oloop == 0) && (ec_slave[k].Obits > 0))
                  oloop = 1;
              // if (oloop > 8)
              //     oloop = 8;
              iloop = ec_slave[k].Ibytes;
              if ((iloop == 0) && (ec_slave[k].Ibits > 0))
                  iloop = 1;
              // if (iloop > 8)
              //     iloop = 8;

              printf("segments : %d : %d %d %d %d\n", ec_group[0].nsegments, ec_group[0].IOsegment[0], ec_group[0].IOsegment[1], ec_group[0].IOsegment[2], ec_group[0].IOsegment[3]);

              printf("Request operational state for all slaves\n");
              expectedWKC = (ec_group[0].outputsWKC * 2) + ec_group[0].inputsWKC;
              printf("Calculated workcounter %d\n", expectedWKC);
              ec_slave[k].state = EC_STATE_OPERATIONAL;
              /* send one valid process data to make outputs in slaves happy*/
              ec_send_processdata();
              ec_receive_processdata(EC_TIMEOUTRET);
              /* request OP state for all slaves */
              ec_writestate(0);
              chk = 40;
              /* wait for all slaves to reach OP state */
              do
              {
                  ec_send_processdata();
                  ec_receive_processdata(EC_TIMEOUTRET);
                  ec_statecheck(0, EC_STATE_OPERATIONAL, 50000);
              } while (chk-- && (ec_slave[k].state != EC_STATE_OPERATIONAL));
              if (ec_slave[k].state == EC_STATE_OPERATIONAL)
              {
                  printf("Operational state reached for all slaves.\n");
                  wkc_count = 0;
                  inOP = TRUE;
                  /* cyclic loop */
                  for (i = 1; i <= 100; i++)
                  {
                      ec_send_processdata();
                      wkc = ec_receive_processdata(EC_TIMEOUTRET);

                      if (wkc >= expectedWKC)
                      {
                          printf("Processdata cycle %4d, WKC %d , O:", i, wkc);

                          for (j = 0; j < oloop; j++)
                          {
                              printf(" %2.2x", *(ec_slave[k].outputs + j));
                          }

                          printf(" I:");
                          for (j = 0; j < iloop; j++)
                          {
                              printf(" %2.2x", *(ec_slave[k].inputs + j));
                          }
                          printf(" T:%lld\r", ec_DCtime);
                          needlf = TRUE;
                      }
                      usleep(5000);
                  }
                  inOP = FALSE;
              }
              else
              {
                  printf("Not all slaves reached operational state.\n");
                  ec_readstate();
                  for (i = 1; i <= ec_slavecount; i++)
                  {
                      if (ec_slave[i].state != EC_STATE_OPERATIONAL)
                      {
                          printf("Slave %d State=0x%2.2x StatusCode=0x%4.4x : %s\n",
                                 i, ec_slave[i].state, ec_slave[i].ALstatuscode, ec_ALstatuscode2string(ec_slave[i].ALstatuscode));
                      }
                  }
              }
              printf("\nRequest init state for all slaves\n");
              ec_slave[k].state = EC_STATE_INIT;

              /* request INIT state for all slaves */
              ec_writestate(0);
            }

        }
        else
        {
            printf("No slaves found!\n");
        }
        printf("End simple test, close socket\n");
        /* stop SOEM, close socket */
        ec_close();
    }
    else
    {
        printf("No socket connection on %s\nExcecute as root\n", ifname);
    }
}

void *ecatcheck(void *ptr)
{
    int slave;

    while (1)
    {
        if (inOP && ((wkc < expectedWKC) || ec_group[currentgroup].docheckstate))
        {
            if (needlf)
            {
                needlf = FALSE;
                printf("\n");
            }
            /* one ore more slaves are not responding */
            ec_group[currentgroup].docheckstate = FALSE;
            ec_readstate();
            for (slave = 1; slave <= ec_slavecount; slave++)
            {
                if ((ec_slave[slave].group == currentgroup) && (ec_slave[slave].state != EC_STATE_OPERATIONAL))
                {
                    ec_group[currentgroup].docheckstate = TRUE;
                    if (ec_slave[slave].state == (EC_STATE_SAFE_OP + EC_STATE_ERROR))
                    {
                        printf("ERROR : slave %d is in SAFE_OP + ERROR, attempting ack.\n", slave);
                        ec_slave[slave].state = (EC_STATE_SAFE_OP + EC_STATE_ACK);
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state == EC_STATE_SAFE_OP)
                    {
                        printf("WARNING : slave %d is in SAFE_OP, change to OPERATIONAL.\n", slave);
                        ec_slave[slave].state = EC_STATE_OPERATIONAL;
                        ec_writestate(slave);
                    }
                    else if (ec_slave[slave].state > 0)
                    {
                        if (ec_reconfig_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d reconfigured\n", slave);
                        }
                    }
                    else if (!ec_slave[slave].islost)
                    {
                        /* re-check state */
                        ec_statecheck(slave, EC_STATE_OPERATIONAL, EC_TIMEOUTRET);
                        if (!ec_slave[slave].state)
                        {
                            ec_slave[slave].islost = TRUE;
                            printf("ERROR : slave %d lost\n", slave);
                        }
                    }
                }
                if (ec_slave[slave].islost)
                {
                    if (!ec_slave[slave].state)
                    {
                        if (ec_recover_slave(slave, EC_TIMEOUTMON))
                        {
                            ec_slave[slave].islost = FALSE;
                            printf("MESSAGE : slave %d recovered\n", slave);
                        }
                    }
                    else
                    {
                        ec_slave[slave].islost = FALSE;
                        printf("MESSAGE : slave %d found\n", slave);
                    }
                }
            }
            if (!ec_group[currentgroup].docheckstate)
                printf("OK : all slaves resumed OPERATIONAL.\n");
        }
        usleep(10000);
    }
}
