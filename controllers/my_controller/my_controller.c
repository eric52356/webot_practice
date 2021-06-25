/*
 * File:          my_controller.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/motor.h>
#include <webots/robot.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <webots/position_sensor.h>
#include <webots/distance_sensor.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 64
#define radius 0.019315
#define distance 0.16
#define wheel 3.14*0.055/4


void foward(double lav,double rav,WbDeviceTag lmotor,WbDeviceTag rmotor);
void right(double lav,double rav,WbDeviceTag lmotor,WbDeviceTag rmotor);
void left(double lav,double rav,WbDeviceTag lmotor,WbDeviceTag rmotor);
int chbd(char x);
int wall(int x);
/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  char str[100];
  FILE *fp=fopen("MotionPlan.txt","r");
  FILE *cw=fopen("MotionExecution.csv","w");
  fgets(str,100,fp);
  printf("%s\n",str);
     // 將訊息輸出至 stderr
  wb_robot_init();
  
  
   /* You should declare here WbDeviceTag variables for storing
    robot devices like this:*/
     WbDeviceTag frontd = wb_robot_get_device("ds0");
     wb_distance_sensor_enable(frontd,TIME_STEP);
     WbDeviceTag rightd = wb_robot_get_device("ds1");
     wb_distance_sensor_enable(rightd,TIME_STEP);
     WbDeviceTag leftd = wb_robot_get_device("ds2");          
     wb_distance_sensor_enable(leftd,TIME_STEP);
     WbDeviceTag lmotor = wb_robot_get_device("left wheel motor");
     WbDeviceTag rmotor = wb_robot_get_device("right wheel motor");
     WbDeviceTag lsensor = wb_robot_get_device("left wheel sensor");
     WbDeviceTag rsensor = wb_robot_get_device("right wheel sensor");
     wb_position_sensor_enable(lsensor,TIME_STEP);
     wb_position_sensor_enable(rsensor,TIME_STEP);
     double lav;//左速度
     double rav;//右速度
     double xl=0;//左距離
     double xr=0;//右距離
     double ol=0;//左原點
     double or=0;//右原點
     char facename[5]={" SENW"};
     int face=1;
     int i=3;//計算指令
     int step=0;//計算步數
     int ch=1;
     int row=0,column=0;//行欄
     int frontwall,rightwall,leftwall;
  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
    wb_motor_set_position(rmotor, INFINITY);
    wb_motor_set_position(lmotor, INFINITY);
    wb_motor_set_velocity(lmotor, 0.0);
    wb_motor_set_velocity(rmotor, 0.0);
  while (wb_robot_step(TIME_STEP) != -1) {
    //double time=wb_robot_get_time();
    frontwall=wb_distance_sensor_get_value(frontd);
    rightwall=wb_distance_sensor_get_value(rightd);
    leftwall=wb_distance_sensor_get_value(leftd);
    if(step==0)
    {
      printf("Step:%03d, Row:0, Column:0, Heading:%c, Left Wall:N, Front Wall:N, Right Wall:Y\n",step,facename[face]);
      fprintf(cw,"Step,Row,Column,Heading,Leftwall,Frontwall,Rightwall\n");
      fprintf(cw,"%d,%d,%d,S,N,N,Y\n",step,row,column);
      step++;
    }
    if(ch==1)
    {
      ol=wb_position_sensor_get_value(lsensor);
      or=wb_position_sensor_get_value(rsensor);    
      ch=0;
    }
    if(str[i]!='\0')
    {
      lav=wb_position_sensor_get_value(lsensor);
      rav=wb_position_sensor_get_value(rsensor);
      xl=radius*(lav-ol);
      xr=radius*(rav-or);
      if(str[i]=='F')
      {
          //ch=0;          
        foward(lav,rav,lmotor,rmotor);
        if(xl>distance&&xr>distance)
        {
           if(facename[face]=='N'||facename[face]=='S')
           row=row+chbd(facename[face]);
           if(facename[face]=='E'||facename[face]=='W')
           column=column+chbd(facename[face]);
           frontwall=wall(frontwall);
           rightwall=wall(rightwall);
           leftwall=wall(leftwall);
           printf("Step:%03d, Row:%d, Column:%d, Heading:%c, Left Wall:%c, Front Wall:%c, Right Wall:%c\n",step,row,column,facename[face],leftwall,frontwall,rightwall);
           fprintf(cw,"%d,%d,%d,%c,%c,%c,%c\n",step,row,column,facename[face],leftwall,frontwall,rightwall);
           wb_motor_set_velocity(lmotor,0.0);
           wb_motor_set_velocity(rmotor,0.0);
           i++;
           ch=1;
           xr=0;
           xl=0;
           step++;
         }
      }
      if(str[i]=='R')
      {
          //ch=0;
        if(ch==1)
        {
          ol=wb_position_sensor_get_value(lsensor);
          or=wb_position_sensor_get_value(rsensor);    
          ch=0;
        }          
        right(lav,rav,lmotor,rmotor);
        if(xl>wheel)
        {
          face--;
          if(face==0)
          face=4;
          frontwall=wall(frontwall);
          rightwall=wall(rightwall);
          leftwall=wall(leftwall);
          printf("Step:%03d, Row:%d, Column:%d, Heading:%c, Left Wall:%c, Front Wall:%c, Right Wall:%c\n",step,row,column,facename[face],leftwall,frontwall,rightwall);
          fprintf(cw,"%d,%d,%d,%c,%c,%c,%c\n",step,row,column,facename[face],leftwall,frontwall,rightwall);
          wb_motor_set_velocity(lmotor,0.0);
          wb_motor_set_velocity(rmotor,0.0);
          i++;
          ch=1;
          xr=0;
          xl=0;
          step++;
        }
      }
      if(str[i]=='L')
      {
          //ch=0;
        if(ch==1)
        {
          ol=wb_position_sensor_get_value(lsensor);
          or=wb_position_sensor_get_value(rsensor);    
          ch=0;
        }          
        left(lav,rav,lmotor,rmotor);
          //printf("%f %f %f\n",xr,xl,wheel);
          //printf("%f--%f\n",lav,rav);
        if(xr>wheel)
        {
          face++;
          if(face==5)
          face=1;
          frontwall=wall(frontwall);
          rightwall=wall(rightwall);
          leftwall=wall(leftwall);
          printf("Step:%03d, Row:%d, Column:%d, Heading:%c, Left Wall:%c, Front Wall:%c, Right Wall:%c\n",step,row,column,facename[face],leftwall,frontwall,rightwall);
          fprintf(cw,"%d,%d,%d,%c,%c,%c,%c\n",step,row,column,facename[face],leftwall,frontwall,rightwall);
          wb_motor_set_velocity(lmotor,0.0);
          wb_motor_set_velocity(rmotor,0.0);
          i++;
          ch=1;
          xr=0;
          xl=0;
          step++;
        }
      }
     }
     else
     {
     fclose(cw);
     break;
     }

    
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */
    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
  };


  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_robot_cleanup();

  return 0;
}

void foward(double lav,double rav,WbDeviceTag lmotor,WbDeviceTag rmotor)
{
  wb_motor_set_velocity(lmotor, 2.0);
  wb_motor_set_velocity(rmotor, 2.0);  
}

void right(double lav,double rav,WbDeviceTag lmotor,WbDeviceTag rmotor)
{
  wb_motor_set_velocity(lmotor, 1.0);
  wb_motor_set_velocity(rmotor, -1.0);
}  

void left(double lav,double rav,WbDeviceTag lmotor,WbDeviceTag rmotor)
{
  wb_motor_set_velocity(lmotor, -1.0);
  wb_motor_set_velocity(rmotor, 1.0); 
}

int chbd(char x)
{
  switch (x)
  {
    case 'S':
      return 1;
    case 'E':
      return 1;
    case 'N':
      return -1;
    case 'W':
      return -1;
   }
   return 0;
}
int wall(int x)
{
  if(x<900)
  return 89;
  else
  return 78;
}