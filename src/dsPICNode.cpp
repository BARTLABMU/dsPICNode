#include <ros/ros.h>
#include <cereal_port/CerealPort.h>
#include "std_msgs/String.h"
#include "dsPICNode/MotorControl.h"
#include "std_msgs/Int16.h"
#include <boost/lexical_cast.hpp>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/Float32MultiArray.h>

#define REPLY_SIZE 1024
#define TIMEOUT 1000
using namespace std;
cereal::CerealPort device;
int speed = 50;             //robot speed 0-99
int step_move = 5;          // step size of flipper
int step_mani = 5;          // step size of Manipulator
int step_wrist = 50;        // step size of AX12
int FL_angle = 0;           // initial flipper FL 0-360
int FR_angle = 0;           // initial flipper FR 0-360
int BL_angle =0;            // initial flipper BL 0-360
int BR_angle =0;            // initial flipper BR 0-360
int j1_angle =90;           // initial Manipulator joint 1 --> j1_min - j1_max  0-180
int j2_angle =0;            // initial Manipulator joint 2 --> j2_min - j2_max
int j3_angle =0;            // initial Manipulator joint 3 --> j3_min - j3_max
int j4_angle =0;            // initial Manipulator joint 4 --> j4_min - j4_max
int j5_angle =512;          // initial servo wrist
int j6_angle = 512;         // initial servo tilt
int j7_angle = 0;
int en_FL = 0;              // read angle value encoder
int en_FR = 0;              // read angle value encoder
int en_BL = 0;              // read angle value encoder
int en_BR =0;               // read angle value encoder
int en_j1 =0, en_j2=0,en_j3=0,en_j4=0, en_j5=0,en_j6=0;
int j1_max = 180;           // maximum angle Joint 1 manipulator
int j1_min = 0;             // minimum angle Joint 1 manipulator
int j2_max = 85;
int j2_min = 0;
int j3_max = 130;
int j3_min = 0;
int j4_max = 200;
int j4_min = 0;
int j5_max = 999;          // maximum angle servo wrist
int j5_min = 0;             // minimum angle servo wrist
int j6_max = 799;
int j6_min = 330;
//----------------send communcition parameter to master----------//
string FL = "A";
string FR = "B";
string FL_L;
string FL_R;
string BR_R;
string BL_L;
string j_1;
string j_2;
string j_3;
string j_4;
string j_5;
string j_6;
string j_7;
char reply[50];         // read encoder from master to this buffer
char Mreply[50];         // read encoder from master to this buffer
char* A;                // pointer for extract each flipper
char* J;
ros::Publisher Leg_pos; // publish flipper angle system (fl fr bl br)
std_msgs::Int16MultiArray fliper_pos;  // parameter for Leg_pos publisher
std::vector<short> pos; // change paremeter to ROS system parameter

//-------------------control Manipulator ------------------//
void mani_pos(int a1,int a2, int a3, int a4)
{
    if(a1>j1_max)
    {
        a1 = j1_max;
    }
    if(a1<j1_min)
    {
        a1 = j1_min;
    }
    if(a2>j2_max)
    {
        a2 = j2_max;
    }
    if(a2<j1_min)
    {
        a2 = j2_min;
    }
    if(a3>j3_max)
    {
        a3 = j3_max;
    }
    if(a3<j3_min)
    {
        a3 = j3_min;
    }
    if(a4>j4_max)
    {
        a4 = j4_max;
    }
    if(a4<j4_min)
    {
        a4 = j4_min;
    }

    //Format to position control "Address direction spd1 spd2 pos1 pos2 pos3 j"  8 letters//
    // G=J1 H=J2 I=J3 J=J4 K=J5 L=J6
    j_1 = "Gl";
    string s = boost::lexical_cast<string>( a1 );   // "lexical_cast" change int (a1) to string (s)
    if(a1 >=100)
    {
    j_1.append("70");               // add string to
    j_1.append(s);
    j_1.append("j");
      }
    else if(a1 < 100&& a1 >= 10 )               // chk pos not more than 3 base
    {
        j_1.append("700");
        j_1.append(s);
        j_1.append("j");
    }
    else if(a1< 10)
    {
        j_1.append("7000");
        j_1.append(s);
        j_1.append("j");
    }
    ROS_INFO("%s",j_1.c_str());

        ros::Duration(0.01).sleep(); device.write(j_1.c_str(),8);  // Ros sleep in this node only
       j1_angle = a1;                           // update pos to monitor

       if(a2 > 180)
       {
           a2 = 180;
       }
       j_2 = "Hl";
        s = boost::lexical_cast<string>( a2 );
       if(a2 >=100)
       {
       j_2.append("05");
       j_2.append(s);
       j_2.append("j");
         }
       else if(a2 < 100&& a2 >= 10 )
       {
           j_2.append("050");
           j_2.append(s);
           j_2.append("j");
       }
       else if(a2 < 10)
       {
           j_2.append("0500");
           j_2.append(s);
           j_2.append("j");
       }
       ROS_INFO("%s",j_2.c_str());

           ros::Duration(0.01).sleep(); device.write(j_2.c_str(),8);
          j2_angle = a2;


          j_3 = "Il";
          s = boost::lexical_cast<string>( a3 );
          if(a3 >=100)
          {
          j_3.append("99");
          j_3.append(s);
          j_3.append("j");
            }
          else if(a3 < 100&& a3 >= 10 )
          {
              j_3.append("990");
              j_3.append(s);
              j_3.append("j");
          }
          else if(a3 < 10)
          {
              j_3.append("9900");
              j_3.append(s);
              j_3.append("j");
          }
          ROS_INFO("%s",j_3.c_str());

              ros::Duration(0.5).sleep(); device.write(j_3.c_str(),8);
             j3_angle = a3;

             j_4 = "Jl";
             s = boost::lexical_cast<string>( a4 );
             if(a4 >=100)
             {
             j_4.append("99");
             j_4.append(s);
             j_4.append("j");
               }
             else if(a4 < 100&& a4 >= 10 )
             {
                 j_4.append("990");
                 j_4.append(s);
                 j_4.append("j");
             }
             else if(a4 < 10)
             {
                 j_4.append("9900");
                 j_4.append(s);
                 j_4.append("j");
             }
             ROS_INFO("%s",j_4.c_str());

                 ros::Duration(0.01).sleep(); device.write(j_4.c_str(),8);
                j4_angle = a4;


}
//------------------Callback from Robot control node------//
//--------------to cmd master to move---------------------//
void teleop_callback(const std_msgs::StringConstPtr& msg)
{   string s;                       // robot spd string
    int old_FL = FL_angle;          // to chk last position to know next direction
    int old_FR = FR_angle;          // if current pos - old pos >0 so then go +direction
    int old_BL = BL_angle;          // if current pos - old pos <0 so then go -direction
    int old_BR = BR_angle;

 //   ROS_INFO("FL: %d, FR: %d, BL: %d, BR: %d",FL_angle,FR_angle,BL_angle,BR_angle);  // SHOW value in window
   // ROS_INFO("j1: %d, j2: %d, j3: %d, j4: %d",j1_angle,j2_angle,j3_angle,j4_angle);


    if(msg->data == "forward")      // msg is from robot control to move "forward"
    {
        // speed control "Address direction spd1 spd2 j"
        // A = ML B=MR C=FL D=FR E=BL F=BR


        if(speed > 99)
        {

             s = boost::lexical_cast<string>( 99 );

          FL.append("l");
           FL.append(s);
            FL.append("j");
          FR.append("l");
           FR.append(s);
            FR.append("j");

        }
        else if(speed < 10)
        {

             s = boost::lexical_cast<string>( speed );

          FL.append("l0");

           FL.append(s);
            FL.append("j");
          FR.append("l0");

           FR.append(s);
            FR.append("j");
        }
                else
        {
             s = boost::lexical_cast<string>( speed );

          FL.append("l");

           FL.append(s);
            FL.append("j");

          FR.append("l");
           FR.append(s);
            FR.append("j");
        }
        ROS_INFO("%s : %s",FL.c_str(), FR.c_str());
        device.write(FL.c_str(),5);
        ros::Duration(0.02).sleep();
        device.write(FR.c_str(),5);
        ros::Duration(0.02).sleep();
        FL = "A";
        FR = "B";
    }
    else if(msg->data == "backward")
    {
        if(speed > 99)
        {

             s = boost::lexical_cast<string>( 99 );

          FL.append("r");
           FL.append(s);
            FL.append("j");

          FR.append("r");
           FR.append(s);
            FR.append("j");

        }
        else if(speed < 10)
        {

             s = boost::lexical_cast<string>( speed );

          FL.append("r0");

           FL.append(s);
            FL.append("j");

          FR.append("r0");
           FR.append(s);
            FR.append("j");
        }
                else
        {
             s = boost::lexical_cast<string>( speed );

          FL.append("r");
           FL.append(s);
            FL.append("j");

          FR.append("r");
           FR.append(s);
            FR.append("j");
        }
        ROS_INFO("%s : %s",FL.c_str(), FR.c_str());
        device.write(FL.c_str(),5);
            ros::Duration(0.02).sleep();
        device.write(FR.c_str(),5);
        ros::Duration(0.02).sleep();
        FL = "A";
        FR = "B";
    }
    else if(msg->data == "left")
    {
        if(speed > 99)
        {

             s = boost::lexical_cast<string>( 99 );

          FL.append("r");
           FL.append(s);
            FL.append("j");

          FR.append("l");
           FR.append(s);
            FR.append("j");

        }
        else if(speed < 10)
        {

             s = boost::lexical_cast<string>( speed );

          FL.append("r0");
           FL.append(s);
            FL.append("j");

          FR.append("l0");
           FR.append(s);
            FR.append("j");
        }
                else
        {
             s = boost::lexical_cast<string>( speed );

          FL.append("r");
           FL.append(s);
            FL.append("j");


          FR.append("l");
           FR.append(s);
            FR.append("j");
        }
        ROS_INFO("%s : %s",FL.c_str(), FR.c_str());
        device.write(FL.c_str(),5);
            ros::Duration(0.02).sleep();
        device.write(FR.c_str(),5);
        ros::Duration(0.02).sleep();
        FL = "A";
        FR = "B";
    }
    else if(msg->data == "right")
    {
        if(speed > 99)
        {

             s = boost::lexical_cast<string>( 99 );

          FL.append("l");
           FL.append(s);
            FL.append("j");

          FR.append("r");
           FR.append(s);
            FR.append("j");

        }
        else if(speed < 10)
        {

             s = boost::lexical_cast<string>( speed );

          FL.append("l0");
           FL.append(s);
            FL.append("j");

          FR.append("r0");
           FR.append(s);
            FR.append("j");
        }
                else
        {
             s = boost::lexical_cast<string>( speed );

          FL.append("l");
           FL.append(s);
            FL.append("j");


          FR.append("r");
           FR.append(s);
            FR.append("j");
        }
        ROS_INFO("%s : %s",FL.c_str(), FR.c_str());
        device.write(FL.c_str(),5);
            ros::Duration(0.02).sleep();
        device.write(FR.c_str(),5);
        ros::Duration(0.02).sleep();
        FL = "A";
        FR = "B";
    }
    else if(msg->data == "right+")                  // MOVE WHEEL R ONLY
    {
        if(speed > 99)
        {

             s = boost::lexical_cast<string>( 99 );


          FR.append("r");
           FR.append(s);
            FR.append("j");

        }
        else if(speed < 10)
        {

             s = boost::lexical_cast<string>( speed );



          FR.append("r0");
           FR.append(s);
            FR.append("j");
        }
                else
        {
             s = boost::lexical_cast<string>( speed );




          FR.append("r");
           FR.append(s);
            FR.append("j");
        }

        ROS_INFO("%s" , FR.c_str());
        //device.write(FL.c_str(),5);
          //  ros::Duration(0.02).sleep();
        device.write(FR.c_str(),5);
        ros::Duration(0.02).sleep();
        FL = "A";
        FR = "B";
    }
    else if(msg->data == "left+")                   // mOVE wHEEL l ONly
    {
        if(speed > 99)
        {

             s = boost::lexical_cast<string>( 99 );

          FL.append("r");
           FL.append(s);
            FL.append("j");

         }
        else if(speed < 10)
        {

             s = boost::lexical_cast<string>( speed );

          FL.append("r0");
           FL.append(s);
            FL.append("j");


        }
                else
        {
             s = boost::lexical_cast<string>( speed );

          FL.append("r");
           FL.append(s);
            FL.append("j");


        }
      //  FR.append("s");
       // FR.append("00");
        //  FR.append("j");





        ROS_INFO(" %s",FL.c_str());
        device.write(FL.c_str(),5);
            ros::Duration(0.02).sleep();

        FL = "A";
        FR = "B";
    }
   else if(msg->data == "stop")
        {

              FL.append("s");
              FL.append("00");
                FL.append("j");

              FR.append("s");
              FR.append("00");
                FR.append("j");



            ROS_INFO("%s : %s",FL.c_str(), FR.c_str());
            device.write(FL.c_str(),5);
                ros::Duration(0.02).sleep();
            device.write(FR.c_str(),5);
            ros::Duration(0.02).sleep();
            FL = "A";
            FR = "B";
        }
    else if(msg->data == "FLup")                    // Flipper FL go +direction
    {
       //  old_FL = FL_angle;



        FL_angle = FL_angle + step_move;
        ROS_INFO("OLD %d, NEW %d", old_FL,FL_angle);
        if(FL_angle > 360)
        {
           FL_angle = 0;
        }
        if(FL_angle == 0)
        {
            FL_L = "Cr";
        }
       else if(FL_angle - old_FL > 0)
        {
            FL_L = "Cr";

        }
        else if(FL_angle - old_FL < 0)
        {
         FL_L = "Cl";
        }
        else if(FL_angle - old_FL == 0)
        {
            FL_L = "Cl";
        }
              string s = boost::lexical_cast<string>( FL_angle );


              if(FL_angle >=100)
              {
              FL_L.append("99");
              FL_L.append(s);
              FL_L.append("j");
                }
              else if(FL_angle < 100 && FL_angle >= 10)
              {
                  FL_L.append("990");
                  FL_L.append(s);
                  FL_L.append("j");
              }
              else if(FL_angle < 10)
              {
                  FL_L.append("9900");
                  FL_L.append(s);
                  FL_L.append("j");
              }


              if(FL_angle != 360 && FL_angle != 0)
              {
                 // if(FL_angle == 0)
                  //{
                  ros::Duration(0.01).sleep(); device.write(FL_L.c_str(),8);
ROS_INFO("%s UP",FL_L.c_str());
                  //}
                  //else if(en_FL==old_FL)
                  //{
                   //   ros::Duration(0.01).sleep(); device.write(FL_L.c_str(),8);
                  //}
              }



    }
    else if(msg->data == "FLdown")
    {

        old_FL = FL_angle;
         FL_angle = FL_angle - step_move;
        ROS_INFO("OLD %d, NEW %d", old_FL,FL_angle);
        if(FL_angle < 0)
        {
          FL_angle = 360;
        }
        if(FL_angle == 360)
        {
            FL_L ="Cl";
        }
        else if(FL_angle - old_FL > 0)
        {
            FL_L = "Cr";

        }
        else if(FL_angle - old_FL < 0)
        {
         FL_L = "Cl";
        }
        else if(FL_angle - old_FL == 0)
        {
            FL_L = "Cl";
        }
              string s = boost::lexical_cast<string>( FL_angle );
              if(FL_angle >=100)
              {
              FL_L.append("99");
              FL_L.append(s);
              FL_L.append("j");
                }
              else if(FL_angle < 100&& FL_angle >= 10 )
              {
                  FL_L.append("990");
                  FL_L.append(s);
                  FL_L.append("j");
              }
              else if(FL_angle < 10)
              {
                  FL_L.append("9900");
                  FL_L.append(s);
                  FL_L.append("j");
              };
              ROS_INFO("%s down",FL_L.c_str());
              if(FL_angle!=360&&FL_angle != 0)
              {
                  ros::Duration(0.01).sleep(); device.write(FL_L.c_str(),8);

              }

    }
    else if(msg->data == "FRup")
    {

        old_FR = FR_angle;
        FR_angle = FR_angle + step_move;



        if(FR_angle > 360)
        {
           FR_angle = 0;
        }


        if(FR_angle == 0)
        {
            FL_R = "Dr";
        }
       else if(FR_angle - old_FR > 0)
        {
            FL_R = "Dr";
           //   ROS_INFO("UPPPP");

        }
        else if(FR_angle - old_FR < 0)
        {
         FL_R = "Dl";
         // ROS_INFO("downnnnn");
        }
        else if(FR_angle - old_FR == 0)
        {
             //  ROS_INFO("000000000000");
            FL_R = "Dl";
        }

              string s = boost::lexical_cast<string>( FR_angle );
              if(FR_angle >=100)
              {
              FL_R.append("99");
              FL_R.append(s);
              FL_R.append("j");
                }
              else if(FR_angle < 100 && FR_angle >= 10)
              {
                  FL_R.append("990");
                  FL_R.append(s);
                  FL_R.append("j");
              }
              else if(FR_angle < 10)
              {
                  FL_R.append("9900");
                  FL_R.append(s);
                  FL_R.append("j");
              }
              ROS_INFO("%s UP",FL_R.c_str());
              if(FR_angle!=360&&FR_angle != 0)
              {
                  ros::Duration(0.01).sleep(); device.write(FL_R.c_str(),8);

              }

    }
    else if(msg->data == "FRdown")
    {

        old_FR = FR_angle;
        FR_angle = FR_angle -step_move;

        if(FR_angle < 0)
        {
          FR_angle = 360;
        }
        if(FR_angle == 360)
        {
            FL_R ="Dl";
        }
        else if(FR_angle - old_FR > 0)
        {
            FL_R = "Dr";

        }
        else if(FR_angle - old_FR < 0)
        {
         FL_R = "Dl";
        }
        else if(FR_angle - old_FR == 0)
        {
            FL_R = "Dl";
        }

              string s = boost::lexical_cast<string>( FR_angle );
              if(FR_angle >=100)
              {
              FL_R.append("99");
              FL_R.append(s);
              FL_R.append("j");
                }
              else if(FR_angle < 100 && FR_angle >= 10)
              {
                  FL_R.append("990");
                  FL_R.append(s);
                  FL_R.append("j");
              }
              else if(FR_angle < 10)
              {
                  FL_R.append("9900");
                  FL_R.append(s);
                  FL_R.append("j");
              }
              ROS_INFO("%s DOWN",FL_R.c_str());
              if(FR_angle!=360&&FR_angle != 0)
              {
                  ros::Duration(0.01).sleep(); device.write(FL_R.c_str(),8);

              }


    }

    else if(msg->data == "BLup")
    {

        old_BL = BL_angle;

        BL_angle = BL_angle +step_move;

        ROS_INFO("OLD %d, NEW %d", old_BL,BL_angle);
        if(BL_angle > 360)
        {
           BL_angle = 0;
        }
        if(BL_angle == 0)
        {
            BL_L = "Er";
        }
        else if(BL_angle - old_BL > 0)
        {
            BL_L = "Er";

        }
        else if(BL_angle - old_BL < 0)
        {
         BL_L = "El";
        }
        else if(BL_angle - old_BL == 0)
        {
            BL_L = "El";
        }
              string s = boost::lexical_cast<string>( BL_angle );
              if(BL_angle >=100)
              {
              BL_L.append("99");
              BL_L.append(s);
              BL_L.append("j");
                }
              else if(BL_angle < 100&& BL_angle >= 10 )
              {
                  BL_L.append("990");
                  BL_L.append(s);
                  BL_L.append("j");
              }
              else if(BL_angle < 10)
              {
                  BL_L.append("9900");
                  BL_L.append(s);
                  BL_L.append("j");
              }
              ROS_INFO("%s",BL_L.c_str());
              if(BL_angle!=360&&BL_angle != 0)
              {
                  ros::Duration(0.01).sleep(); device.write(BL_L.c_str(),8);

              }

    }
    else if(msg->data == "BLdown")
    {

         old_BL = BL_angle;
         BL_angle = BL_angle - step_move;
          ROS_INFO("OLD %d, NEW %d", old_BL,BL_angle);
         if(BL_angle < 0)
         {
           BL_angle = 360;
         }
         if(BL_angle == 360)
         {
             BL_L = "El";
         }
       else if(BL_angle - old_BL > 0)
         {
             BL_L = "Er";

         }
         else if(BL_angle - old_BL < 0)
         {
          BL_L = "El";
         }
         else if(BL_angle - old_BL == 0)
         {
             BL_L = "El";
         }
              string s = boost::lexical_cast<string>( BL_angle );
              if(BL_angle >=100)
              {
              BL_L.append("99");
              BL_L.append(s);
              BL_L.append("j");
                }
              else if(BL_angle < 100 && BL_angle >= 10)
              {
                  BL_L.append("990");
                  BL_L.append(s);
                  BL_L.append("j");
              }
              else if(BL_angle < 10)
              {
                  BL_L.append("9900");
                  BL_L.append(s);
                  BL_L.append("j");
              }
              ROS_INFO("%s",BL_L.c_str());
              if(BL_angle!=360&&BL_angle != 0)
              {
                  ros::Duration(0.01).sleep(); device.write(BL_L.c_str(),8);

              }
    }
    else if(msg->data == "BRup")
    {

        old_BR = BR_angle;
        BR_angle = BR_angle +step_move;
        if(BR_angle > 360)
        {
           BR_angle = 0;
        }

        if(BR_angle == 0)
        {
            BR_R = "Fr";
        }
        else if(BR_angle - old_BR > 0)
        {
            BR_R = "Fr";

        }
        else if(BR_angle - old_BR < 0)
        {
         BR_R = "Fl";
        }
        else if(BR_angle - old_BR == 0)
        {
            BR_R = "Fl";
        }

              string s = boost::lexical_cast<string>( BR_angle );
              if(BR_angle >=100)
              {
              BR_R.append("99");
              BR_R.append(s);
              BR_R.append("j");
                }
              else if(BR_angle < 100 && BR_angle >= 10 )
              {
                  BR_R.append("990");
                  BR_R.append(s);
                  BR_R.append("j");
              }
              else if(BR_angle < 10)
              {
                  BR_R.append("9900");
                  BR_R.append(s);
                  BR_R.append("j");
              }

              ROS_INFO("%s",BR_R.c_str());
              if(BR_angle!=360&&BR_angle != 0)
              {
                  ros::Duration(0.01).sleep(); device.write(BR_R.c_str(),8);

              }

    }
    else if(msg->data == "BRdown")
    {

        old_BR = BR_angle;
        BR_angle = BR_angle -step_move;
        if(BR_angle < 0)
        {
            BR_angle = 360;
        }


        if(BR_angle == 360)
        {
            BR_R = "Fl";
        }
        else if(BR_angle - old_BR > 0)
        {
            BR_R = "Fr";

        }
        else if(BR_angle - old_BR < 0)
        {
         BR_R = "Fl";
        }
        else if(BR_angle - old_BR == 0)
        {
            BR_R = "Fl";
        }

              string s = boost::lexical_cast<string>( BR_angle );
              if(BR_angle >=100)
              {
              BR_R.append("99");
              BR_R.append(s);
              BR_R.append("j");
                }
              else if(BR_angle < 100 && BR_angle >= 10)
              {
                  BR_R.append("990");
                  BR_R.append(s);
                  BR_R.append("j");
              }
              else if(BR_angle < 10)
              {
                  BR_R.append("9900");
                  BR_R.append(s);
                  BR_R.append("j");
              }
              ROS_INFO("%s",BR_R.c_str());
              if(BR_angle!=360&&BR_angle != 0)
              {
                  ros::Duration(0.01).sleep(); device.write(BR_R.c_str(),8);

              }
    }

    //-------------Special Posture ---------------//
    else if(msg->data == "down45down0")
    {
         if(FL_angle < 135)
        {
             FL_L = "Cr99135j";
             ROS_INFO("cUP %s",FL_L.c_str());
             device.write(FL_L.c_str(),8);
             FL_angle = 135;
        }
        else if((FL_angle-135) > (360-FL_angle)+135 )
        {
            FL_L = "Cr99135j";
            ROS_INFO("cUP %s",FL_L.c_str());
            device.write(FL_L.c_str(),8);
            FL_angle = 135;

        }
        else if((FL_angle)-135 <(360-FL_angle)+135)
        {
            FL_L = "Cl99135j";
            ROS_INFO("cUP %s",FL_L.c_str());
            device.write(FL_L.c_str(),8);
            FL_angle = 135;
        }

         ros::Duration(0.01).sleep();
         if(FR_angle < 135)
        {
             FL_R = "Dr99135j";
             ROS_INFO("cUP %s",FL_R.c_str());
             device.write(FL_R.c_str(),8);
             FR_angle = 135;
        }
        else if((FR_angle-135) > (360-FR_angle)+135 )
        {
            FL_R = "Dr99135j";
            ROS_INFO("cUP %s",FL_R.c_str());
            device.write(FL_R.c_str(),8);
            FR_angle = 135;

        }
        else if((FR_angle)-135 <(360-FR_angle)+135)
        {
            FL_R = "Dl99135j";
            ROS_INFO("cUP %s",FL_R.c_str());
            device.write(FL_R.c_str(),8);
            FR_angle = 135;
        }
         ros::Duration(0.01).sleep();
         if(BL_angle < 90)
        {
             BL_L = "Er99090j";
             ROS_INFO("cUP %s",BL_L.c_str());
             device.write(BL_L.c_str(),8);
             BL_angle = 90;
        }
        else if((BL_angle-90) > (360-BL_angle)+90 )
        {
            BL_L = "Er99090j";
            ROS_INFO("cUP %s",BL_L.c_str());
            device.write(BL_L.c_str(),8);
            BL_angle = 90;

        }
        else if((BL_angle)-90 <(360-BL_angle)+90)
        {
            BL_L = "El99090j";
            ROS_INFO("cUP %s",BL_L.c_str());
            device.write(BL_L.c_str(),8);
            BL_angle = 90;
        }

         ros::Duration(0.01).sleep();

         if(BR_angle < 90)
        {
             BR_R = "Fr99090j";
             ROS_INFO("cUP %s",BR_R.c_str());
             device.write(BR_R.c_str(),8);
             BR_angle = 90;
        }
        else if((BR_angle-90) > (360-BR_angle)+90 )
        {
            BR_R = "Fr99090j";
            ROS_INFO("cUP %s",BR_R.c_str());
            device.write(BR_R.c_str(),8);
            BR_angle = 90;

        }
        else if((BR_angle)-90 <(360-BR_angle)+90)
        {
            BR_R = "Fl99090j";
            ROS_INFO("cUP %s",BR_R.c_str());
            device.write(BR_R.c_str(),8);
            BR_angle = 90;
        }

    }

    else if(msg->data == "down45down45")
    {
         if(FL_angle < 135)
        {
             FL_L = "Cr99135j";
             ROS_INFO("cUP %s",FL_L.c_str());
             device.write(FL_L.c_str(),8);
             FL_angle = 135;
        }
        else if((FL_angle-135) > (360-FL_angle)+135 )
        {
            FL_L = "Cr99135j";
            ROS_INFO("cUP %s",FL_L.c_str());
            device.write(FL_L.c_str(),8);
            FL_angle = 135;

        }
        else if((FL_angle)-135 <(360-FL_angle)+135)
        {
            FL_L = "Cl99135j";
            ROS_INFO("cUP %s",FL_L.c_str());
            device.write(FL_L.c_str(),8);
            FL_angle = 135;
        }

         ros::Duration(0.01).sleep();
         if(FR_angle < 135)
        {
             FL_R = "Dr99135j";
             ROS_INFO("cUP %s",FL_R.c_str());
             device.write(FL_R.c_str(),8);
             FR_angle = 135;
        }
        else if((FR_angle-135) > (360-FR_angle)+135 )
        {
            FL_R = "Dr99135j";
            ROS_INFO("cUP %s",FL_R.c_str());
            device.write(FL_R.c_str(),8);
            FR_angle = 135;

        }
        else if((FR_angle)-135 <(360-FR_angle)+135)
        {
            FL_R = "Dl99135j";
            ROS_INFO("cUP %s",FL_R.c_str());
            device.write(FL_R.c_str(),8);
            FR_angle = 135;
        }
         ros::Duration(0.01).sleep();
         if(BL_angle < 135)
        {
             BL_L = "Er99135j";
             ROS_INFO("cUP %s",BL_L.c_str());
             device.write(BL_L.c_str(),8);
             BL_angle = 135;
        }
        else if((BL_angle-135) > (360-BL_angle)+135 )
        {
            BL_L = "Er99135j";
            ROS_INFO("cUP %s",BL_L.c_str());
            device.write(BL_L.c_str(),8);
            BL_angle = 135;

        }
        else if((BL_angle)-135 <(360-BL_angle)+135)
        {
            BL_L = "El99135j";
            ROS_INFO("cUP %s",BL_L.c_str());
            device.write(BL_L.c_str(),8);
            BL_angle = 135;
        }

         ros::Duration(0.01).sleep();

         if(BR_angle < 135)
        {
             BR_R = "Fr99135j";
             ROS_INFO("cUP %s",BR_R.c_str());
             device.write(BR_R.c_str(),8);
             BR_angle = 135;
        }
        else if((BR_angle-135) > (360-BR_angle)+135 )
        {
            BR_R = "Fr99135j";
            ROS_INFO("cUP %s",BR_R.c_str());
            device.write(BR_R.c_str(),8);
            BR_angle = 135;

        }
        else if((BR_angle)-135 <(360-BR_angle)+135)
        {
            BR_R = "Fl99135j";
            ROS_INFO("cUP %s",BR_R.c_str());
            device.write(BR_R.c_str(),8);
            BR_angle = 135;
        }

    }
    else if(msg->data == "up0up45")
    {
        if(FL_angle < 90)
       {
            FL_L = "Cr99090j";
            ROS_INFO("cUP %s",FL_L.c_str());
            device.write(FL_L.c_str(),8);
            FL_angle = 90;
       }
       else if((FL_angle-90) > (360-FL_angle)+90 )
       {
           FL_L = "Cr99090j";
           ROS_INFO("cUP %s",FL_L.c_str());
           device.write(FL_L.c_str(),8);
           FL_angle = 90;

       }
       else if((FL_angle)-90 <(360-FL_angle)+90)
       {
           FL_L = "Cl99090j";
           ROS_INFO("cUP %s",FL_L.c_str());
           device.write(FL_L.c_str(),8);
           FL_angle = 90;
       }

             ros::Duration(0.01).sleep();
        if(FR_angle < 90)
       {
            FL_R = "Dr99090j";
            ROS_INFO("cUP %s",FL_R.c_str());
            device.write(FL_R.c_str(),8);
            FR_angle = 90;
       }
       else if((FR_angle-90) > (360-FR_angle)+90 )
       {
           FL_R = "Dr99090j";
           ROS_INFO("cUP %s",FL_R.c_str());
           device.write(FL_R.c_str(),8);
           FR_angle = 90;

       }
       else if((FR_angle)-90 <(360-FR_angle)+90)
       {
           FL_R = "Dl99090j";
           ROS_INFO("cUP %s",FL_R.c_str());
           device.write(FL_R.c_str(),8);
           FR_angle = 90;
       }
          ros::Duration(0.01).sleep();
         if(BL_angle < 45)
        {
             BL_L = "Er99045j";
             ROS_INFO("cUP %s",BL_L.c_str());
             device.write(BL_L.c_str(),8);
             BL_angle = 45;
        }
        else if((BL_angle-45) > (360-BL_angle)+45 )
        {
            BL_L = "Er99045j";
            ROS_INFO("cUP %s",BL_L.c_str());
            device.write(BL_L.c_str(),8);
            BL_angle = 45;

        }
        else if((BL_angle)-45 <(360-BL_angle)+45)
        {
            BL_L = "El99045j";
            ROS_INFO("cUP %s",BL_L.c_str());
            device.write(BL_L.c_str(),8);
            BL_angle = 45;
        }

         ros::Duration(0.01).sleep();

         if(BR_angle < 45)
        {
             BR_R = "Fr99045j";
             ROS_INFO("cUP %s",BR_R.c_str());
             device.write(BR_R.c_str(),8);
             BR_angle = 45;
        }
        else if((BR_angle-45) > (360-BR_angle)+45 )
        {
            BR_R = "Fr99045j";
            ROS_INFO("cUP %s",BR_R.c_str());
            device.write(BR_R.c_str(),8);
            BR_angle = 45;

        }
        else if((BR_angle)-45 <(360-BR_angle)+45)
        {
            BR_R = "Fl99045j";
            ROS_INFO("cUP %s",BR_R.c_str());
            device.write(BR_R.c_str(),8);
            BR_angle = 45;
        }

    }
    else if(msg->data == "up0down45")
    {
        if(FL_angle < 90)
       {
            FL_L = "Cr99090j";
            ROS_INFO("cUP %s",FL_L.c_str());
            device.write(FL_L.c_str(),8);
            FL_angle = 90;
       }
       else if((FL_angle-90) > (360-FL_angle)+90 )
       {
           FL_L = "Cr99090j";
           ROS_INFO("cUP %s",FL_L.c_str());
           device.write(FL_L.c_str(),8);
           FL_angle = 90;

       }
       else if((FL_angle)-90 <(360-FL_angle)+90)
       {
           FL_L = "Cl99090j";
           ROS_INFO("cUP %s",FL_L.c_str());
           device.write(FL_L.c_str(),8);
           FL_angle = 90;
       }

             ros::Duration(0.01).sleep();
        if(FR_angle < 90)
       {
            FL_R = "Dr99090j";
            ROS_INFO("cUP %s",FL_R.c_str());
            device.write(FL_R.c_str(),8);
            FR_angle = 90;
       }
       else if((FR_angle-90) > (360-FR_angle)+90 )
       {
           FL_R = "Dr99090j";
           ROS_INFO("cUP %s",FL_R.c_str());
           device.write(FL_R.c_str(),8);
           FR_angle = 90;

       }
       else if((FR_angle)-90 <(360-FR_angle)+90)
       {
           FL_R = "Dl99090j";
           ROS_INFO("cUP %s",FL_R.c_str());
           device.write(FL_R.c_str(),8);
           FR_angle = 90;
       }
          ros::Duration(0.01).sleep();
         if(BL_angle < 135)
        {
             BL_L = "Er99135j";
             ROS_INFO("cUP %s",BL_L.c_str());
             device.write(BL_L.c_str(),8);
             BL_angle = 135;
        }
        else if((BL_angle-135) > (360-BL_angle)+135 )
        {
            BL_L = "Er99135j";
            ROS_INFO("cUP %s",BL_L.c_str());
            device.write(BL_L.c_str(),8);
            BL_angle = 135;

        }
        else if((BL_angle)-135 <(360-BL_angle)+135)
        {
            BL_L = "El99135j";
            ROS_INFO("cUP %s",BL_L.c_str());
            device.write(BL_L.c_str(),8);
            BL_angle = 135;
        }

         ros::Duration(0.01).sleep();

         if(BR_angle < 135)
        {
             BR_R = "Fr99135j";
             ROS_INFO("cUP %s",BR_R.c_str());
             device.write(BR_R.c_str(),8);
             BR_angle = 135;
        }
        else if((BR_angle-135) > (360-BR_angle)+135 )
        {
            BR_R = "Fr99135j";
            ROS_INFO("cUP %s",BR_R.c_str());
            device.write(BR_R.c_str(),8);
            BR_angle = 135;

        }
        else if((BR_angle)-135 <(360-BR_angle)+135)
        {
            BR_R = "Fl99135j";
            ROS_INFO("cUP %s",BR_R.c_str());
            device.write(BR_R.c_str(),8);
            BR_angle = 135;
        }

    }
    else if(msg->data == "cup")
    {
         if(FL_angle < 45)
        {
             FL_L = "Cr99045j";
             ROS_INFO("cUP %s",FL_L.c_str());
             device.write(FL_L.c_str(),8);
             FL_angle = 45;
        }
        else if((FL_angle-45) > (360-FL_angle)+45 )
        {
            FL_L = "Cr99045j";
            ROS_INFO("cUP %s",FL_L.c_str());
            device.write(FL_L.c_str(),8);
            FL_angle = 45;

        }
        else if((FL_angle)-45 <(360-FL_angle)+45)
        {
            FL_L = "Cl99045j";
            ROS_INFO("cUP %s",FL_L.c_str());
            device.write(FL_L.c_str(),8);
            FL_angle = 45;
        }

         ros::Duration(0.01).sleep();
         if(FR_angle < 50)
        {
             FL_R = "Dr99050j";
             ROS_INFO("cUP %s",FL_R.c_str());
             device.write(FL_R.c_str(),8);
             FR_angle = 50;
        }
        else if((FR_angle-50) > (360-FR_angle)+50 )
        {
            FL_R = "Dr99050j";
            ROS_INFO("cUP %s",FL_R.c_str());
            device.write(FL_R.c_str(),8);
            FR_angle = 45;

        }
        else if((FR_angle)-50 <(360-FR_angle)+50)
        {
            FL_R = "Dl99050j";
            ROS_INFO("cUP %s",FL_R.c_str());
            device.write(FL_R.c_str(),8);
            FR_angle = 50;
        }
         ros::Duration(0.01).sleep();
         if(BL_angle < 45)
        {
             BL_L = "Er99045j";
             ROS_INFO("cUP %s",BL_L.c_str());
             device.write(BL_L.c_str(),8);
             BL_angle = 45;
        }
        else if((BL_angle-45) > (360-BL_angle)+45 )
        {
            BL_L = "Er99045j";
            ROS_INFO("cUP %s",BL_L.c_str());
            device.write(BL_L.c_str(),8);
            BL_angle = 45;

        }
        else if((BL_angle)-45 <(360-BL_angle)+45)
        {
            BL_L = "El99045j";
            ROS_INFO("cUP %s",BL_L.c_str());
            device.write(BL_L.c_str(),8);
            BL_angle = 45;
        }

         ros::Duration(0.01).sleep();

         if(BR_angle < 50)
        {
             BR_R = "Fr99050j";
             ROS_INFO("cUP %s",BR_R.c_str());
             device.write(BR_R.c_str(),8);
             BR_angle = 50;
        }
        else if((BR_angle-50) > (360-BR_angle)+50 )
        {
            BR_R = "Fr99050j";
            ROS_INFO("cUP %s",BR_R.c_str());
            device.write(BR_R.c_str(),8);
            BR_angle = 50;

        }
        else if((BR_angle)-50<(360-BR_angle)+50)
        {
            BR_R = "Fl99050j";
            ROS_INFO("cUP %s",BR_R.c_str());
            device.write(BR_R.c_str(),8);
            BR_angle = 50;
        }

    }
         else if(msg->data == "superman")
         {
             if(FL_angle < 100)
            {
                 FL_L = "Cr99100j";
                 ROS_INFO("cUP %s",FL_L.c_str());
                 device.write(FL_L.c_str(),8);
                 FL_angle = 100;
            }
            else if((FL_angle-100) > (360-FL_angle)+100 )
            {
                FL_L = "Cr99100j";
                ROS_INFO("cUP %s",FL_L.c_str());
                device.write(FL_L.c_str(),8);
                FL_angle = 100;

            }
            else if((FL_angle)-100<(360-FL_angle)+100)
            {
                FL_L = "Cl99100j";
                ROS_INFO("cUP %s",FL_L.c_str());
                device.write(FL_L.c_str(),8);
                FL_angle = 100;
            }

                  ros::Duration(0.01).sleep();
             if(FR_angle < 100)
            {
                 FL_R = "Dr99100j";
                 ROS_INFO("cUP %s",FL_R.c_str());
                 device.write(FL_R.c_str(),8);
                 FR_angle = 100;
            }
            else if((FR_angle-100) > (360-FR_angle)+100 )
            {
                FL_R = "Dr99100j";
                ROS_INFO("cUP %s",FL_R.c_str());
                device.write(FL_R.c_str(),8);
                FR_angle = 100;

            }
            else if((FR_angle)-100 <(360-FR_angle)+100)
            {
                FL_R = "Dl99100j";
                ROS_INFO("cUP %s",FL_R.c_str());
                device.write(FL_R.c_str(),8);
                FR_angle = 100;
            }
               ros::Duration(0.01).sleep();

             if(BL_angle < 110)
            {
                 BL_L = "Er99110j";
                 ROS_INFO("cUP %s",BL_L.c_str());
                 device.write(BL_L.c_str(),8);
                 BL_angle = 110;
            }
            else if((BL_angle-110) > (360-BL_angle)+110)
            {
                BL_L = "Er99110j";
                ROS_INFO("cUP %s",BL_L.c_str());
                device.write(BL_L.c_str(),8);
                BL_angle = 110;

            }
            else if((BL_angle)-100 <(360-BL_angle)+100)
            {
                BL_L = "El99110j";
                ROS_INFO("cUP %s",BL_L.c_str());
                device.write(BL_L.c_str(),8);
                BL_angle = 110;
            }

             ros::Duration(0.01).sleep();

             if(BR_angle <100)
            {
                 BR_R = "Fr99100j";
                 ROS_INFO("cUP %s",BR_R.c_str());
                 device.write(BR_R.c_str(),8);
                 BR_angle = 100;
            }
            else if((BR_angle-100) > (360-BR_angle)+100 )
            {
                BR_R = "Fr99100j";
                ROS_INFO("cUP %s",BR_R.c_str());
                device.write(BR_R.c_str(),8);
                BR_angle = 100;

            }
            else if((BR_angle)-100 <(360-BR_angle)+100)
            {
                BR_R = "Fl99100j";
                ROS_INFO("cUP %s",BR_R.c_str());
                device.write(BR_R.c_str(),8);
                BR_angle = 100;
            }
    }
    else if(msg->data == "up45down45")
    {
        if(FL_angle < 45)
       {
            FL_L = "Cr99045j";
            ROS_INFO("cUP %s",FL_L.c_str());
            device.write(FL_L.c_str(),8);
            FL_angle = 45;
       }
       else if((FL_angle-45) > (360-FL_angle)+45 )
       {
           FL_L = "Cr99045j";
           ROS_INFO("cUP %s",FL_L.c_str());
           device.write(FL_L.c_str(),8);
           FL_angle = 45;

       }
       else if((FL_angle)-45 <(360-FL_angle)+45)
       {
           FL_L = "Cl99045j";
           ROS_INFO("cUP %s",FL_L.c_str());
           device.write(FL_L.c_str(),8);
           FL_angle = 45;
       }

        ros::Duration(0.01).sleep();
        if(FR_angle < 50)
       {
            FL_R = "Dr99050j";
            ROS_INFO("cUP %s",FL_R.c_str());
            device.write(FL_R.c_str(),8);
            FR_angle = 50;
       }
       else if((FR_angle-50) > (360-FR_angle)+50 )
       {
           FL_R = "Dr99050j";
           ROS_INFO("cUP %s",FL_R.c_str());
           device.write(FL_R.c_str(),8);
           FR_angle = 45;

       }
       else if((FR_angle)-50 <(360-FR_angle)+50)
       {
           FL_R = "Dl99050j";
           ROS_INFO("cUP %s",FL_R.c_str());
           device.write(FL_R.c_str(),8);
           FR_angle = 50;
       }

        ros::Duration(0.01).sleep();
        if(BL_angle < 135)
       {
            BL_L = "Er99135j";
            ROS_INFO("cUP %s",BL_L.c_str());
            device.write(BL_L.c_str(),8);
            BL_angle = 135;
       }
       else if((BL_angle-135) > (360-BL_angle)+135 )
       {
           BL_L = "Er99135j";
           ROS_INFO("cUP %s",BL_L.c_str());
           device.write(BL_L.c_str(),8);
           BL_angle = 135;

       }
       else if((BL_angle)-135 <(360-BL_angle)+135)
       {
           BL_L = "El99135j";
           ROS_INFO("cUP %s",BL_L.c_str());
           device.write(BL_L.c_str(),8);
           BL_angle = 135;
       }

          ros::Duration(0.01).sleep();
        if(BR_angle < 135)
       {
            BR_R = "Fr99135j";
            ROS_INFO("cUP %s",BR_R.c_str());
            device.write(BR_R.c_str(),8);
            BR_angle = 135;
       }
       else if((BR_angle-135) > (360-BR_angle)+135 )
       {
           BR_R = "Fr99135j";
           ROS_INFO("cUP %s",BR_R.c_str());
           device.write(BR_R.c_str(),8);
           BR_angle = 135;

       }
       else if((BR_angle)-135 <(360-BR_angle)+135)
       {
           BR_R = "Fl99135j";
           ROS_INFO("cUP %s",BR_R.c_str());
           device.write(BR_R.c_str(),8);
           BR_angle = 135;
       }
    }
    else if(msg->data == "up45down0")
    {
        if(FL_angle < 45)
       {
            FL_L = "Cr99045j";
            ROS_INFO("cUP %s",FL_L.c_str());
            device.write(FL_L.c_str(),8);
            FL_angle = 45;
       }
       else if((FL_angle-45) > (360-FL_angle)+45 )
       {
           FL_L = "Cr99045j";
           ROS_INFO("cUP %s",FL_L.c_str());
           device.write(FL_L.c_str(),8);
           FL_angle = 45;

       }
       else if((FL_angle)-45 <(360-FL_angle)+45)
       {
           FL_L = "Cl99045j";
           ROS_INFO("cUP %s",FL_L.c_str());
           device.write(FL_L.c_str(),8);
           FL_angle = 45;
       }

          ros::Duration(0.01).sleep();
        if(FR_angle < 45)
       {
            FL_R = "Dr99045j";
            ROS_INFO("cUP %s",FL_R.c_str());
            device.write(FL_R.c_str(),8);
            FR_angle = 45;
       }
       else if((FR_angle-45) > (360-FR_angle)+45 )
       {
           FL_R = "Dr99045j";
           ROS_INFO("cUP %s",FL_R.c_str());
           device.write(FL_R.c_str(),8);
           FR_angle = 45;

       }
       else if((FR_angle)-45 <(360-FR_angle)+45)
       {
           FL_R = "Dl99045j";
           ROS_INFO("cUP %s",FL_R.c_str());
           device.write(FL_R.c_str(),8);
           FR_angle = 45;
       }

          ros::Duration(0.01).sleep();
          if(BL_angle < 110)
         {
              BL_L = "Er99110j";
              ROS_INFO("cUP %s",BL_L.c_str());
              device.write(BL_L.c_str(),8);
              BL_angle = 110;
         }
         else if((BL_angle-110) > (360-BL_angle)+110)
         {
             BL_L = "Er99110j";
             ROS_INFO("cUP %s",BL_L.c_str());
             device.write(BL_L.c_str(),8);
             BL_angle = 110;

         }
         else if((BL_angle)-110 <(360-BL_angle)+110)
         {
             BL_L = "El99110j";
             ROS_INFO("cUP %s",BL_L.c_str());
             device.write(BL_L.c_str(),8);
             BL_angle = 110;
         }

          ros::Duration(0.01).sleep();

          if(BR_angle <100)
         {
              BR_R = "Fr99100j";
              ROS_INFO("cUP %s",BR_R.c_str());
              device.write(BR_R.c_str(),8);
              BR_angle = 100;
         }
         else if((BR_angle-100) > (360-BR_angle)+100 )
         {
             BR_R = "Fr99100j";
             ROS_INFO("cUP %s",BR_R.c_str());
             device.write(BR_R.c_str(),8);
             BR_angle = 100;

         }
         else if((BR_angle)-100 <(360-BR_angle)+100)
         {
             BR_R = "Fl99100j";
             ROS_INFO("cUP %s",BR_R.c_str());
             device.write(BR_R.c_str(),8);
             BR_angle = 100;
         }
    }
    else if(msg->data == "laghome")
    {
        if(FL_angle < 0)
       {
            FL_L = "Cr99001j";
            ROS_INFO("cUP %s",FL_L.c_str());
            device.write(FL_L.c_str(),8);
            FL_angle = 1;
       }
       else if((FL_angle-0) > (360-FL_angle)+0 )
       {
           FL_L = "Cr99001j";
           ROS_INFO("cUP %s",FL_L.c_str());
           device.write(FL_L.c_str(),8);
           FL_angle = 1;

       }
       else if((FL_angle)-0 <(360-FL_angle)+0)
       {
           FL_L = "Cl99001j";
           ROS_INFO("cUP %s",FL_L.c_str());
           device.write(FL_L.c_str(),8);
           FL_angle = 1;
       }

          ros::Duration(0.01).sleep();
        if(FR_angle < 0)
       {
            FL_R = "Dr99001j";
            ROS_INFO("cUP %s",FL_R.c_str());
            device.write(FL_R.c_str(),8);
            FR_angle = 1;
       }
       else if((FR_angle-0) > (360-FR_angle)+0 )
       {
           FL_R = "Dr99001j";
           ROS_INFO("cUP %s",FL_R.c_str());
           device.write(FL_R.c_str(),8);
           FR_angle = 1;

       }
       else if((FR_angle)-0 <(360-FR_angle)+0)
       {
           FL_R = "Dl99001j";
           ROS_INFO("cUP %s",FL_R.c_str());
           device.write(FL_R.c_str(),8);
           FR_angle = 1;
       }

          ros::Duration(0.01).sleep();
        if(BL_angle < 0)
       {
            BL_L = "Er99001j";
            ROS_INFO("cUP %s",BL_L.c_str());
            device.write(BL_L.c_str(),8);
            BL_angle = 1;
       }
       else if((BL_angle-0) > (360-BL_angle)+0 )
       {
           BL_L = "Er99001j";
           ROS_INFO("cUP %s",BL_L.c_str());
           device.write(BL_L.c_str(),8);
           BL_angle = 1;

       }
       else if((BL_angle)-0 <(360-BL_angle)+0)
       {
           BL_L = "El99001j";
           ROS_INFO("cUP %s",BL_L.c_str());
           device.write(BL_L.c_str(),8);
           BL_angle = 1;
       }

          ros::Duration(0.01).sleep();
        if(BR_angle < 0)
       {
            BR_R = "Fr99001j";
            ROS_INFO("cUP %s",BR_R.c_str());
            device.write(BR_R.c_str(),8);
            BR_angle = 1;
       }
       else if((BR_angle-0) > (360-BR_angle)+0 )
       {
           BR_R = "Fr99001j";
           ROS_INFO("cUP %s",BR_R.c_str());
           device.write(BR_R.c_str(),8);
           BR_angle = 1;

       }
       else if((BR_angle)-0 <(360-BR_angle)+0)
       {
           BR_R = "Fl99001j";
           ROS_INFO("cUP %s",BR_R.c_str());
           device.write(BR_R.c_str(),8);
           BR_angle = 1;
       }
    }
    //-------------------Manipulator posture------------//
    else if(msg->data == "j1up")
    {
        j1_angle = j1_angle+step_mani;
        if(j1_angle > j1_max)
        {
            j1_angle = j1_max;
        }
        j_1 = "Gl";
        string s = boost::lexical_cast<string>( j1_angle );
        if(j1_angle >=100)
        {
        j_1.append("70");
        j_1.append(s);
        j_1.append("j");
          }
        else if(j1_angle < 100&& j1_angle >= 10 )
        {
            j_1.append("700");
            j_1.append(s);
            j_1.append("j");
        }
        else if(j1_angle < 10)
        {
            j_1.append("7000");
            j_1.append(s);
            j_1.append("j");
        }
        ROS_INFO("%s",j_1.c_str());
        if(j1_angle!=360)
        {
            ros::Duration(0.01).sleep(); device.write(j_1.c_str(),8);

        }
    }
    else if(msg->data == "j1down")
    {
        j1_angle = j1_angle-step_mani;
        if(j1_angle < j1_min)
        {
            j1_angle = j1_min;
        }
        j_1 = "Gl";
        string s = boost::lexical_cast<string>( j1_angle );
        if(j1_angle >=100)
        {
        j_1.append("70");
        j_1.append(s);
        j_1.append("j");
          }
        else if(j1_angle < 100&& j1_angle >= 10 )
        {
            j_1.append("700");
            j_1.append(s);
            j_1.append("j");
        }
        else if(j1_angle < 10)
        {
            j_1.append("7000");
            j_1.append(s);
            j_1.append("j");
        }
        ROS_INFO("%s",j_1.c_str());
        if(j1_angle!=360)
        {
            ros::Duration(0.01).sleep(); device.write(j_1.c_str(),8);

        }
    }
    else if(msg->data == "j2up")
    {
        j2_angle = j2_angle+step_mani;
        if(j2_angle > j2_max)
        {
            j2_angle = j2_max;
        }
        j_2 = "Hl";
        string s = boost::lexical_cast<string>( j2_angle );
        if(j2_angle >=100)
        {
        j_2.append("05");
        j_2.append(s);
        j_2.append("j");
          }
        else if(j2_angle < 100&& j2_angle >= 10 )
        {
            j_2.append("050");
            j_2.append(s);
            j_2.append("j");
        }
        else if(j2_angle < 10)
        {
            j_2.append("0500");
            j_2.append(s);
            j_2.append("j");
        }
        ROS_INFO("%s",j_2.c_str());
        if(j2_angle!=360)
        {
            ros::Duration(0.01).sleep(); device.write(j_2.c_str(),8);

        }
    }
    else if(msg->data == "j2down")
    {
        j2_angle = j2_angle-step_mani;
        if(j2_angle < j2_min)
        {
            j2_angle = j2_min;
        }
        j_2 = "Hl";
        string s = boost::lexical_cast<string>( j2_angle );
        if(j2_angle >=100)
        {
        j_2.append("05");
        j_2.append(s);
        j_2.append("j");
          }
        else if(j2_angle < 100&& j2_angle >= 10 )
        {
            j_2.append("050");
            j_2.append(s);
            j_2.append("j");
        }
        else if(j2_angle < 10)
        {
            j_2.append("0500");
            j_2.append(s);
            j_2.append("j");
        }
        ROS_INFO("%s",j_2.c_str());
        if(j2_angle!=360)
        {
            ros::Duration(0.01).sleep(); device.write(j_2.c_str(),8);

        }
    }
    else if(msg->data == "j3up")
    {
        j3_angle = j3_angle+step_mani;
        if(j3_angle > j3_max)
        {
            j3_angle = j3_max;
        }
        j_3 = "Il";
        string s = boost::lexical_cast<string>( j3_angle );
        if(j3_angle >=100)
        {
        j_3.append("05");
        j_3.append(s);
        j_3.append("j");
          }
        else if(j3_angle < 100&& j3_angle >= 10 )
        {
            j_3.append("050");
            j_3.append(s);
            j_3.append("j");
        }
        else if(j3_angle < 10)
        {
            j_3.append("0500");
            j_3.append(s);
            j_3.append("j");
        }
        ROS_INFO("%s",j_3.c_str());
        if(j3_angle!=360)
        {
            ros::Duration(0.01).sleep(); device.write(j_3.c_str(),8);

        }
    }
    else if(msg->data == "j3down")
    {

        j3_angle = j3_angle-step_mani;
        if(j3_angle < j3_min)
        {
            j3_angle = j3_min;
        }
        j_3 = "Il";
        string s = boost::lexical_cast<string>( j3_angle );
        if(j3_angle >=100)
        {
        j_3.append("05");
        j_3.append(s);
        j_3.append("j");
          }
        else if(j3_angle < 100&& j3_angle >= 10 )
        {
            j_3.append("050");
            j_3.append(s);
            j_3.append("j");
        }
        else if(j3_angle < 10)
        {
            j_3.append("0500");
            j_3.append(s);
            j_3.append("j");
        }
        ROS_INFO("%s",j_3.c_str());
        if(j3_angle!=360)
        {
            ros::Duration(0.01).sleep(); device.write(j_3.c_str(),8);

        }
    }
    else if(msg->data == "j4up")
    {
        j4_angle = j4_angle+step_mani;
        if(j4_angle > j4_max)
        {
            j4_angle = j4_max;
        }
        j_4 = "Jl";
        string s = boost::lexical_cast<string>( j4_angle );
        if(j4_angle >=100)
        {
        j_4.append("05");
        j_4.append(s);
        j_4.append("j");
          }
        else if(j4_angle < 100&& j4_angle >= 10 )
        {
            j_4.append("050");
            j_4.append(s);
            j_4.append("j");
        }
        else if(j4_angle < 10)
        {
            j_4.append("0500");
            j_4.append(s);
            j_4.append("j");
        }
        ROS_INFO("%s",j_4.c_str());
        if(j4_angle!=360)
        {
            ros::Duration(0.01).sleep(); device.write(j_4.c_str(),8);

        }
    }
    else if(msg->data == "j4down")
    {
        j4_angle = j4_angle-step_mani;
        if(j4_angle <j4_min)
        {
            j4_angle = j4_min;
        }
        j_4 = "Jl";
        string s = boost::lexical_cast<string>( j4_angle );
        if(j4_angle >=100)
        {
        j_4.append("05");
        j_4.append(s);
        j_4.append("j");
          }
        else if(j4_angle < 100&& j4_angle >= 10 )
        {
            j_4.append("050");
            j_4.append(s);
            j_4.append("j");
        }
        else if(j4_angle < 10)
        {
            j_4.append("0500");
            j_4.append(s);
            j_4.append("j");
        }
        ROS_INFO("%s",j_4.c_str());
        if(j4_angle!=360)
        {
            ros::Duration(0.01).sleep(); device.write(j_4.c_str(),8);

        }
    }
    else if(msg->data == "j5up")
    {
        j5_angle = j5_angle+step_wrist;
        if(j5_angle > j5_max)
        {
            j5_angle = j5_max;
        }
        j_5 = "Kl";
        string s = boost::lexical_cast<string>( j5_angle );
        if(j5_angle >=100)
        {
        j_5.append("05");
        j_5.append(s);
        j_5.append("j");
          }
        else if(j5_angle < 100&& j5_angle >= 10 )
        {
            j_5.append("050");
            j_5.append(s);
            j_5.append("j");
        }
        else if(j5_angle < 10)
        {
            j_5.append("0500");
            j_5.append(s);
            j_5.append("j");
        }
        ROS_INFO("%s",j_5.c_str());

            ros::Duration(0.03).sleep(); device.write(j_5.c_str(),8); ros::Duration(0.03).sleep();


    }
    else if(msg->data == "j5down")
    {
        j5_angle = j5_angle-step_wrist;
        if(j5_angle < j5_min)
        {
            j5_angle = j5_min;
        }
        j_5 = "Kl";
        string s = boost::lexical_cast<string>( j5_angle );
        if(j5_angle >=100)
        {
        j_5.append("05");
        j_5.append(s);
        j_5.append("j");
          }
        else if(j5_angle < 100&& j5_angle >= 10 )
        {
            j_5.append("050");
            j_5.append(s);
            j_5.append("j");
        }
        else if(j5_angle < 10)
        {
            j_5.append("0500");
            j_5.append(s);
            j_5.append("j");
        }
        ROS_INFO("%s",j_5.c_str());

           ros::Duration(0.03).sleep(); device.write(j_5.c_str(),8); ros::Duration(0.03).sleep();


    }
    else if(msg->data == "j6down")
    {
        j6_angle = j6_angle+step_wrist;
        if(j6_angle > j6_max)
        {
            j6_angle = j6_max;
        }
        j_6 = "Ll";
        string s = boost::lexical_cast<string>( j6_angle );
        if(j6_angle >=100)
        {
        j_6.append("70");
        j_6.append(s);
        j_6.append("j");
          }
        else if(j6_angle < 100&& j6_angle >= 10 )
        {
            j_6.append("700");
            j_6.append(s);
            j_6.append("j");
        }
        else if(j6_angle < 10)
        {
            j_6.append("7000");
            j_6.append(s);
            j_6.append("j");
        }
        ROS_INFO("%s",j_6.c_str());

            ros::Duration(0.03).sleep(); device.write(j_6.c_str(),8); ros::Duration(0.03).sleep();


    }
    else if(msg->data == "j6up")
    {
        j6_angle = j6_angle-step_wrist;
        if(j6_angle < j6_min)
        {
            j6_angle = j6_min;
        }
        j_6 = "Ll";
        string s = boost::lexical_cast<string>( j6_angle );
        if(j6_angle >=100)
        {
        j_6.append("70");
        j_6.append(s);
        j_6.append("j");
          }
        else if(j6_angle < 100&& j6_angle >= 10 )
        {
            j_6.append("700");
            j_6.append(s);
            j_6.append("j");
        }
        else if(j6_angle < 10)
        {
            j_6.append("7000");
            j_6.append(s);
            j_6.append("j");
        }
        ROS_INFO("%s",j_6.c_str());

            ros::Duration(0.03).sleep(); device.write(j_6.c_str(),8); ros::Duration(0.03).sleep();


    }
    else if(msg->data == "j7up")
    {
        j7_angle = j7_angle+step_wrist;
        if(j7_angle > 180)
        {
            j7_angle = 180;
        }
        j_7 = "Ml";
        string s = boost::lexical_cast<string>( j7_angle );
        if(j7_angle >=100)
        {
        j_7.append("50");
        j_7.append(s);
        j_7.append("j");
          }
        else if(j7_angle < 100&& j7_angle >= 10 )
        {
            j_7.append("500");
            j_7.append(s);
            j_7.append("j");
        }
        else if(j7_angle < 10)
        {
            j_7.append("5000");
            j_7.append(s);
            j_7.append("j");
        }
        ROS_INFO("%s",j_7.c_str());

            ros::Duration(0.01).sleep(); device.write(j_7.c_str(),8);


    }
    else if(msg->data == "j7down")
    {
        j7_angle = j7_angle-step_wrist;
        if(j7_angle < 0)
        {
            j7_angle = 0;
        }
        j_7 = "Ml";
        string s = boost::lexical_cast<string>( j7_angle );
        if(j7_angle >=100)
        {
        j_7.append("99");
        j_7.append(s);
        j_7.append("j");
          }
        else if(j7_angle < 100&& j7_angle >= 10 )
        {
            j_7.append("990");
            j_7.append(s);
            j_7.append("j");
        }
        else if(j7_angle < 10)
        {
            j_7.append("9900");
            j_7.append(s);
            j_7.append("j");
        }
        ROS_INFO("%s",j_7.c_str());

            ros::Duration(0.01).sleep(); device.write(j_7.c_str(),8);


    }
    //----------------Special Manipulator posture -------------//
    else if(msg->data == "manireset")
    {
        mani_pos(90,0,5,0);
    }
    else if(msg->data == "manimiddle")
    {
        mani_pos(90,45,90,0);
    }
    else if(msg->data == "manihigh")
    {
        mani_pos(90,60,130,0);
    }
    else if(msg->data == "manidoor")
    {
        mani_pos(110,70,120,0);
    }
    else if(msg->data == "maniup")
    {
        mani_pos(90,85,120,0);
    }
    else if(msg->data == "camreset")
    {
        j_5 =  "Kl99512j";
        ros::Duration(0.01).sleep(); device.write(j_5.c_str(),8);
        j_6 =  "Ll10415j";
        ros::Duration(0.01).sleep(); device.write(j_6.c_str(),8);
        j_6 =  "Ll10315j";
        ros::Duration(0.01).sleep(); device.write(j_6.c_str(),8);
        j5_angle = 512;
        j6_angle = 315;
    }
    else

    {
       device.write(msg->data.c_str());
    }

}

//------------------subscribe value to this node------------//
void SpeedCallback(const std_msgs::Int16ConstPtr& msg)
{

speed = msg->data;
}
void angle_callback(const std_msgs::Float32MultiArrayConstPtr& msg)
{
    int a1 = (int)msg->data[0];
    int a2 = (int)msg->data[1];
    int a3 = (int)msg->data[2];
    int a4 = (int)msg->data[3];
    //int a5 = (int)msg->data[4];
    //int a6 = (int)msg->data[5];
    mani_pos(a1,a2,a3,a4);
}
 /*void update_callback(const ros::TimerEvent& event)
{
    try         // if in "try" can do then do, but if can't do go to "catch"
    {
    device.write("?Aj");
    ros::Duration(0.01).sleep();
      int result_io =  device.read(reply,18,1000);   // buffer,length,timeout
  //    ROS_INFO("%d",result_io);         // chk number of msgs
    //  ROS_INFO("%s",reply);
       en_FL = strtod(reply,&A);        // 'strtod' change str to double
       en_FR = strtod(A,&A);
       en_BL = strtod(A,&A);
       en_BR = strtod(A,NULL);

          FL_angle = en_FL;
          FR_angle = en_FR;
           BR_angle = en_BR;
             BL_angle = en_BL;


     device.write("?Mj");
      ros::Duration(0.01).sleep();
         result_io =  device.read(Mreply,18,1000);   // buffer,length,timeout
         //   ROS_INFO("%d",result_io);         // chk number of msgs
          //  ROS_INFO("%s",Mreply);
            en_j1 = strtod(Mreply,&J);        // 'strtod' change str to double
            en_j2 = strtod(J,&J);
            en_j3 = strtod(J,&J);
            en_j4 = strtod(J,NULL);
            pos.push_back(en_FL);            // put the read angle to pos vector
            pos.push_back(en_FR);
            pos.push_back(en_BL);
            pos.push_back(en_BR);
            pos.push_back(en_j1);
            pos.push_back(en_j2);
            pos.push_back(en_j3);
            pos.push_back(en_j4);
         pos.resize(8);
            fliper_pos.data = pos;           // change to ros std_msgs
            fliper_pos.data.resize(8);
            Leg_pos.publish(fliper_pos);     // publish angle
           ROS_INFO("FL %d, FR %d, BL %d, BR %d, J1 %d, J2 %d, J3 %d, J4 %d",en_FL,en_FR,en_BL,en_BR,en_j1,en_j2,en_j3,en_j4);
      }
       catch(cereal::Exception& e)
       {
        ROS_ERROR("Control Timeout");
       }
}
*/
void legcallback(const ros::TimerEvent& event)  // timer of this node for read leg pos
{
    try         // if in "try" can do then do, but if can't do go to "catch"
    {
    device.write("?Aj");
    ros::Duration(0.01).sleep();
      int result_io =  device.read(reply,18,1000);   // buffer,length,timeout
  //    ROS_INFO("%d",result_io);         // chk number of msgs
      ROS_INFO("%s",reply);
       en_FL = strtod(reply,&A);        // 'strtod' change str to double
       en_FR = strtod(A,&A);
       en_BL = strtod(A,&A);
       en_BR = strtod(A,NULL);


    //       FL_angle = en_FL;
      //     FR_angle = en_FR;
        //   BR_angle = en_BR;
          // BL_angle = en_BL;

      device.write("?Mj");
      ros::Duration(0.01).sleep();
         result_io =  device.read(Mreply,18,1000);   // buffer,length,timeout
         //   ROS_INFO("%d",result_io);         // chk number of msgs
          //  ROS_INFO("%s",Mreply);
            en_j1 = strtod(Mreply,&J);        // 'strtod' change str to double
            en_j2 = strtod(J,&J);
            en_j3 = strtod(J,&J);
            en_j4 = strtod(J,NULL);
            en_j5 = j5_angle*0.29;
            en_j6 = j6_angle*0.29;
            pos.push_back(en_FL);            // put the read angle to pos vector
            pos.push_back(en_FR);
            pos.push_back(en_BL);
            pos.push_back(en_BR);
            pos.push_back(en_j1);
            pos.push_back(en_j2);
            pos.push_back(en_j3);
            pos.push_back(en_j4);
            pos.push_back(en_j5);
            pos.push_back(en_j6);
            pos.resize(10);
            fliper_pos.data = pos;           // change to ros std_msgs
            fliper_pos.data.resize(10);
            Leg_pos.publish(fliper_pos);     // publish angle
            ROS_INFO(":FL %d, FR %d, BL %d, BR %d, J1 %d, J2 %d, J3 %d, J4 %d, J5 %d, J6 %d",fliper_pos.data[0],fliper_pos.data[1],fliper_pos.data[2],fliper_pos.data[3],fliper_pos.data[4],fliper_pos.data[5],fliper_pos.data[6],fliper_pos.data[7],fliper_pos.data[8],fliper_pos.data[9]);
            fliper_pos.data.capacity();
            pos.clear();
      }
       catch(cereal::Exception& e)
       {
        ROS_ERROR("Control Timeout");
       }
}
//-------------------------------Main --------------//
int main(int argc, char** argv)
{
    ros::init(argc, argv, "dsPIC_Node");            // start ros node name"dsPIC_Node"
    ros::NodeHandle n;                              // handle the information
    ros::Subscriber sub = n.subscribe("teleop_control", 1, teleop_callback);    //subscribe cmd from robot control
    ros::Subscriber sub2 = n.subscribe("speed", 1, SpeedCallback);              //sbscribe spd from robot control
    ros::Subscriber sub3 = n.subscribe("angle",10,angle_callback);              //subscribe angle from robot control
 Leg_pos = n.advertise<std_msgs::Int16MultiArray>("leg_pos",100);               //advertised leg pos to publish
 ros::Timer timer = n.createTimer(ros::Duration(0.3), legcallback);             //define timer function
// ros::Timer en_update = n.createTimer(ros::Duration(20),update_callback);
 std::string str = "/dev/ttyUSB0";
 n.getParam("device", str);
    //char reply[REPLY_SIZE];

    // Change the next line according to your port name and baud rate
    try
    {
      // ROS_INFO("Try to open port %s.",str.c_str());
     device.open("/dev/ttyUSB0", 115200);
     device.write("?Aj");
     ros::Duration(0.01).sleep();
       int result_io =  device.read(reply,18,1000);   // buffer,length,timeout
       //    ROS_INFO("%d",result_io);         // chk number of msgs
         //  ROS_INFO("%s",reply);
            en_FL = strtod(reply,&A);        // 'strtod' change str to double
            en_FR = strtod(A,&A);
            en_BL = strtod(A,&A);
            en_BR = strtod(A,NULL);
            FL_angle = en_FL;
        FR_angle = en_FR;
        BL_angle = en_BL;
        BR_angle = en_BR;
        device.write("?Mj");
        ros::Duration(0.01).sleep();
           result_io =  device.read(Mreply,18,1000);   // buffer,length,timeout
      //    ROS_INFO("%d",result_io);         // chk number of msgs
        //  ROS_INFO("%s",reply);
           en_j1 = strtod(Mreply,&J);        // 'strtod' change str to double
           en_j2 = strtod(J,&J);
           en_j3 = strtod(J,&J);
           en_j4 = strtod(J,NULL);
           j1_angle = en_j1;
           j2_angle = en_j2;
           j3_angle = en_j3;
           j4_angle = en_j4;
    }

    catch(cereal::Exception& e)
    {
        ROS_FATAL("Failed to open the serial port!!!");
        ROS_BREAK();
    }

    ROS_INFO("The serial port is opened.");

  /*  ros::Rate r(25);



    //we will be sending commands of type "twist"
    //geometry_msgs::Twist base_cmd;
   // geometry_msgs::Twist cmdl;
 vector<short int> pos;

    while(ros::ok())
    {

        //try
        //{
        device.write("?Aj");
        ros::Duration(0.01).sleep();
          int result_io =  device.read(reply,18,1000);   // buffer,length,timeout
          //ROS_INFO("%d",result_io);
            //ROS_INFO("%s",reply);
           en_FL = strtod(reply,&A);
           en_FR = strtod(A,&A);
           en_BL = strtod(A,&A);
           en_BR = strtod(A,NULL);


           pos.push_back(en_FL);
           pos.push_back(en_FR);
           pos.push_back(en_BL);
           pos.push_back(en_BR);

           fliper_pos.data = pos;
           Leg_pos.publish(fliper_pos);
          // ROS_INFO("FL %d, FR %d, BL %d, BR %d",en_FL,en_FR,en_BL,en_BR);

          //}
           catch(cereal::Exception& e)
           {
            ROS_ERROR("Control Timeout");
           }



            ros::spinOnce();
            r.sleep();
    }*/
    ros::spin();
    return 0;
}
