
#if (ARDUINO >= 100)
 #include <Arduino.h>
#else
 #include <WProgram.h>
#endif
#include <ros.h>
#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/Twist.h>
#include <ros/time.h>

#include "robot_specs.h"



#define LeftMotorA  5
#define LeftMotorB  9
#define RightMotorA  10
#define RightMotorB  11

#define LOOPTIME        100   // PID loop time(ms)
#define SMOOTH      10

#define sign(x) (x > 0) - (x < 0)

#define FORWARD 1
#define BACKWARD 2
#define STOP 0

unsigned long lastMilli = 0;       // loop timing 
unsigned long lastMilliPub = 0;

long rpm_req1 = 0;
long rpm_req2 = 0;

long long rpm_act1 = 0;
long long rpm_act2 = 0;

long long t1_l=0;




double rpm_req1_smoothed = 0;
double rpm_req2_smoothed = 0;

int directionLeft = FORWARD;
int directionRight = FORWARD;



 int PWM_val1 = 0;
 int PWM_val2 = 0;

volatile long count1 = 0;          // rev counter
volatile long count2 = 0;

long countAnt1 = 0;
long countAnt2 = 0;

float Kp =   0.5;
float Kd =   0;
float Ki =   0;



int flagL=0 , flagR=0 , fl , fr;


ros::NodeHandle nh;

void handle_cmd( const geometry_msgs::Twist& cmd_msg)
{
  double x = cmd_msg.linear.x;
  double z = cmd_msg.angular.z;
  if (z == 0) 
  {     // go straight
    // convert m/s to rpm
    rpm_req1 = x*60/(pi*wheel_diameter);
    rpm_req2 = rpm_req1;
  }
  else if (x == 0) 
  {
    // convert rad/s to rpm
    rpm_req2 = z*track_width*60/(wheel_diameter*pi*2);
    rpm_req1 = -rpm_req2;
  }
  else
  {
    rpm_req1 = x*60/(pi*wheel_diameter)-z*track_width*60/(wheel_diameter*pi*2);
    rpm_req2 = x*60/(pi*wheel_diameter)+z*track_width*60/(wheel_diameter*pi*2);
  }
}



ros::Subscriber<geometry_msgs::Twist> sub("cmd_vel_mux/input/teleop", handle_cmd);

geometry_msgs::Vector3Stamped rpm_msg;
ros::Publisher rpm_pub("rpm", &rpm_msg);

ros::Time current_time;
ros::Time last_time;



char c;

void setup() 
{
     Serial1.begin(9600);
     
     count1 = 0;
     count2 = 0;
     
     countAnt1 = 0;
     countAnt2 = 0;
     
     rpm_req1 = 0;
     rpm_req2 = 0;
     
     rpm_act1 = 0;
     rpm_act2 = 0;
     
     PWM_val1 = 0;
     PWM_val2 = 0;
    
     
     nh.initNode();
    nh.getHardware()->setBaud(57600);
    nh.subscribe(sub);
     nh.advertise(rpm_pub);
     
      
     pinMode(A5,INPUT);
     pinMode(A4,INPUT);
     
    
    
    pinMode(LeftMotorA , OUTPUT);
    pinMode(LeftMotorB , OUTPUT);
    
    pinMode(RightMotorA , OUTPUT);
    pinMode(RightMotorB , OUTPUT);
    
    digitalWrite(LeftMotorA , LOW);
    digitalWrite(LeftMotorB , LOW);
    digitalWrite(RightMotorA , LOW);
    digitalWrite(RightMotorB , LOW);
    
    
    
   
}

void loop() 
{

    if(Serial1.available())
    {
       c = Serial1.read();
       
       switch(c)
    {
      
      
      case 'f' :
                PWM_val1 = 150;
                PWM_val2 = 150;
               motorRun(1,FORWARD,PWM_val1);
               motorRun(2,FORWARD,PWM_val2);
               break;
    
      case 'b' : motorRun(1,BACKWARD,180);
               motorRun(2,BACKWARD,180);
               break;
    
      case 'l' : motorRun(1,STOP,0);
               motorRun(2,FORWARD,180);
               break;
    
      case 'r' : motorRun(1,FORWARD,180);
               motorRun(2,STOP,0);
               break;
    
       case 's' : 
                PWM_val1 = 0;
                PWM_val2 = 0;
               motorRun(1,STOP,0);
               motorRun(2,STOP,0);
               break;
    
    
      default : PWM_val1 = 0;
                PWM_val2 = 0;
              
               break;        
    }
    
    }





   fl = analogRead(A5);
  if((fl>400) && flagL==0)
    {
      //Serial1.print("L");
      flagL=1;
      encoderLeft();
    }
    
    else if(fl<100)
    flagL=0;

     fr=analogRead(A4);
    if((fr>400) && flagR==0)
    {
      //Serial1.print("R");
      flagR=1;
      encoderRight();
    }
    else if(fr<100)
    flagR=0;


    

  
 
 
   unsigned long time = millis();
  if(time-lastMilli>= 500) 
  {      // enter tmed loop
    getMotorData(time-lastMilli);



     Serial1.print("RPM of Left motor :");
      Serial1.print(long(rpm_act1));
      Serial1.print("\tRPM of Right motor :");
      Serial1.println(long(rpm_act2));
      
      
      Serial1.print("rpm_req1: ");
   Serial1.print(rpm_req1);
   Serial1.print("\trpm_req2: ");
   Serial1.println(rpm_req2);

   PWM_val1 = abs(updatePid(1, PWM_val1, rpm_req1, rpm_act1));
   PWM_val2 = abs(updatePid(2, PWM_val2, rpm_req2, rpm_act2));


 //PWM_val1 = 150.0*rpm_req1/MAX_RPM;
 //PWM_val2 = 150.0*rpm_req2/MAX_RPM;

   


   Serial1.print("PWM1 : ");
   Serial1.print(PWM_val1);
   Serial1.print("\tPWM2 : ");
   Serial1.println(PWM_val2);
   Serial1.println();

   
   // if(PWM_val1 > 0) directionLeft = FORWARD;
    //else if(PWM_val1 < 0) directionLeft = BACKWARD;
    //if (rpm_req1 == 0) directionLeft = STOP;
    //if(PWM_val2 > 0) directionRight = FORWARD;
    //else if(PWM_val2 < 0) directionRight = BACKWARD;
    //if (rpm_req2 == 0) directionRight = STOP;

   
    directionLeft = FORWARD;
    directionRight = FORWARD;
    motorRun(1,directionLeft,PWM_val1);
    motorRun(2,directionRight,PWM_val2);



      
    publishRPM(time-lastMilli);
    
    lastMilli = time;
  }
    
    
    nh.spinOnce();
}


void getMotorData(unsigned long time)  
{
  unsigned long dt = time*8;
  long long  dcount1 = count1-countAnt1;
  long long  dcount2 = count2-countAnt2;
  rpm_act1 = (dcount1*60000)/dt;
  rpm_act2 = (dcount2*60000)/dt;
 
  countAnt1 = count1;
  countAnt2 = count2;
}


int updatePid(int id, int command, double targetValue, double currentValue) {
  double pidTerm = 0;                            // PID correction
  double error = 0;
  double new_pwm = 0;
  double new_cmd = 0;
  static double last_error1 = 0;
  static double last_error2 = 0;
  static double int_error1 = 0;
  static double int_error2 = 0;
  
  error = targetValue-currentValue;
  if (id == 1) {
    int_error1 += error;
    pidTerm = Kp*error + Kd*(error-last_error1) + Ki*int_error1;
    last_error1 = error;
  }
  else {
    int_error2 += error;
    pidTerm = Kp*error + Kd*(error-last_error2) + Ki*int_error2;
    last_error2 = error;
  }
  new_pwm = constrain(abs(command)*MAX_RPM/150.0 + pidTerm, -MAX_RPM, MAX_RPM);
  
   //Serial1.print("new_pwm: ");
   //Serial1.println(new_pwm);
  

  
  
  new_cmd = 150.0*new_pwm/MAX_RPM;
  return int(new_cmd);
}




void motorRun(int Id , int Direction , int Speed)
{
  switch(Id)
  {
    // left motor
    case 1 : if(Direction == FORWARD)
               {
               analogWrite(LeftMotorA , Speed);
               analogWrite(LeftMotorB , 0);
               break;
               }
              
              else if(Direction == BACKWARD)
               {
               analogWrite(LeftMotorA , 0);
               analogWrite(LeftMotorB , Speed);
               break;
               } 

               else if(Direction == STOP)
               {
               digitalWrite(LeftMotorA , HIGH);
               digitalWrite(LeftMotorB , HIGH);
               break;
               }  


                // Right motor
    case 2 : if(Direction == FORWARD)
               {
               analogWrite(RightMotorA , Speed);
               analogWrite(RightMotorB , 0);
               break;
               }
              
              else if(Direction == BACKWARD)
                {
               analogWrite(RightMotorA , 0);
               analogWrite(RightMotorB , Speed);
               break;
               }  

               else if(Direction == STOP)
                {
               digitalWrite(RightMotorA , HIGH);
               digitalWrite(RightMotorB , HIGH);
               break;
               } 

     default : break;
  }
}


void publishRPM(unsigned long time) {
  rpm_msg.header.stamp = nh.now();
  rpm_msg.vector.x = rpm_act1;
  rpm_msg.vector.y = rpm_act2;
  rpm_msg.vector.z = double(time)/1000;
  rpm_pub.publish(&rpm_msg);
  nh.spinOnce();
}


void encoderLeft() 
{
count1++;
}




void encoderRight() 
{
 count2++;
}
