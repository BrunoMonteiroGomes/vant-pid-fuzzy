    #include "ros/ros.h"
    #include "geometry_msgs/Twist.h"
    #include "std_msgs/String.h"     
    #include "geometry_msgs/Pose.h"
    #include <iostream>
    #include <math.h>   
    #include <tf/tf.h>
    #include <tf/transform_listener.h>
    #include "geometry_msgs/Point.h"
         
    using namespace std;
    
    struct Point {
		float x;
		float y;
	};
    
    geometry_msgs::Pose setpoint;    
    geometry_msgs::Pose pose; 	
      
    Point my_array[11];
	
	 
    int main(int argc, char **argv)
    {
				
	  ros::init(argc, argv, "vant_pid_fuzzy_comandos");
     
      ros::NodeHandle n; 
      tf::TransformListener listener;     
           
      ros::Publisher pub = n.advertise<geometry_msgs::Pose>("vant/setpoint", 1000);      
           
      ros::Rate loop_rate(10);    
      
      Point my_array[11];
      Point point;
      double erroDistancia = 10;
      
      point.x = 9;
	  point.y = 1;
      my_array[0] = point;
      
      point.x = 9;
	  point.y = 9;
      my_array[1] = point;
      
      point.x = 1;
	  point.y = 9;
      my_array[2] = point;		
			
	  point.x = 1;
	  point.y = 1;
      my_array[3] = point;		
			
	  point.x = 8;
	  point.y = 2;
      my_array[4] = point;	
		
	  point.x = 8;
	  point.y = 8;
      my_array[5] = point;
      
      point.x = 2;
	  point.y = 8;
      my_array[6] = point;
			
	  point.x = 2;
	  point.y = 2;
      my_array[7] = point;		
			
	  point.x = 7;
	  point.y = 3;
      my_array[8] = point;		
			
	  point.x = 7;
	  point.y = 7;
      my_array[9] = point;		
			
	  point.x = 3;
	  point.y = 7;
      my_array[10] = point;		
			
	  point.x = 3;
	  point.y = 3;
      my_array[11] = point;		
      
      point.x = 6;
	  point.y = 4;
      my_array[12] = point;
			
	  point.x = 6;
	  point.y = 6;
      my_array[13] = point;		
			
	  point.x = 4;
	  point.y = 6;
      my_array[14] = point;		
			
	  point.x = 4;
	  point.y = 4;
      my_array[15] = point;	
      	
	  point.x = 5;
	  point.y = 5;
      my_array[16] = point;	
			
	  point.x = 5;
	  point.y = 0.76;
      my_array[17] = point;				
      
     
      if (ros::ok())
      {
                 
        ros::spinOnce();
                    
        ROS_INFO("INICIANDO SIMULACAO...");        	
    	    	 	
    	ros::spinOnce();	
    	
    	tf::StampedTransform transform;
    	
        int i = 0;
        
        setpoint.position.x = my_array[i].x;
	    setpoint.position.y = my_array[i].y;
    		
    	while (1) {		
			
				
			try {
					listener.waitForTransform("map", "base_link", ros::Time(0), ros::Duration(2.0));
					listener.lookupTransform("map", "base_link", ros::Time(0), transform);
					
					pose.position.x = transform.getOrigin().x(); 
					pose.position.y = transform.getOrigin().y();
					pose.position.z = 0;			
					
				} catch (tf::TransformException ex){
					ROS_ERROR("%s",ex.what());
					ros::Duration(1.0).sleep();
				} 	
				
			  // if (i == 0) {
			//	   ROS_INFO("Indo para o ponto: [%d] ", i + 1);
			//   }	
				
						   
			   pub.publish(setpoint);
			   erroDistancia = abs(sqrt(pow(setpoint.position.x - pose.position.x, 2) + pow(setpoint.position.y - pose.position.y, 2)));
			   
			   if (erroDistancia < 0.2) {
				   i++;
				 //  ROS_INFO("Indo para o ponto: [%d] ", i + 1);
				   setpoint.position.x = my_array[i].x;
				   setpoint.position.y = my_array[i].y;				   
			   } 
			   
			   if (i == 18 && erroDistancia < 0.2) {
				  ROS_INFO("FIM DA SIMULACAO..."); 
				  ros::shutdown();
			   }
			   
			   ros::spinOnce();			
			   loop_rate.sleep();
	       
		}    	
    	
      }
     
      return 0;
    }

