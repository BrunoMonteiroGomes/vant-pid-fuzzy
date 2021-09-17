    #include <stdlib.h>
    #include "ros/ros.h"
    #include "geometry_msgs/Twist.h"
    #include "std_msgs/String.h" 
    #include "nav_msgs/Odometry.h" 
    #include "sensor_msgs/LaserScan.h"
    #include "geometry_msgs/Pose.h"
    #include "geometry_msgs/Vector3.h"
    #include <iostream>
    #include <math.h>
    #include <tf/tf.h>
	#include <tf/transform_listener.h>
		         
    using namespace std;
    
    #include "FLIE-master/flie.h"
    
    typedef struct { 
		float erro;
		float kp, kd, ki;
		float integral, erroAnterior;
		float saida;
		float dt;
		
	} pidType;
	
	pidType pidDistancia, pidAngulo;
	
	void calculaPID (pidType * pid) {
		
		float derivativo = (pid->erro - pid->erroAnterior)/ pid->dt;
		pid->integral += pid->erro * pid->dt;
		
		pid->saida = pid->erro       * pid->kp 
		           + pid->integral   * pid->ki 
		           + derivativo      * pid->kd;
		      
		pid->erroAnterior = pid->erro;		
	}	
	
	double yaw_angle, x_current, y_current, theta_current;
	
	double frente, direita, esquerda;
	bool ori_ok = false, pos_ok = false;
	float posdesejada[2], oridesejada, erropos=99, erroorie=99, erropos_1, erropos_2;
	float tolerance_orie = 0.05, tolerance_pos = 0.1;
	bool is_frente = false;
	bool is_esquerda = false;
	bool is_direita = false;
	double range_desvio_far = 1.2; //1
	double range_desvio_close = 0.8; //0.7
	double range_desvio_very_close = 0.5; //0.4
	double range_desvio = range_desvio_far;
	double lidar_size, lidar_size_received;
	double lidar_ignorar_percentage = 0.15; //0.3
	double lidar_laterais_percentage = 0.95;//0.95
	double count_lados;
	double count_frente;
	double count_ign;
	double max_lin_free = 1.2;
	double max_lin_obs = 0.6;
	double seg_esq, seg_frente, seg_dir, seg_ign, seg_ign_esq, seg_ign_dir;
	int size_1, size_2,	size_3,	size_4,	size_5, total;
	int i,j,k;
	
	nav_msgs::Odometry odom;
	
	geometry_msgs::Pose pose; 
	geometry_msgs::Pose setpoint; 
	
    tf::Pose tf_pose;
      	
	void subCallbackSetpoint(const geometry_msgs::Pose::ConstPtr& msg) { 
		setpoint.position.x = msg->position.x; 
		setpoint.position.y = msg->position.y; 
		setpoint.position.z = msg->position.z; 
	}		
	
	void chatterCallback(const nav_msgs::Odometry::ConstPtr& msg) {
		
		x_current = msg->pose.pose.position.z;
		y_current = msg->pose.pose.position.y;

		tf::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y, 	
		                msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
		                
		tf::Matrix3x3 m(q);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		theta_current = yaw + M_PI/2; 		
    }
    
    void subCallback_lidar(const sensor_msgs::LaserScan::ConstPtr& lidar) {	
		
		lidar_size_received = lidar->ranges.size();
		lidar_size = floor(lidar_size_received*(1-lidar_ignorar_percentage));

		size_1 = floor((lidar_size_received - lidar_size)/2);
		size_2 = floor(lidar_size * lidar_laterais_percentage)/2;
		size_4 = size_2;
		size_5 = size_1;
		size_3 = lidar_size_received - size_1 - size_2 - size_4 - size_5;

		total = size_1 + size_2 + size_3 + size_4 + size_5;

		direita = 1.5; //1.5
		for (i=size_1;i<(size_1 + size_2 + size_3 + size_4);i++){
			if(lidar->ranges[i] < direita && lidar->ranges[i] > 0.01){
				direita = lidar->ranges[i];
			}
		}
		
		frente = 1.5; //1.5
		for (i=i;i<(size_1 + size_2 + size_3);i++){
			if(lidar->ranges[i] < frente && lidar->ranges[i] > 0.01){
				frente = lidar->ranges[i];
			}
		}

		esquerda = 1.5; //1.5
		for (i=i;i<(size_1 + size_2 + size_3 + size_4);i++){
			if(lidar->ranges[i] < esquerda && lidar->ranges[i] > 0.01){
				esquerda = lidar->ranges[i];
			}
		}
		
		range_desvio = erropos < range_desvio_close ? range_desvio_close : range_desvio_far;
		range_desvio = erropos < range_desvio_very_close ? range_desvio_very_close : range_desvio;

		//Verifica os objetos próximos e, se dentro do range escolhido, aciona a flag de desvio.
		is_esquerda = esquerda < range_desvio ? true : false;
		is_frente = frente < range_desvio * 0.8? true : false; //1.2
		is_direita = direita < range_desvio ? true : false; //
	}
	
	 
    int main(int argc, char **argv)
    {
	
	  ros::init(argc, argv, "vant_pid_fuzzy");
	  ros::NodeHandle n; 
	  tf::TransformListener listener;
	  	
	  float angulo, erroAngulo;
	  float erroDistancia;
	  
	  //Setup PID
	  
	  pidAngulo.erro         = 0;
	  pidAngulo.dt           = 0;
	  pidAngulo.erroAnterior = 0;
	  pidAngulo.integral     = 0;
	  	  
	  pidDistancia.erro         = 0;
	  pidDistancia.dt           = 0;
	  pidDistancia.erroAnterior = 0;
	  pidDistancia.integral     = 0;
	  
	  pidAngulo.kp = 1.2;   
	  pidAngulo.kd = 1.5; 
	  pidAngulo.ki = 0; 
	  
	  pidDistancia.kp =  0.2; 
	  pidDistancia.kd = 0.02;
	  pidDistancia.ki = 0;	  
	  
	
	  ros::Time tBegin = ros::Time::now();
	  ros::Time tEnd = ros::Time::now();	         
           
      ros::Publisher pub = n.advertise<geometry_msgs::Twist>("/cmd_vel", 1000);
      ros::Subscriber subsetpoint = n.subscribe("vant/setpoint", 1000, subCallbackSetpoint);
      ros::Subscriber sub = n.subscribe("/odom", 1000, chatterCallback);
      ros::Subscriber sub_lidar = n.subscribe("scan", 1000, subCallback_lidar);
     
      ros::Rate loop_rate(10);  
      
//****************************************************************//
//		      CONTROLADOR FUZZY PARA DESVIO DE OBSTÁCULO	      //
//****************************************************************//
      
      /*** Proximidade do obstáculo - begin ***/

	  fuzzy_set cat_obstaculo[3];

	  cat_obstaculo[0].setname("VERYCLOSE");
	  cat_obstaculo[0].setrange(0, 2);
	  cat_obstaculo[0].setval(0, 0, 0.4, 0.6);   

	  cat_obstaculo[1].setname("CLOSE");
      cat_obstaculo[1].setrange(0, 2);
	  cat_obstaculo[1].setval(0.5, 0.7, 0.9); 

	  cat_obstaculo[2].setname("FAR");
	  cat_obstaculo[2].setrange(0, 2);
	  cat_obstaculo[2].setval(0.8, 1, 2, 2); 

	  linguisticvariable obstaculo_esquerda;
	  obstaculo_esquerda.setname("obstaculo_esquerda");
	  obstaculo_esquerda.includecategory(&cat_obstaculo[0]);
	  obstaculo_esquerda.includecategory(&cat_obstaculo[1]);
	  obstaculo_esquerda.includecategory(&cat_obstaculo[2]);

	  linguisticvariable obstaculo_frente;
	  obstaculo_frente.setname("obstaculo_frente");
	  obstaculo_frente.includecategory(&cat_obstaculo[0]);
	  obstaculo_frente.includecategory(&cat_obstaculo[1]);
	  obstaculo_frente.includecategory(&cat_obstaculo[2]);

	  linguisticvariable obstaculo_direita;
	  obstaculo_direita.setname("obstaculo_direita");
	  obstaculo_direita.includecategory(&cat_obstaculo[0]);
	  obstaculo_direita.includecategory(&cat_obstaculo[1]);
	  obstaculo_direita.includecategory(&cat_obstaculo[2]);

    /*** Proximidade do obstáculo - end ***/
    
    /*** Velocidade Linear para Desvio de Obstáculo - begin ***/

	  fuzzy_set cat_vel_lin_obs[4];
                                        // Valores da primeira abordagem (Comentado)
	  cat_vel_lin_obs[0].setname("QZ");
	  cat_vel_lin_obs[0].setrange(0,1);
	  cat_vel_lin_obs[0].setval(0,0,0.35); // (0,0,0.3)

      cat_vel_lin_obs[1].setname("VERYSLOW");
	  cat_vel_lin_obs[1].setrange(0,1);
	  cat_vel_lin_obs[1].setval(0.2,0.4,0.6); // (0,0.3,0.6)

	  cat_vel_lin_obs[2].setname("SLOW");
	  cat_vel_lin_obs[2].setrange(0,1);
	  cat_vel_lin_obs[2].setval(0.5,0.65,1,1); // (0.3,0.6,1,1)

	  linguisticvariable vel_linear_obs;
	  vel_linear_obs.setname("vel_linear_obs");
	  vel_linear_obs.includecategory(&cat_vel_lin_obs[0]);
	  vel_linear_obs.includecategory(&cat_vel_lin_obs[1]);
	  vel_linear_obs.includecategory(&cat_vel_lin_obs[2]);

    /*** Velocidade Linear para Desvio de Obstáculo - end ***/
    
    /*** Velocidade Angular para Desvio de Obstáculo - begin ***/

	fuzzy_set cat_vel_ang_obs[7];
											// Valores da primeira abordagem (Comentado)
	cat_vel_ang_obs[0].setname("NVB");
	cat_vel_ang_obs[0].setrange(-3,0); 
	cat_vel_ang_obs[0].setval(-3, -3, -2.5, -2); 

	cat_vel_ang_obs[1].setname("NB");
	cat_vel_ang_obs[1].setrange(-3,0); 
	cat_vel_ang_obs[1].setval(-2.5, -2, -1); 

	cat_vel_ang_obs[2].setname("NS");
	cat_vel_ang_obs[2].setrange(-3,0); 
	cat_vel_ang_obs[2].setval(-2, -1, -0.2); // (-2, -1, -0.4)

	cat_vel_ang_obs[3].setname("QZ");
	cat_vel_ang_obs[3].setrange(-3,3); 
	cat_vel_ang_obs[3].setval(-0.3, 0, 0.3); // (-0.7, 0, 0.7)

	cat_vel_ang_obs[4].setname("PS");
	cat_vel_ang_obs[4].setrange(0,3);
	cat_vel_ang_obs[4].setval(0.2, 1, 2); // (0.4, 1, 2)

	cat_vel_ang_obs[5].setname("PB");
	cat_vel_ang_obs[5].setrange(0,3);
	cat_vel_ang_obs[5].setval(1, 2, 2.5); 

	cat_vel_ang_obs[6].setname("PVB");
	cat_vel_ang_obs[6].setrange(0,3);
	cat_vel_ang_obs[6].setval(2, 2.5, 3, 3); 

	linguisticvariable vel_angular_obs;
	vel_angular_obs.setname("vel_angular");
	vel_angular_obs.includecategory(&cat_vel_ang_obs[0]);
	vel_angular_obs.includecategory(&cat_vel_ang_obs[1]);
	vel_angular_obs.includecategory(&cat_vel_ang_obs[2]);
	vel_angular_obs.includecategory(&cat_vel_ang_obs[3]);
	vel_angular_obs.includecategory(&cat_vel_ang_obs[4]);
	vel_angular_obs.includecategory(&cat_vel_ang_obs[5]);
	vel_angular_obs.includecategory(&cat_vel_ang_obs[6]);
		
/*** Velocidade Angular para Desvio de Obstáculo - end ***/

//****************************************************************//
//		      REGRAS FUZZY PARA DESVIO DE OBSTÁCULO	              //
//****************************************************************//

/*** Regras de controle de desvio linear - begin ***/

	fuzzy_control fc_desvio_linear;
	fc_desvio_linear.set_defuzz(CENTROID);
	fc_desvio_linear.definevars(obstaculo_esquerda, obstaculo_frente, obstaculo_direita, vel_linear_obs);

	fc_desvio_linear.insert_rule("VERYCLOSE","VERYCLOSE","VERYCLOSE","QZ");
	fc_desvio_linear.insert_rule("VERYCLOSE","VERYCLOSE","CLOSE","QZ");
	fc_desvio_linear.insert_rule("VERYCLOSE","VERYCLOSE","FAR","QZ");
	fc_desvio_linear.insert_rule("VERYCLOSE","CLOSE","VERYCLOSE","QZ");
	fc_desvio_linear.insert_rule("VERYCLOSE","CLOSE","CLOSE","QZ");
	fc_desvio_linear.insert_rule("VERYCLOSE","CLOSE","FAR","QZ");
	fc_desvio_linear.insert_rule("VERYCLOSE","FAR","VERYCLOSE","QZ");
	fc_desvio_linear.insert_rule("VERYCLOSE","FAR","CLOSE","QZ");
	fc_desvio_linear.insert_rule("VERYCLOSE","FAR","FAR","QZ");
	fc_desvio_linear.insert_rule("CLOSE","VERYCLOSE","VERYCLOSE","QZ");
	fc_desvio_linear.insert_rule("CLOSE","VERYCLOSE","CLOSE","QZ");
	fc_desvio_linear.insert_rule("CLOSE","VERYCLOSE","FAR","QZ");
	fc_desvio_linear.insert_rule("CLOSE","CLOSE","VERYCLOSE","QZ");
	fc_desvio_linear.insert_rule("CLOSE","CLOSE","CLOSE","VERYSLOW");
	fc_desvio_linear.insert_rule("CLOSE","CLOSE","FAR","VERYSLOW");
	fc_desvio_linear.insert_rule("CLOSE","FAR","VERYCLOSE","QZ");
	fc_desvio_linear.insert_rule("CLOSE","FAR","CLOSE","VERYSLOW");
	fc_desvio_linear.insert_rule("CLOSE","FAR","FAR","VERYSLOW");
	fc_desvio_linear.insert_rule("FAR","VERYCLOSE","VERYCLOSE","QZ");
	fc_desvio_linear.insert_rule("FAR","VERYCLOSE","CLOSE","QZ");
	fc_desvio_linear.insert_rule("FAR","VERYCLOSE","FAR","QZ");
	fc_desvio_linear.insert_rule("FAR","CLOSE","VERYCLOSE","QZ");
	fc_desvio_linear.insert_rule("FAR","CLOSE","CLOSE","VERYSLOW");
	fc_desvio_linear.insert_rule("FAR","CLOSE","FAR","VERYSLOW");
	fc_desvio_linear.insert_rule("FAR","FAR","VERYCLOSE","QZ");
	fc_desvio_linear.insert_rule("FAR","FAR","CLOSE","VERYSLOW");//slow
	fc_desvio_linear.insert_rule("FAR","FAR","FAR","SLOW");
	
/*** Regras de controle de desvio linear - end ***/


/*** Regras de controle de desvio angular - begin ***/

	fuzzy_control fc_desvio_angular;
	fc_desvio_angular.set_defuzz(CENTROID);
	fc_desvio_angular.definevars(obstaculo_esquerda, obstaculo_frente, obstaculo_direita, vel_angular_obs);

	fc_desvio_angular.insert_rule("VERYCLOSE","VERYCLOSE","VERYCLOSE","NVB");
	fc_desvio_angular.insert_rule("VERYCLOSE","VERYCLOSE","CLOSE","NVB");
	fc_desvio_angular.insert_rule("VERYCLOSE","VERYCLOSE","FAR","NVB");
	fc_desvio_angular.insert_rule("VERYCLOSE","CLOSE","VERYCLOSE","NVB");
	fc_desvio_angular.insert_rule("VERYCLOSE","CLOSE","CLOSE","NB");
	fc_desvio_angular.insert_rule("VERYCLOSE","CLOSE","FAR","NB");
	fc_desvio_angular.insert_rule("VERYCLOSE","FAR","VERYCLOSE","QZ");
	fc_desvio_angular.insert_rule("VERYCLOSE","FAR","CLOSE","NS");
	fc_desvio_angular.insert_rule("VERYCLOSE","FAR","FAR","NVB");
	fc_desvio_angular.insert_rule("CLOSE","VERYCLOSE","VERYCLOSE","PVB");
	fc_desvio_angular.insert_rule("CLOSE","VERYCLOSE","CLOSE","NVB");
	fc_desvio_angular.insert_rule("CLOSE","VERYCLOSE","FAR","NVB");
	fc_desvio_angular.insert_rule("CLOSE","CLOSE","VERYCLOSE","PVB");
	fc_desvio_angular.insert_rule("CLOSE","CLOSE","CLOSE","NB");
	fc_desvio_angular.insert_rule("CLOSE","CLOSE","FAR","NS");
	fc_desvio_angular.insert_rule("CLOSE","FAR","VERYCLOSE","PB");
	fc_desvio_angular.insert_rule("CLOSE","FAR","CLOSE","NS");
	fc_desvio_angular.insert_rule("CLOSE","FAR","FAR","NS");
	fc_desvio_angular.insert_rule("FAR","VERYCLOSE","VERYCLOSE","PVB");
	fc_desvio_angular.insert_rule("FAR","VERYCLOSE","CLOSE","PVB");
	fc_desvio_angular.insert_rule("FAR","VERYCLOSE","FAR","NVB");
	fc_desvio_angular.insert_rule("FAR","CLOSE","VERYCLOSE","PVB");
	fc_desvio_angular.insert_rule("FAR","CLOSE","CLOSE","PS");
	fc_desvio_angular.insert_rule("FAR","CLOSE","FAR","NS");
	fc_desvio_angular.insert_rule("FAR","FAR","VERYCLOSE","PVB");
	fc_desvio_angular.insert_rule("FAR","FAR","CLOSE","PS");
	fc_desvio_angular.insert_rule("FAR","FAR","FAR","NS");
	
/*** Regras de controle de desvio angular - end ***/

      
      
      if (ros::ok())
      {
        geometry_msgs::Twist msg;
    	int i;
        
        ROS_INFO("Ligando o controlador Fuzzy PID ...");
         	
    	msg.linear.x = 0;
        msg.angular.z = 0;
    	
    	tf::StampedTransform transform;
		
	    try {
				listener.waitForTransform("odom", "base_link", ros::Time(0), ros::Duration(5.0));
				listener.lookupTransform("odom", "base_link", ros::Time(0), transform);
				
				setpoint.position.x = 5.049;
				setpoint.position.y = 0.760;
    	
				pose.position.x = transform.getOrigin().x(); 
				pose.position.y = transform.getOrigin().y(); 
				pose.position.z = 0; 
											
			} catch (tf::TransformException ex){
				ROS_ERROR("%s",ex.what());
				ros::Duration(1.0).sleep();
			}			
			
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
				
		  if (is_frente || is_esquerda || is_direita) { 

				msg.angular.z = fc_desvio_angular.make_inference(esquerda, frente, direita);
				msg.linear.x = fc_desvio_linear.make_inference(esquerda, frente, direita)*max_lin_obs;

				erropos = sqrt(pow(setpoint.position.x - pose.position.x, 2) + pow(setpoint.position.y - pose.position.y, 2));

				pub.publish(msg);
				ros::spinOnce();
				loop_rate.sleep();

				
				ROS_INFO("******* OBSTACULO DETECTADO - FUZZY *******");	
				ROS_INFO("msg.linear.x, msg.angular.z: [%f] [%f]", msg.linear.x, msg.angular.z);
				ROS_INFO("frente: [%f]", frente);
				ROS_INFO("direita: [%f]", direita);
				ROS_INFO("esquerda: [%f]", esquerda);
				ROS_INFO("range_desvio: [%f]", range_desvio);
				ROS_INFO("pose.position.x, pose.position.y: [%f] [%f]", pose.position.x, pose.position.y);
				ROS_INFO("---------------------------------------------------------------------------------");
				ROS_INFO("---------------------------------------------------------------------------------");
				ROS_INFO("---------------------------------------------------------------------------------");
				

			} else {		
		   	   			
				  // Cálculo dt
				  tEnd = ros::Time::now();
				  ros::Duration dt = tEnd - tBegin;
				  tBegin = tEnd;
								 
				  angulo = atan2(setpoint.position.y - pose.position.y, setpoint.position.x - pose.position.x);
				  
				  erroAngulo =  angulo - theta_current;
				  
				  if (erroAngulo > M_PI)
						erroAngulo-= 2*M_PI;
				  if (erroAngulo <= -M_PI)
						erroAngulo += 2*M_PI; 
						
				  pidAngulo.erro = erroAngulo;
				  pidAngulo.dt = dt.nsec;
						 
				  // Cálculo erro da distância euclidiana
				  erroDistancia = sqrt(pow(setpoint.position.x - pose.position.x, 2) + pow(setpoint.position.y - pose.position.y, 2));
				  erropos = erroDistancia;
				  pidDistancia.erro = erroDistancia;
				  pidDistancia.dt = dt.nsec;
				  
				  // Cálculo dos PIDs
				  calculaPID(&pidAngulo);
				  calculaPID(&pidDistancia);
						  
				  
				  if (abs(pidAngulo.saida) > 3*M_PI/2) {
					  if (pidAngulo.saida > 0) {
						   msg.angular.z = 3*M_PI/2;
						 }
					  if (pidAngulo.saida < 0) {
						   msg.angular.z = -3*M_PI/2;
					  }
							
				  } else {
					  msg.angular.z = pidAngulo.saida;
				  }	  
						  
				  if (abs(pidDistancia.saida) > 0.8) { //5
					  msg.linear.x = 0.8;
				  } else {
					  msg.linear.x = pidDistancia.saida;
				  }			
										  
				  ROS_INFO("******* CONTROLADOR - PID *******");
				  ROS_INFO("msg.linear.x, msg.angular.z: [%f] [%f]", msg.linear.x, msg.angular.z);
				  ROS_INFO("---------------------------------------------------------------------------------");
				  ROS_INFO("X , Y, angulo: [%f] [%f] [%f]", transform.getOrigin().x(), transform.getOrigin().y(), angulo*57.29);	
				  ROS_INFO("---------------------------------------------------------------------------------");	
				  ROS_INFO("setpoint.position.x, setpoint.position.y: [%f] [%f]", setpoint.position.x, setpoint.position.y);	
				  ROS_INFO("---------------------------------------------------------------------------------");
				  ROS_INFO("---------------------------------------------------------------------------------");
				  ROS_INFO("---------------------------------------------------------------------------------");
				  ROS_INFO("---------------------------------------------------------------------------------");		 
				  
				  pub.publish(msg);
				  ros::spinOnce();			
				  loop_rate.sleep();           
				} 
			}    	
    	
      }
     
      return 0;
    }

