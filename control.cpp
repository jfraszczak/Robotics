// controlDLL.cpp : Defines the entry point for the DLL application.

//

#include "servo.h"

#include "param.h"

#include "control.h"

//#include "UiAgent.h"

#include "PrVector.h"

#include "PrMatrix.h"

#include "Utils.h" // misc. utility functions, such as toRad, toDeg, etc.

#include <math.h>

#include <algorithm>

using std::min;

using std::max;





struct CubicSpline {

   double t0 , tf ;

   PrVector a0 , a1 , a2 , a3 ;

};

CubicSpline spline ;



struct Circle{

   double t0;
   double t_rot = 5.0;

   double r = 0.2; 
   double x_center = 0.6;

   double y_center = 0.35;

   double alpha = 0;
   
   void resetTime(){
   	t0 = gv.curTime;
   }

   

};

Circle circle;



// Compute total trajectory length

double computeTf ( GlobalVariables & gv )

{  

   double tf = 0;

   double constraint1, constraint2; 

   for(int i = 0; i < gv.dof; i++){

      constraint1 = abs(3 / 2 * (gv.qd[i] - gv.q[i]) / gv.dqmax[i]);

      constraint2 = sqrt(abs(6 * (gv.qd[i] - gv.q[i]) / gv.ddqmax[i]));

      if(tf < std::max(constraint1, constraint2)){

         tf = std::max(constraint1, constraint2);

      }

   }

   return tf;

}



void PrintDebug(GlobalVariables& gv);



// *******************************************************************

// Initialization functions

// *******************************************************************



void InitControl(GlobalVariables& gv) 

{

   // This code runs before the first servo loop

}



void PreprocessControl(GlobalVariables& gv)

{

   // This code runs on every servo loop, just before the control law

   

   

  // 

    if ((gv.dof == 3) || (gv.dof == 6)) {



        //get the correct joint angles depending on the current mode:

        double q1,q2,q3;

        if (gv.dof == 3) {

            q1 = gv.q[0];

            q2 = gv.q[1];

            q3 = gv.q[2];

        } else if (gv.dof == 6) {

            q1 = gv.q[1];

            q2 = gv.q[2];

            q3 = gv.q[4];

        }



        PrVector3 g123 = PrVector3(0,0,0); //Variable that holds the torque exerted by gravity for each joint



        //Compute g123 here!

        double c1 = cos(q1);

        double s12 = sin(q1 + q2);

        double s123 = sin(q1 + q2 + q3);

        

        double r1 = R2;

        double r2 = 0.189738;

        double r3 = R6;

        

        double l1 = L2;

        double l2 = L3;

        double l3 = L6;

        

        double m1 = M2;

        double m2 = M3 + M4 + M5;

        double m3 = M6;

        

        double g = -9.81;

        



        g123[2] = r3*s123*m3*g;

        g123[1] = r2*s12*m2*g + (l2*s12 + r3*s123)*m3*g;

        g123[0] = r1*c1*m1*g + (l1*c1 + r2*s12)*m2*g + (l1*c1 + l2*s12 + r3*s123)*m3*g;





        //maps the torques to the right joint indices depending on the current mode:

        if (gv.dof == 3) {

            gv.G[0] = g123[0];

            gv.G[1] = g123[1];

            gv.G[2] = g123[2];

        } else if (gv.dof == 6) {

            gv.G[1] = g123[0];

            gv.G[2] = g123[1];

            gv.G[4] = g123[2];

        }

        // printing example, do not leave print inthe handed in solution 

//        printVariable(g123, "g123");

    } else {

        gv.G = PrVector(gv.G.size());

    }   

}



void PostprocessControl(GlobalVariables& gv) 

{

   // This code runs on every servo loop, just after the control law

}



void initFloatControl(GlobalVariables& gv) 

{

    // Control Initialization Code Here

}



void initOpenControl(GlobalVariables& gv) 

{

	// Control Initialization Code Here

}



void initNjholdControl(GlobalVariables& gv) 

{

	// Control Initialization Code Here

}



void initJholdControl(GlobalVariables& gv) 

{

	// Control Initialization Code Here

}



void initNjmoveControl(GlobalVariables& gv) 

{

	// Control Initialization Code Here

}



void initJmoveControl(GlobalVariables& gv) 

{

	// Control Initialization Code Here

}



void initNjgotoControl(GlobalVariables& gv) 

{

	// Control Initialization Code Here

} 



void initJgotoControl(GlobalVariables& gv) 

{

	// Control Initialization Code Here

}



void initNjtrackControl(GlobalVariables& gv) 

{

	// Control Initialization Code Here

   

   spline.t0 = gv.curTime;

   spline.tf = computeTf(gv);

   spline.a0 = PrVector(gv.dof);

   spline.a1 = PrVector(gv.dof);

   spline.a2 = PrVector(gv.dof);

   spline.a3 = PrVector(gv.dof);

   

   for(int i = 0; i < gv.dof; i++){

      spline.a0[i] = gv.q[i];

      spline.a1[i] = 0;

      spline.a2[i] = 3 / pow(spline.tf, 2) * (gv.qd[i] - gv.q[i]);

      spline.a3[i] = -2 / pow(spline.tf, 3) * (gv.qd[i] - gv.q[i]);

   }

}



void initJtrackControl(GlobalVariables& gv) 

{

	// Control Initialization Code Here

}



void initNxtrackControl(GlobalVariables& gv) 

{

	// Control Initialization Code Here

}



void initXtrackControl(GlobalVariables& gv) 

{

	// Control Initialization Code Here

} 



void initNholdControl(GlobalVariables& gv) 

{

	// Control Initialization Code Here

}



void initHoldControl(GlobalVariables& gv) 

{

	// Control Initialization Code Here

}



void initNgotoControl(GlobalVariables& gv) 

{

	// Control Initialization Code Here

} 



void initGotoControl(GlobalVariables& gv) 

{

	// Control Initialization Code Here

} 



void initNtrackControl(GlobalVariables& gv) 

{

	// Control Initialization Code Here

}



void initTrackControl(GlobalVariables& gv) 

{

	// Control Initialization Code Here

} 



void initPfmoveControl(GlobalVariables& gv) 

{

	// Control Initialization Code Here

} 



void initLineControl(GlobalVariables& gv) 

{

	// Control Initialization Code Here

}



void initProj1Control(GlobalVariables& gv) 

{

	// Control Initialization Code Her

	gv.qd[0] = 0.096;

	gv.qd[1] = 0.967;

	gv.qd[2] = -1.061;

	initNjtrackControl(gv);

}



void initProj2Control(GlobalVariables& gv) 

{

	// Control Initialization Code Here

	circle.resetTime();

}



void initProj3Control(GlobalVariables& gv) 

{

	// Control Initialization Code Here
	circle.resetTime();

}





// *******************************************************************

// Control laws

// *******************************************************************



void noControl(GlobalVariables& gv)

{

}



void floatControl(GlobalVariables& gv)

{   

	for(int i = 0; i < gv.dof; i++)

		gv.tau[i] = gv.G[i];

	// this only works on the real robot unless the function is changed to use cout

	// the handed in solution must not contain any printouts

//	PrintDebug(gv);

}



void openControl(GlobalVariables& gv)

{

   floatControl(gv);  // Remove this line when you implement this controller

}



void njholdControl(GlobalVariables& gv) 

{

   floatControl(gv);  // Remove this line when you implement this controller

}



void jholdControl(GlobalVariables& gv) 

{

   floatControl(gv);  // Remove this line when you implement this controller

}



void njmoveControl(GlobalVariables& gv)

{

   for(int i = 0; i < gv.dof; i++)

      gv.tau[i] = gv.kp[i] * (gv.qd[i] - gv.q[i]);

}



void jmoveControl(GlobalVariables& gv)

{

   floatControl(gv);  // Remove this line when you implement this controller

}



void njgotoControl(GlobalVariables& gv) 

{	

   for(int i = 0; i < gv.dof; i++)

      gv.tau[i] = gv.kp[i] * (gv.qd[i] - gv.q[i]) + gv.G[i];

}



void jgotoControl(GlobalVariables& gv) 

{

   for(int i = 0; i < gv.dof; i++)

      gv.tau[i] = gv.kp[i] * (gv.qd[i] - gv.q[i]) - gv.kv[i] * gv.dq[i] + gv.G[i];

}



void njtrackControl(GlobalVariables& gv) 

{

   if(gv.curTime > spline.tf + spline.t0){

   	floatControl(gv);

   	return;

   }

   

   double q_des, dq_des;

   for(int i = 0; i < gv.dof; i++){

      q_des = spline.a0[i] + spline.a1[i] * (gv.curTime - spline.t0) + spline.a2[i] * pow((gv.curTime - spline.t0), 2) + spline.a3[i] * pow((gv.curTime - spline.t0), 3);

      dq_des = spline.a1[i] + 2 * spline.a2[i] * (gv.curTime - spline.t0) + 3 * spline.a3[i] * pow((gv.curTime - spline.t0), 2);

      gv.qd[i] = q_des;

      gv.tau[i] = gv.kp[i] * (q_des - gv.q[i]) + gv.kv[i] * (dq_des - gv.dq[i]) + gv.G[i];

   }

}



void jtrackControl(GlobalVariables& gv)

{

   floatControl(gv);  // Remove this line when you implement this controller

}



void nxtrackControl(GlobalVariables& gv) 

{

   floatControl(gv);  // Remove this line when you implement this controller

}



void xtrackControl(GlobalVariables& gv) 

{

   floatControl(gv);  // Remove this line when you implement this controller

}



void nholdControl(GlobalVariables& gv) 

{

   floatControl(gv);  // Remove this line when you implement this controller

}



void holdControl(GlobalVariables& gv) 

{

   floatControl(gv);  // Remove this line when you implement this controller

}



void ngotoControl(GlobalVariables& gv) 

{

   floatControl(gv);  // Remove this line when you implement this controller

}



void gotoControl(GlobalVariables& gv) 

{

   floatControl(gv);  // Remove this line when you implement this controller

}



void ntrackControl(GlobalVariables& gv) 

{

   floatControl(gv);  // Remove this line when you implement this controller

}



void trackControl(GlobalVariables& gv) 

{

   floatControl(gv);  // Remove this line when you implement this controller

}



void pfmoveControl(GlobalVariables& gv) 

{

   floatControl(gv);  // Remove this line when you implement this controller

}



void lineControl(GlobalVariables& gv)

{

   floatControl(gv);  // Remove this line when you implement this controller

}



void proj1Control(GlobalVariables& gv) 

{

    njtrackControl(gv);

}



void proj2Control(GlobalVariables& gv)

{

   // Trajectory computation

   gv.xd[0] = cos(-(gv.curTime - circle.t0) / circle.t_rot * 2 * M_PI) * circle.r + circle.x_center;

   gv.xd[1] = sin(-(gv.curTime - circle.t0) / circle.t_rot * 2 * M_PI) * circle.r + circle.y_center;

   gv.xd[2] = circle.alpha;

   

   gv.dxd[0] = sin(-(gv.curTime - circle.t0) / circle.t_rot * 2 * M_PI) * 2 * M_PI * circle.r / circle.t_rot;

   gv.dxd[1] = -cos(-(gv.curTime - circle.t0) / circle.t_rot * 2 * M_PI) * 2 * M_PI * circle.r / circle.t_rot;

   gv.dxd[2] = 0;

   

   gv.qd[0] = gv.dxd[0];

   gv.qd[1] = gv.dxd[1];

   gv.qd[2] = gv.dxd[2];

  

   // Controller

   double f[3];

   for(int i = 0; i < 3; i++){

      f[i] = gv.kp[i] * (gv.xd[i] - gv.x[i]) + gv.kv[i] * (gv.dxd[i] - gv.dx[i]);

   }

   

   for(int i = 0; i < 3; i++){

      gv.tau[i] = gv.Jtranspose[i][0] * f[0] + gv.Jtranspose[i][1] * f[1] + gv.Jtranspose[i][2] * f[2] + gv.G[i];

   }

}





void proj3Control(GlobalVariables& gv) 

{

   double angular_velocity = 2 * M_PI / circle.t_rot;
   double angular_acceleration = 2 * M_PI / 25;
   double rotations_number = 3;
   
   double tb = angular_velocity / angular_acceleration;
   double alpha_des = 0;
   double angular_velocity_des = 0;
   
   double rotations_number_during_acceleration = 0.5 * angular_acceleration * pow(tb, 2) / (2 * M_PI);
   double rotation_trajectory_time = (rotations_number - 2 * rotations_number_during_acceleration) * circle.t_rot;
   
   if(gv.curTime < circle.t0 + tb){
   	angular_velocity_des = -angular_acceleration * (gv.curTime - circle.t0);
   	alpha_des = -0.5 * angular_acceleration * pow((gv.curTime - circle.t0), 2);
   	
   	gv.xd[0] = cos(alpha_des) * circle.r + circle.x_center;

	gv.xd[1] = sin(alpha_des) * circle.r + circle.y_center;

	gv.xd[2] = circle.alpha;

	   

	gv.dxd[0] = -sin(alpha_des) * circle.r * angular_velocity_des;

	gv.dxd[1] = cos(alpha_des) * circle.r * angular_velocity_des;

	gv.dxd[2] = 0;
	
	gv.qd[0] = alpha_des;

   	gv.qd[1] = angular_velocity_des;
   }
   else if(gv.curTime < circle.t0 + tb + rotation_trajectory_time){
      gv.xd[0] = cos(-(gv.curTime - circle.t0 - tb) / circle.t_rot * 2 * M_PI - 0.5 * angular_acceleration * pow(tb, 2)) * circle.r + circle.x_center;

      gv.xd[1] = sin(-(gv.curTime - circle.t0 - tb) / circle.t_rot * 2 * M_PI - 0.5 * angular_acceleration * pow(tb, 2)) * circle.r + circle.y_center;

      gv.xd[2] = circle.alpha;

   

      gv.dxd[0] = sin(-(gv.curTime - circle.t0 - tb) / circle.t_rot * 2 * M_PI - 0.5 * angular_acceleration * pow(tb, 2)) * 2 * M_PI * circle.r / circle.t_rot;

      gv.dxd[1] = -cos(-(gv.curTime - circle.t0 - tb) / circle.t_rot * 2 * M_PI - 0.5 * angular_acceleration * pow(tb, 2)) * 2 * M_PI * circle.r / circle.t_rot;

      gv.dxd[2] = 0;
      
      gv.qd[0] = -(gv.curTime - circle.t0 - tb) / circle.t_rot * 2 * M_PI - 0.5 * angular_acceleration * pow(tb, 2);

      gv.qd[1] = -angular_velocity;
   }
   else if(gv.curTime < circle.t0 + rotation_trajectory_time + 2 * tb){
        angular_velocity_des = -angular_velocity + angular_acceleration * (gv.curTime - (circle.t0 + rotation_trajectory_time + tb));
   	alpha_des = 0.5 * angular_acceleration * pow(gv.curTime - (circle.t0 + rotation_trajectory_time + tb), 2) - angular_velocity * (gv.curTime - (circle.t0 + rotation_trajectory_time + tb));
   	alpha_des -= rotation_trajectory_time / circle.t_rot * 2 * M_PI + 0.5 * angular_acceleration * pow(tb, 2);
   	
   	gv.xd[0] = cos(alpha_des) * circle.r + circle.x_center;

	gv.xd[1] = sin(alpha_des) * circle.r + circle.y_center;

	gv.xd[2] = circle.alpha;

	   

	gv.dxd[0] = -sin(alpha_des) * circle.r * angular_velocity_des;

	gv.dxd[1] = cos(alpha_des) * circle.r * angular_velocity_des;

	gv.dxd[2] = 0;
	
	gv.qd[0] = alpha_des;

   	gv.qd[1] = angular_velocity_des;
   }
   
   gv.qd[0] = fmod(gv.qd[0], 2 * M_PI);
   
   // Controller

   double f[3];

   for(int i = 0; i < 3; i++){

      f[i] = gv.kp[i] * (gv.xd[i] - gv.x[i]) + gv.kv[i] * (gv.dxd[i] - gv.dx[i]);

   }

   

   for(int i = 0; i < 3; i++){

      gv.tau[i] = gv.Jtranspose[i][0] * f[0] + gv.Jtranspose[i][1] * f[1] + gv.Jtranspose[i][2] * f[2] + gv.G[i];

   }

}



// *******************************************************************

// Debug function

// *******************************************************************



void PrintDebug(GlobalVariables& gv)

{

   // Replace this code with any debug information you'd like to get

   // when you type "pdebug" at the prompt.

   printf( "This sample code prints the torque and mass\n" );

   gv.tau.display( "tau" );

   gv.A.display( "A" );

}



#ifdef WIN32

// *******************************************************************

// XPrintf(): Replacement for printf() which calls ui->VDisplay()

// whenever the ui object is available.  See utility/XPrintf.h.

// *******************************************************************



int XPrintf( const char* fmt, ... )

{

  int returnValue;

  va_list argptr;

  va_start( argptr, fmt );



  returnValue = vprintf( fmt, argptr );



  va_end( argptr );

  return returnValue;

}

#endif //#ifdef WIN32



/********************************************************



END OF DEFAULT STUDENT FILE 



ADD HERE ALL STUDENT DEFINED AND AUX FUNCTIONS 



*******************************************************/

