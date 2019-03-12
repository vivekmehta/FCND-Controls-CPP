#Building a Controller#

##Body rate and roll/pitch control (scenario 2)##
###Implemented body rate control and ###

Code to implement the body rate:

	V3F pqrErr = pqrCmd - pqr;
	V3F MOI(Ixx, Iyy, Izz);
	momentCmd = MOI*(kpPQR*pqrErr);

Code for motor thrust command:

	float l=L/sqrtf(2);
		
	float c = collThrustCmd;
	float t1 = momentCmd.x/l;
	float t2 = momentCmd.y/l;
	float t3 = momentCmd.z/kappa;
		
	cmd.desiredThrustsN[0] = (t1+t2-t3+c) / 4.f; // front left
	cmd.desiredThrustsN[1] = (-t1+t2+t3+c) / 4.f; // front right
	cmd.desiredThrustsN[2] = (t1-t2+t3+c) / 4.f; // rear left
	cmd.desiredThrustsN[3] = (-t1-t2-t3+c) / 4.f; // rear right
	###Implement roll pitch control in C++.

Parameter values: kpPQR = 85, 65, 5

###Implement roll/pitch controller:###

	  float c_d = collThrustCmd/mass;
	        
	  if(collThrustCmd > 0.0) {
	    float target_R13 = -CONSTRAIN(accelCmd[0]/c_d, -maxTiltAngle, maxTiltAngle); 
	    float target_R23 = -CONSTRAIN(accelCmd[1]/c_d, -maxTiltAngle, maxTiltAngle); 

	    float b_x_c_dot = kpBank * (target_R13 - R(0, 2));
	    float b_y_c_dot = kpBank * (target_R23 - R(1, 2));
		
	    pqrCmd[0] = 1/R(2,2) * (R(1,0)* b_x_c_dot - R(0,0)*b_y_c_dot);
	    pqrCmd[1] = 1/R(2,2) * (R(1,1)* b_x_c_dot - R(0,1)*b_y_c_dot); 
	  }
	  else {
	    printf("negative thrust command");
	    pqrCmd[0] = 0.0;
	    pqrCmd[1] = 0.0;
	    collThrustCmd = 0.0;
	  }
	
kpBank: 10

performance matrics output:
	Simulation #270 (../config/2_AttitudeControl.txt)
	PASS: ABS(Quad.Roll) was less than 0.025000 for at least 0.750000 seconds
	PASS: ABS(Quad.Omega.X) was less than 2.500000 for at least 0.750000 seconds

Video showing the result for this:
!(./animations/vivek_scenario2.mov) 


##Position/velocity and yaw angle control (scenario 3)##

### Implement altitude controller (PD version) ###

		velZCmd = CONSTRAIN(velZCmd, -maxDescentRate, maxAscentRate);
		float PTerm = kpPosZ*(posZCmd - posZ);
		float DTerm = kpVelZ*(velZCmd - velZ);
		
		float u1Bar = PTerm + DTerm + accelZCmd;
		float zDotDot = (u1Bar-9.81f)/R(2,2);
			
		thrust = - mass * CONSTRAIN(zDotDot, -maxDescentRate/dt, maxAscentRate/dt);


### Implement lateral position control ###

		// Limiting the velcoity
		if(velCmd.mag()>maxSpeedXY) {
		 velCmd *= maxSpeedXY/velCmd.mag();
		} 
		
		//PD controller for accelration
		accelCmd += kpPosXY * (posCmd - pos) + kpVelXY * (velCmd - vel);
		
		//Limiting the acceleration
		if(accelCmd.mag()>maxAccelXY){
		accelCmd *=  maxAccelXY/accelCmd.mag();
		}


### Implement yaw control ###

		if (yawCmd > 0 ) {
	    	yawCmd = fmodf(yawCmd, 2 * F_PI);
	  	} else {
	    	yawCmd = -fmodf(-yawCmd, 2 * F_PI);
	  	}
	 	float yawErr = yawCmd - yaw;
		
	  	if(yawErr > F_PI) {
	    	yawErr = yawErr - 2.0*F_PI;
	  	}
	  	else if(yawErr < -F_PI) {
	    	yawErr = yawErr + 2.0*F_PI;
	  	}
		
	  	yawRateCmd = kpYaw * yawErr;


Simulation #2 (../config/3_PositionControl.txt)
PASS: ABS(Quad1.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Pos.X) was less than 0.100000 for at least 1.250000 seconds
PASS: ABS(Quad2.Yaw) was less than 0.100000 for at least 1.000000 seconds

parameter values tuned are:

		# Position control gains
		kpPosXY = 30 #1
		kpPosZ = 25 #1

		# Velocity control gains
		kpVelXY = 12
		kpVelZ = 10

		# Angle control gains
		kpBank = 12 #5
		kpYaw = 2 #1

Unlike mentioned in suggestion kpVel values are half of kpPos which are working. 


Video showing the result for this:
!(./animations/vivek_scenario3.mov) 

##Non-idealities and robustness (scenario 4)

### Altitude controller is modififed:

	integratedAltitudeError += (posZCmd - posZ)*dt;
	float ITerm = KiPosZ * integratedAltitudeError
		
	float u1Bar = PTerm + DTerm + ITerm + accelZCmd;


Video showing the result for this:
!(./animations/vivek_scenario4.mov) 


##Tracking trajectories ##

Despite trying tuning for hours couldn't get the tuning with mass 0.4. But looking at python helper code given, i changed the mass to 0.5 and the evalution changed to pass.

Here is video showing the result:
!(./animations/vivek_scenario5.mov) 


