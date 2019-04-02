#include <kipr/botball.h>
// Functions
	void calibrate_gyro();// calibrates gyro to find bias
	void sconG();//drives straight with gyro, used in loops, short for straight con gyro, con being with in spanish
	void tconG();// turns with gyro, without angle parameter
	void twg();// shorthand for turn with gyro method, uses the gyroscope to turn
	void swg();//shorthand for straight with gyro method, drives straight for a set distance
	double bias = 0;// stores the bias from the calibrate gyro method
	double DEGREES_CONVERSION = 594.44;// It was initially 6444.444, but switched for some reason
// Servo ports

    int lift= 0;// indicates the servo port for the attachment that changes the elevation of the igus chain 
	int liftBump = 0;
// Servo movement
	//Lift
	int count = 0;//used in controlling servo movement
	int angle = 350;// angle set
	int base = 0;// starting height
    int up = 700;
    int down = 0;
//Motors    
	int extended = -2120;//-10800
	int igus = 0;// motor port for igus chain
	int extend = -750;// mav speed for igus chain
    int EDistance = 2000; // trial and error to find actual distance
//Speeds
	int Lnormal = 200;
	int Rnormal = 202;
    
	
int main()
{
    create_connect();// attempts to connect the create to the wabally
    calibrate_gyro();
    enable_servos();
    set_servo_position(lift,up);
    while(get_create_lcliff_amt() > 2000 && get_create_rcliff_amt() > 2000)// Finds the front starting box line
    {
        sconG(Lnormal);
        // helps track cliff sensor readings
        //printf("Driving until cliff sensors are below 2000: %d\n" ,get_create_rcliff_amt());
    }
    twg(Lnormal,-Rnormal,90);// Turns to face the electric lines
    while(get_create_lbump() == 0 && get_create_rbump() == 0)//moves until it hits the wall in front of the electric lines
    {
        sconG(Lnormal);
        msleep(10);
    }
    create_stop();
    //printf("Finished the loop\n");
    while(get_create_distance() > -10)// Reverses slightly
    {
        create_drive_direct(-Lnormal,-Rnormal);
        msleep(10);
    }
    create_stop();
    printf("I backed up!\n");
    twg(-Lnormal,Rnormal,90);//turns to face the center of the board
    //drives until the center of the is on the middle line
    while(get_create_lcliff_amt() > 2000 && get_create_rcliff_amt() > 2000)
    {
        sconG(Lnormal);
        //printf("Driving until cliff sensors are below 2000: %d\n" ,get_create_rcliff_amt());//helps track cliff sensor readings
        msleep(10);
    }
    create_stop();
    //This part will involve moving the servos and motors to complete the electric line task
    while (up - count > angle)// Lowers the lift from starting position to active position
        {
        set_servo_position(lift, up - count );//while the servo position is greater than the angle
        count+= 2;//increases the count by 2, effectively lowers servo at faster rate
        msleep(10);
        }
    while(gmpc(igus) > extended )//extends the igus chain towards the washers
    {
    	mav(igus,extend);
        msleep(10);
    }
    off(0);
    printf("Out of the loop\n");
    set_create_distance(0);
    while(get_create_distance() > -100)
    {
        sconG(-200);
        msleep(10);
    }
    create_stop();
    /**
    while(analog(liftBump) == 0)
    {
        sconG(-Lnormal);
        msleep(10);
    }
    **/
    /**while(gmpc(igus) < -extended)//retracts igus chain
    {
    	mav(igus,-extend);
        msleep(10);
    }
    **/
     //drives until the center of the is on the middle line
   while(get_create_lcliff_amt() > 2000 && get_create_rcliff_amt() > 2000)//drives until middle line
   {
       sconG(Lnormal);
       msleep(10);
   }
    set_servo_position(lift,angle);
   while(gmpc(igus) > extended)//entends igus chain
    {
    	mav(igus,extend);
        msleep(10);
    }
    // Code Will follow here...
    
    // and end before here
    disable_servos();
    create_disconnect();//attempts to disconnect the create from the wabally
    return 0;// returns integer value 0 due to main function being an integer type
}//---------------------------------------------------------- End main--------------------------------------------
// Other Methods
	void calibrate_gyro()//sets the gyro to account for bias 
	{
 		int i = 0;// used to take a number of readings for the gyro sensor
    	double avg = 0;// average will be used with sum of gyro values, is total not average despite the name
    	while(i<50)// updates average with gyro functions while integer i is less than 50
    	{
        	avg += gyro_z();
        	msleep(1);
        	i++;
    	}
    	bias = avg / 50;// the bias is equal to the average value of the double avg divided by the number of readings taken, which is 50
    	printf("New Bias %f\n", bias);//prints the bias on wabally screen
            
	}// end void 1
    void twg(int lSpeed, int rSpeed, int targetTheta)// turns using the gyro
	{
    	set_create_total_angle(0);
        double theta = 0;//declares the variable that stores the current degrees
    	targetTheta = targetTheta * DEGREES_CONVERSION;	//uses degrees instead of KIPR degrees
        create_drive_direct(lSpeed, rSpeed);//begins turning at speeds specified in parameters
        msleep(20);
       	//keeps the motors running until the robot reaches the desired angle
    	while(theta < targetTheta)
    	{
        	create_drive_direct(lSpeed,rSpeed);//continues to move using parameter speeds
            msleep(100);//turns for .01 seconds
        	theta += abs(gyro_z() - bias) * 10;//theta = omega(angular velocity, the value returned by gyroscopes) * time
    	}
    	//stops the motors after reaching the turn
 		create_stop();
    }
    void swg(int speed, double distance) // gyro drives straight for a given distance
    {
        double theta = 0;//theta will be used in this instance to track gyro readings 
        set_create_distance(0);//resets create distance
        create_drive_direct(speed,speed+2);//create drives straight
        
        if(distance > 0)//if the create needs to move forward
        {
            while(get_create_distance() < distance)// while the create has traveled less than the distance parameter
            {
                while(get_create_distance() < 5)// Allows for greater theta values to begin driving
                {
                    if(theta > 300)//if the create is accelerating to the left
                    {
                        create_drive_direct(speed + 18, speed - 20); 
                    }
                    else if (theta < -300)// if the create is accelerating to the right
                    {
                        create_drive_direct(speed - 18, speed + 20);
                    }
                    else// it is not driving at an angle, it will go straight
                    {
                        create_drive_direct(speed,speed+2);
                    }
                    msleep(10);
                    theta += (gyro_z() - bias) * 10;// updates theta to accout for bias
                }// end while
                    if(theta > 50)// lower theta values to adapt to lower variations in theta, if it accelerates to the left
                    {
                         create_drive_direct(speed + 18, speed - 20); 
                    }
                    else if (theta < -50)// lower values, if it accelerates to the right
                    {
                        create_drive_direct(speed - 18, speed + 20);
                    }
                    else // create continues to drive if the theta values are within limits
                    {
                       create_drive_direct(speed,speed+2);
                    }
                    msleep(10);
                    theta += (gyro_z() - bias) * 10;
                    //printf("Gyro Z reads %d",gyro_z());
            }//End while
        }
         else// if the distance value is less than zero, if the create will move backwards
            while(get_create_distance() > distance)// while the create has traveled less than the distance parameter
            {
                while(get_create_distance() > -5)// Allows for greater theta values to begin driving
                {
                    if(theta > 300)//if the create accelerates to the left(facing forwards)
                    {
                        create_drive_direct(speed + 18, speed - 20); 
                    }
                    else if (theta < -300)// if the create accelerates to the right(facing forwards)
                    {
                        create_drive_direct(speed - 18, speed + 20);
                    }
                    else//if the create is functioning within limits
                    {
                        create_drive_direct(speed,speed+2);
                    }
                    msleep(10);
                    theta += (gyro_z() - bias) * 10;// updates theta to accout for bias
                    //printf("Gyro Z reads %d",gyro_z());// prints the values for the gyro z value
                }// end while
                    if(theta > 50)// lower theta values to adapt to lower variations in theta
                    {
                         create_drive_direct(speed + 18, speed - 20); 
                    }
                    else if (theta < -50)// lower values
                    {
                        create_drive_direct(speed - 18, speed + 20);
                    }
                    else // create continues to drive if the theta values are within limits
                    {
                       create_drive_direct(speed,speed+2);
                    }
                    msleep(10);
                    theta += (gyro_z() - bias) * 10;
                    //printf("Gyro Z reads %d",gyro_z());
            }//End while  
            create_stop();
    }// end void
    void sconG(int speed) // gyro drives straight for a given distance
    {
        set_create_distance(0);
        double theta = 0;
        create_drive_direct(speed,speed+2);//create drives straight
        msleep(10);
        printf("Create should be driving now\n");
        theta += (gyro_z() - bias) * 10;// updates theta to accout for bias
        //printf("Gyro Z reads %d",gyro_z());// prints the values for the gyro z value
        if(theta > 50)// lower theta values to adapt to lower variations in theta
        {
           	 create_drive_direct(speed + 18, speed - 20); 
        }
        else if (theta < -50)// lower values
        {
            create_drive_direct(speed - 18, speed + 20);
        }
        else // create continues to drive if the theta values are within limits
        {
            create_drive_direct(speed,speed+2);
        }
        //msleep(10);
        theta += (gyro_z() - bias) * 10;
        //printf("Gyro Z reads %d",gyro_z());
        //create_stop();
    }// end void 3
void tconG(int speed1,int speed2) // gyro drives straight for a given distance
    {
        double theta = 0;
        create_drive_direct(speed1,speed2);//create drives straight
        msleep(10);
        theta += (gyro_z() - bias) * 10;// updates theta to accout for bias
        //printf("Gyro Z reads %d",gyro_z());// prints the values for the gyro z value
        if(theta > 50)// lower theta values to adapt to lower variations in theta
        {
           	 create_drive_direct(speed1 + 20, speed2 - 20); 
        }
        else if (theta < -50)// lower values
        {
            create_drive_direct(speed1 - 20, speed2 + 20);
        }
        else // create continues to drive if the theta values are within limits
        {
            create_drive_direct(speed1,speed2);
        }
        //msleep(10);
        theta += (gyro_z() - bias) * 10;
        //printf("Gyro Z reads %d",gyro_z());
        //create_stop();
    }// end void 4