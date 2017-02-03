package org.usfirst.frc.team20.robot;

import java.io.IOException;

import com.ctre.CANTalon;
import com.ctre.CANTalon.FeedbackDevice;
import com.kauailabs.navx.frc.AHRS;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SerialPort;
//import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
//import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */




public class Robot extends IterativeRobot implements PIDOutput{
	    public static int AUTOMODE = AutoModes.AUTO_MODE_1;
	    double multiplier;
	    double distance;
	    double speed;
	    int state;
	    float turnAngle = 0.0f;
	    boolean done;
        boolean turnAngleSet;
        boolean turnAngleSet2;
	    AHRS hrs;
	    CANTalon rightMaster = new CANTalon(2);
	    CANTalon rightSlave = new CANTalon(3);
	    CANTalon leftMaster = new CANTalon(0);
	    CANTalon leftSlave = new CANTalon(1); 
	    Util util = new Util();
	    double angle;
	    int test = 0;
	    PIDController turnController;
	    RobotDrive MyDrive;
	    double rotateToAngleRate;
	    double currentRotationRate;
	    Timer currentTime;   
	    int autoMode = 0;
	        //myRobot.setExpiration(0.1);
	  
	/**
	 * This function is run when the robot is first started up and should be
	 * used for any initialization code.
	 */
	
	@Override
	public void robotInit() {
		hrs = new AHRS(SerialPort.Port.kMXP);
		rightSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
		rightSlave.set(rightMaster.getDeviceID());
		leftSlave.changeControlMode(CANTalon.TalonControlMode.Follower);
		leftSlave.set(leftMaster.getDeviceID());
        //hrs.reset();
		MyDrive = new RobotDrive(rightMaster,leftMaster);
		leftMaster.setFeedbackDevice(FeedbackDevice.QuadEncoder);
		//leftMaster.configEncoderCodesPerRev(25);
        switch (AUTOMODE) {
        	case AutoModes.AUTO_MODE_1 :
        		//state = States.GET_CAMERA_ANGLE;
        		break;
        	default :
        		System.out.println("AUTO MODE = " + AUTOMODE);
        }
  	}
	
	  
	
	

	/**
	 * This autonomous (along with the chooser code above) shows how to select
	 * between different autonomous modes using the dashboard. The sendable
	 * chooser code works with the Java SmartDashboard. If you prefer the
	 * LabVIEW Dashboard, remove all of the chooser code and uncomment the
	 * getString line to get the auto name from the text box below the Gyro
	 *
	 * You can add additional auto modes by adding additional comparisons to the
	 * switch structure below with additional strings. If using the
	 * SendableChooser make sure to add them to the chooser code above as well.
	 */
	@Override
	public void autonomousInit() {
		leftMaster.setEncPosition(0);
		AUTOMODE = AutoModes.AUTO_MODE_2;
		MyDrive.setSafetyEnabled(false);
		hrs.reset();
		
		System.out.println("???????????????????????????????????????Resetting gyro " + hrs.getAngle());
		state = States.GET_CAMERA_ANGLE;
		
	}
	
  	/**
	 * This function is called periodically during autonomous
	 */
	@Override
	public void autonomousPeriodic() {
		
		switch (AUTOMODE) {
			case AutoModes.AUTO_MODE_1:
				Automode001();
				break;
			case AutoModes.AUTO_MODE_2:
				Automode002();
				break;

			default :
				System.out.println("Auto Mode not set");
		}
		/*
		if (done == false ) {
			turnController.setSetpoint(turnAngle);
			turnController.enable();
			done = true;
		}
			
			currentRotationRate = rotateToAngleRate;
			try {
				MyDrive.drive(0.0, currentRotationRate);
				Timer.delay(.005);
				System.out.println(hrs.getAngle());
			} catch ( RuntimeException ex ) {
				DriverStation.reportError("Error communicating with drive system: " + ex.getMessage(), true);
		}  */
	}
	  
	

	/**
	 * This function is called periodically during operator control
	 */
	@Override
	public void teleopPeriodic() {
		MyDrive.setSafetyEnabled(false);
	}

	/**
	 * This function is called periodically during test mode
	 */
	@Override
	public void testPeriodic() {
	try {
		String distance = util.readSocket("10.0.20.9", 9999,"008");
		System.out.println("Starting distance 1: ");
		System.out.println(distance);
	} catch (IOException e) {
		// TODO Auto-generated catch block
		System.out.println("Test error: " + e.toString());
	}
		
	}
	
	@Override
	public void pidWrite(double output) {
	
		// TODO Auto-generated method stub
		//System.out.println("output = " + output);
		//Timer.delay(.02);
		 rotateToAngleRate = output;
		 
		
	}
	
//Auto modes
private void Automode001()
{
	if (state == States.GET_CAMERA_ANGLE){ 
		turnAngle = getCameraAngle();
		System.out.println("++++ GetCameraAngle Turn Angle =  " + turnAngle);
		state = States.SET_PID_LOOP;
	}
	if (state == States.SET_PID_LOOP){
			setTurnController();
			System.out.println("++++++++++++++++++++++++Set Pid Loop ************** ");
			state = States.TURN_ANGLE;
		}
		
	if (state == States.TURN_ANGLE){
		
			
		System.out.println("++++++++++++++++++++++++Turn Angle " + turnAngle + " navx angle " + hrs.getAngle());
		System.out.println("++++++++++++++++++++++++currentRotationRate " + currentRotationRate);
			TurnAngle();
			if (Math.abs(currentRotationRate) < .23 && Math.abs(turnAngle - hrs.getAngle()) < .3 ){
				currentRotationRate = 0;
				TurnAngle();
				turnController.disable();
				state = States.DONE;
			}
			}
			}
private void Automode002()
{
	multiplier = Double.parseDouble(SmartDashboard.getString("DB/String 0", ""));
	distance  = Double.parseDouble(SmartDashboard.getString("DB/String 1", ""));
	speed = Double.parseDouble(SmartDashboard.getString("DB/String 2", ""));
	driveStraight(speed,distance);
	//turnAngle = 45;
	//TurnAngle();
	
	
}
public void driveStraight(double speed, double inches){
	System.out.println("Speed" + speed); // .5
	System.out.println("multiplier" + multiplier); // 6.5
	System.out.println("distance" + distance); // 80
	
	if(leftMaster.getEncPosition()/1024*Math.PI*4 > (inches*multiplier)){
		System.out.println("Done");
		leftMaster.set(0);
		rightMaster.set(0);
		
		//leftMaster.set(0);
		//rightMaster.set(0);
	}
	else{
		rightMaster.set(speed);
		leftMaster.set(-speed);
		System.out.println(leftMaster.getEncPosition());
	}
	
	
}
				
			
		
		
		



private void setTurnController() {
    turnController = new PIDController(PidValues.kP, PidValues.kI, PidValues.kD, PidValues.kF, hrs,this);
    turnController.setInputRange(-180.0f,  180.0f);
    turnController.setOutputRange(-1.0, 1.0);
    turnController.setAbsoluteTolerance(PidValues.kToleranceDegrees);
    turnController.setContinuous(true);
    turnController.setSetpoint(turnAngle);
    turnController.enable();
}


private float getCameraAngle()
{
	    float  angleToTurn = 500f;
		try {
			System.out.println("*******Trying to turn angle");
			angleToTurn = Float.parseFloat(util.readSocket("10.0.20.9", 9999, "009"));
			System.out.println("*******Turned angle  ********** = " + turnAngle );
		} catch (NumberFormatException | IOException e) {
			// TODO Auto-generated catch block
			e.printStackTrace();
		}

	   return angleToTurn;
}	

private void TurnAngle()
{
	currentRotationRate = rotateToAngleRate;
//	 if (Math.abs(turnAngle - hrs.getAngle()) < .6)
//	 {
//		 MyDrive.drive(0.0, 0.0);
//		 state = States.DONE;
//		 System.out.println("++++++++++++++++++Done++++++++++++++++");
//	 }
//	 else
//	 {
		 try {
			 MyDrive.arcadeDrive(0.0, currentRotationRate);
			 System.out.println(hrs.getAngle());
		 	} catch ( RuntimeException ex ) {
		 		DriverStation.reportError("Error communicating with drive system: " + ex.getMessage(), true);
		 	}
	 //}
}

	
}





	


