package frc.robot.subsystems;

//import frc.robot.commands.LimeVisionCommand;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
//import edu.wpi.first.wpilibj.drive.Vector2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/**
 * An example subsystem. You can replace me with your own Subsystem.
 */
public class Limelight extends SubsystemBase{
	// Put methods for controlling this subsystem
	// here. Call these from Commands.
	

	
	
	//Create variables
	public double targetD;
	public boolean hasTarget;
	public double xOffset;
	public double yOffset;
	public double area;
	public double skew;
	public double LEDMode;
	public double camMode;
	public double pipeline;


	// public static final Vector2d resolution = new Vector2d(120,120);
	
	public NetworkTable getLimetable() {
		return NetworkTableInstance.getDefault().getTable("limelight");
	}

	
	//public void initDefaultCommand() {
		// Set the default command for a subsystem here.
		//setDefaultCommand(new LimeVisionCommand());
	//}
	
	//Does the camera proccessor have a target?
	public boolean getHasTarget() {
		targetD = getLimetable().getEntry("tv").getDouble(0); 
		if(targetD == 0) {
			hasTarget = false;
		}else if(targetD == 1) {
			hasTarget = true;
		}
		return hasTarget;
	}
	
	// Horizontal Offset From Crosshair To Target (-27 degrees to 27 degrees)
	public double getXOffset() {
		xOffset = getLimetable().getEntry("tx").getDouble(0);
		return xOffset;
	}
	
	//Vertical Offset From Crosshair To Target (-20.5 degrees to 20.5 degrees)
	public double getYOffset() {
		yOffset = getLimetable().getEntry("ty").getDouble(0);
		return yOffset;
	}
	
	//Target Area (0% of image to 100% of image)
	public double getArea() {
		area = getLimetable().getEntry("ta").getDouble(0);
		return area;
	}
	
	//Skew or rotation (-90 degrees to 0 degrees)
	public double getSkew() {
		skew = getLimetable().getEntry("ts").getDouble(0);
		return skew;
	}

	//Limelight LED state
	public double getLEDMode() {
		LEDMode = getLimetable().getEntry("ledMode").getDouble(1);
		return LEDMode;
	}
	
	//Limelight Camera state
	public double getCamMode() {
		camMode = getLimetable().getEntry("camMode").getDouble(0);
		return camMode;
	}
	
	//get current pipeline that is being used
	public double getPipeline() {
		pipeline = getLimetable().getEntry("pipeline").getDouble(0);
		return pipeline;
	}
	
	//Set the LED mode of the limelight
	public void switchLED() {
		if(getLEDMode() == 0) {
			getLimetable().getEntry("ledMode").setDouble(1);
			SmartDashboard.putString("LED Mode", "Off");
		}else if(getLEDMode() == 1) {
			getLimetable().getEntry("ledMode").setDouble(0);
			SmartDashboard.putString("LED Mode", "On");
		}else if(getLEDMode() == 2) {
			getLimetable().getEntry("ledMode").setDouble(1);
			SmartDashboard.putString("LED Mode", "Off");
		}
	}
	
	//Set the camera mode
	public void switchCamera() {
		if(getCamMode() == 0) {
			getLimetable().getEntry("camMode").setDouble(1);
			SmartDashboard.putString("Camera Mode", "Camera");
		}else if(getCamMode() == 1) {
			getLimetable().getEntry("camMode").setDouble(0);
			SmartDashboard.putString("Camera Mode", "Vision");
		}
	}
	
	
	//Set the pipeline
	public void setPipeline(double pipeline) {
		getLimetable().getEntry("pipeline").setDouble(pipeline);
		SmartDashboard.putNumber("Pipeline: ", pipeline);
	}
}