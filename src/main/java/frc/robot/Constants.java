// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import com.ctre.phoenix6.configs.CANrangeConfiguration;
import com.ctre.phoenix6.signals.UpdateModeValue;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

// import edu.wpi.first.units.measure.*; 
/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }

  public static final class AutoConstants {
    public static final double X_REEF_ALIGNMENT_P = 3.3;
    public static final double Y_REEF_ALIGNMENT_P = 3.3;
    public static final double ROT_REEF_ALIGNMENT_P = 0.058;

    public static final double ROT_SETPOINT_REEF_ALIGNMENT = 0;  // Rotation
    public static final double ROT_TOLERANCE_REEF_ALIGNMENT = 1;

    public static final double X_SETPOINT_REEF_ALIGNMENT = -0;  // Vertical pose
    public static final double X_TOLERANCE_REEF_ALIGNMENT = 0.02;

    public static final double Y_SETPOINT_REEF_ALIGNMENT = 0.16+0.05;  // Horizontal pose
    public static final double Y_TOLERANCE_REEF_ALIGNMENT = 0.02;

    public static final double DONT_SEE_TAG_WAIT_TIME = 1;
    public static final double POSE_VALIDATION_TIME = 0.3;
  }

  public static class VisionConstants {
    public static final String LIMELIGHT_NAME = "limelight";
    public static final Distance LIMELIGHT_LENS_HEIGHT = Distance.ofBaseUnits(8, Inches);
    public static final Angle LIMELIGHT_ANGLE = Angle.ofBaseUnits(0, Degrees);

    public static final Distance REEF_APRILTAG_HEIGHT = Distance.ofBaseUnits(6.875, Inches);
    public static final Distance PROCCESSOR_APRILTAG_HEIGHT = Distance.ofBaseUnits(45.875, Inches);
    public static final Distance CORAL_APRILTAG_HEIGHT = Distance.ofBaseUnits(53.25, Inches);
  }

  public static class PIDvalues {
    public static final double rotationalKP = 0.5;
    public static final double rotationalKI = 0;
    public static final double rotationalKD = 0.5;
    public static final double rotationalTolerance = 0.005;

    public static final double XtranslationalKP = 0.1;
    public static final double XtranslationalKI = 0;
    public static final double XtranslationalKD = 0;
    public static final double XtranslationalTolerance = 0.01;

    public static final double YtranslationalKP = 0.1;
    public static final double YtranslationalKI = 0;
    public static final double YtranslationalKD = 0;
    public static final double YtranslationalTolerance = 0.01;
  }

  public static class CANids {
    public static final int armMotor = 10;
    public static final int intakeMotor = 6;
    public static final int elevatorRightMotor = 4;
    public static final int elevatorLeftMotor = 5;
    public static final int gyro = 0;
    public static final int coralSensor = 25;
  }

  public static class PWMids {
    public static final int sheet = 0;
  }

  public static class TickValues {
    public static final double armSafetyTicks = 4.55;
    //public static final double elevatorSafetyTicks = 11.77;
    public static final double L3ArmTicks = 11.77;
    public static final double L1ElevatorTicks = 9.5;
    public static final double L2ElevatorTicks = 22.40;
    public static final double L3ElevatorTicks = 42.36;
  }
  public static class ElevatorConstants {
    public static final double kP = 2;
    public static final double kI = 0;
    public static final double kD = 0.1;
    public static final double kG = 0;
    public static final double kMaxRotations = 44.36;
  }

  public static CANrangeConfiguration coralSensorConfig = new CANrangeConfiguration();
  static {
    coralSensorConfig.ToFParams.UpdateMode = UpdateModeValue.ShortRange100Hz;
    coralSensorConfig.ProximityParams.ProximityThreshold = 0.1;
  }
}
