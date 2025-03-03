// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import java.util.function.Function;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.function.Supplier;

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

  public static NetworkTableInstance NT_INSTANCE = NetworkTableInstance.getDefault();
  public static Supplier<DriverStation.Alliance> ALLIANCE_SUPPLIER = () -> DriverStation.getAlliance().orElse(DriverStation.Alliance.Blue);;
  public static Distance ROBOT_LENGTH_WITH_BUMPERS = Inches.of(30);
  public static Distance ROBOT_WIDTH_WITH_BUMPERS = Inches.of(28);
  
  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
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
    public static final int armMotor = 2;
    public static final int intakeMotor = 0;
    public static final int elevatorRightMotor = 0;
    public static final int elevatorLeftMotor = 0;
    public static final int gyro = 0;
  }

  public static class TickValues {
    public static final int armSafetyTicks = 0;
  }

   public static final class VisionConstantsTest {
        public static Matrix<N3, N1> ODOM_STD_DEV = VecBuilder.fill(0.01, 0.01, Units.degreesToRadians(0.01));
        public static Function<Double, Matrix<N3, N1>> VISION_STD_DEV_MULTITAG_FUNCTION = dist -> dist < 3.0
                        ? VecBuilder.fill(Math.min(0.1, 0.1 * dist), Math.min(0.1, 0.1 * dist), Units.degreesToRadians(1.0))
                        : VecBuilder.fill(0.15 * dist, 0.15 * dist, Units.degreesToRadians(180.0) * dist);;
        public static Function<Double, Matrix<N3, N1>> VISION_STD_DEV_FUNCTION = dist -> dist < 3.0
                        ? VecBuilder.fill(0.15 * dist, 0.15 * dist, Units.degreesToRadians(10.0) * dist)
                        : VecBuilder.fill(0.5 * dist, 0.5 * dist, Units.degreesToRadians(180.0) * dist);;

        public static final class LimelightConstants {
            public static String LIMELIGHT_NT_NAME = "Limelight";

            public static double LL_FORWARD = 0.0;
            public static double LL_RIGHT = 0.0;
            public static double LL_UP = 0.0;
            public static double LL_ROLL = 0.0;
            public static double LL_PITCH = 0.0;
            public static double LL_YAW = 0.0;

            public static double LL_MAX_TAG_CLEAR_DIST = 4.0;
        }
    }
}
