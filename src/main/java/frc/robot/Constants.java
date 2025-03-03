// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

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

   public static final class Swerve {
        public static final String swerveCanBus = "storm";

        public static final COTSTalonFXSwerveConstants chosenModule =  
            COTSTalonFXSwerveConstants.SDS.MK4i.KrakenX60(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.75);
        /* Center to Center distance of left and right modules in meters. */
        public static final double wheelBase = Units.inchesToMeters(24.75);
        /* Center to Center distance of front and rear module wheels in meters. */
        public static final double wheelCircumference = chosenModule.wheelCircumference * 0.97845; // testing

        /* Swerve Kinematics 
         * No need to ever change this unless you are not doing a traditional rectangular/square 4 module swerve */
         public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
            new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
            new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 25;
        public static final int angleCurrentThreshold = 40;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 40; 
        public static final int driveCurrentThreshold = 60;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /* These values are used by the drive falcon to ramp in open loop and closed loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.10; //TODO: This must be tuned to specific robot
        /* After completeing characterization and inserting 
         * the KS, KV, and KA values into the code, tune the 
         * drive motor kP until it doesn't overshoot and 
         * doesnt oscilate around a target velocity. */
        public static final double driveKI = 0.0; //Leave driveKI at 0.0
        public static final double driveKD = 0.0; //Leave driveKD at 0.0
        public static final double driveKF = 0.0; //Leave driveKF at 0.0 

        /* Drive Motor Characterization Values From SYSID */ 
        public static final double driveKS = 0.32; 
        public static final double driveKV = 1.51; 
        public static final double driveKA = 0.27;  

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = isRocky ? 4.572 : 5.21208; 
        /* These are theorectial values to start with, tune after
         * Kraken FOC (L1.0): ft/s = 12.4 | m/s = 3.77952
         * Kraken FOC (L1.5): ft/s = 14.2 | m/s = 4.32816
         * Kraken FOC (L2.0): ft/s = 15.0 | m/s = 4.572
         * Kraken FOC (L2.5): ft/s = 17.1 | m/s = 5.21208
         * Kraken FOC (L3.0): ft/s = 16.5 | m/s = 5.0292
         * Kraken FOC (L3.5): ft/s = 18.9 | m/s = 5.76072
         */
        /** Radians per Second */
        // public static final double maxAngularVelocity = 10.0; //TODO: This must be tuned to specific robot
        public static final double driveRadius = Math.hypot(wheelBase, trackWidth) / 2.0;
        public static final double maxAngularVelocity = maxSpeed / driveRadius;

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Brake;

        /* Module Specific Constants */
        public static final double maxAngleError = 2.0; // Degrees before we alert that the module is not aligned

        /* Front Left Module - Module 0 */
        public static final class Mod0 { 
            public static final int driveMotorID = 0;
            public static final int angleMotorID = 1;
            public static final int canCoderID = 0;
            public static final String canBusID = swerveCanBus;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(isRocky ? -138.33 : 33.9 + 180.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, canBusID, angleOffset);
        }

        /* Front Right Module - Module 1 */
        public static final class Mod1 { 
            public static final int driveMotorID = 18;
            public static final int angleMotorID = 19;
            public static final int canCoderID = 1;
            public static final String canBusID = swerveCanBus;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(isRocky ? 148.00 : -72.2 + 180.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, canBusID, angleOffset);
        }
        
        /* Back Left Module - Module 2 */
        public static final class Mod2 { 
            public static final int driveMotorID = 8;
            public static final int angleMotorID = 9;
            public static final int canCoderID = 2;
            public static final String canBusID = swerveCanBus;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(isRocky ? 61.61 : 162.6 + 180.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, canBusID, angleOffset);
        }

        /* Back Right Module - Module 3 */
        public static final class Mod3 { 
            public static final int driveMotorID = 10;
            public static final int angleMotorID = 11;
            public static final int canCoderID = 3;
            public static final String canBusID = swerveCanBus;
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(isRocky ? -27.24 : -25.4 + 180.0);
            public static final SwerveModuleConstants constants = 
                new SwerveModuleConstants(driveMotorID, angleMotorID, canCoderID, canBusID, angleOffset);
        }
    }
}
