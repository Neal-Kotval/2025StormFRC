// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
// import com.ctre.phoenix6.Utils;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
// import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Arm.MoveArm;
import frc.robot.commands.Elevator.MoveElevator;
import frc.robot.commands.Intake.MoveIntake;
import frc.robot.commands.Swerve.AlignCommand;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SysId.SwerveDriveSysId;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    private Vision m_Vision = new Vision();
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);

    public final Swerve drivetrain = TunerConstants.createDrivetrain();
    private final SendableChooser<Command> autoChooser;
    private final SwerveDriveSysId m_swerveSysId;
    private final Arm arm = new Arm();
    private final Elevator elevator = new Elevator();
    private final Intake intake = new Intake();

    //Operator
    public Trigger operatorY = new Trigger(joystick2.y());
    public Trigger operatorX = new Trigger(joystick2.x());
    public Trigger operatorA = new Trigger(joystick2.a());
    public Trigger operatorB = new Trigger(joystick2.b());
    public Trigger padUp = new Trigger(joystick2.povUp());
    public Trigger padDown = new Trigger(joystick2.povDown());
    public Trigger padLeft = new Trigger(joystick2.povLeft());
    public Trigger padRight = new Trigger(joystick2.povRight());
    public Trigger leftYAxisActiveUp = new Trigger(()->(joystick2.getLeftY()>0.1));
    public Trigger leftYAxisActiveDown = new Trigger(()->(joystick2.getLeftY()<-0.1));
    public Trigger rightYAxisActiveUp = new Trigger(()->(joystick2.getRightY()>0.1));
    public Trigger rightYAxisActiveDown = new Trigger(()->(joystick2.getRightY()<-0.1));
    public Trigger leftBumper = new Trigger(joystick2.leftBumper());
    public Trigger rightBumper = new Trigger(joystick2.rightBumper());
    public Trigger rightTrigger = new Trigger(()->(joystick2.getRightTriggerAxis()>0.1));
    public Trigger leftTrigger = new Trigger(()->(joystick2.getLeftTriggerAxis()>0.1));

    public RobotContainer() {

        // NamedCommands.registerCommand("MoveArm", new MoveArm(0.1));

        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        m_swerveSysId = new SwerveDriveSysId(drivetrain);

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();

        addSysIdDebugDebugTab();
    }

    
    

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.b().whileTrue(drivetrain.createDriveToPose(new Pose2d(new Translation2d(0,0), new Rotation2d(0))));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));
        // if (Utils.isSimulation()) {
        //     drivetrain.resetPose(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
        // }
        joystick.y().whileTrue(new AlignCommand(drivetrain, m_Vision,0));
        drivetrain.registerTelemetry(logger::telemeterize);

        leftYAxisActiveDown.whileTrue(new MoveArm(arm, -0.05));
        leftYAxisActiveDown.whileTrue(new MoveArm(arm, -0.05));

        operatorA.whileTrue(new MoveElevator(elevator, 0.05));
        operatorB.whileTrue(new MoveIntake(intake, 0.05));
    }

    public Command getAutonomousCommand() {
        return autoChooser.getSelected();
    }

    private void addSysIdDebugDebugTab() {
        ShuffleboardTab debugTab = Shuffleboard.getTab("Sys Id debug");
        debugTab.add(m_swerveSysId.createTranslationSysIdCommand().withName("Translation sysid"));
        debugTab.add(m_swerveSysId.createSteerSysIdCommand().withName("Steer sysid"));
        debugTab.add(m_swerveSysId.createRotationSysIdCommand().withName("Rotation sysid"));

    }
}
