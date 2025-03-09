// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Arm.*;
import frc.robot.commands.Elevator.ElevatorSetPosition;
import frc.robot.commands.Elevator.MoveElevator;
import frc.robot.commands.Intake.IntakeUntilDetected;
import frc.robot.commands.Intake.MoveIntake;
import frc.robot.commands.Intake.TimedIntake;
import frc.robot.commands.Swerve.AlignCommand;
import frc.robot.commands.Swerve.TimedSwerve;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.Swerve.Swerve;
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

        NamedCommands.registerCommand("setL4", new ElevatorSetPosition(elevator, arm, Constants.TickValues.L3ElevatorTicks));
        NamedCommands.registerCommand("poseEstimate", new InstantCommand(()->drivetrain.setTranslationToVision()));
        NamedCommands.registerCommand("driveToTag", new AlignCommand(m_Vision, drivetrain, 0.35, 0));
        NamedCommands.registerCommand("AlignCoral", new TimedSwerve(drivetrain, 2.5, 0.1, 0.1));
        NamedCommands.registerCommand("intakeUntil", new IntakeUntilDetected(intake, -0.2));
        //NamedCommands.registerCommand("driveToB1",  drivetrain.createDriveToPose(7.960,6.608,-135.000));
        NamedCommands.registerCommand("TimedIntake", new TimedIntake(intake,2,-0.5));



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
                drive.withVelocityX(-joystick.getLeftY() * drivetrain.swerveDampingFactor(0.5) * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * drivetrain.swerveDampingFactor(0.5) * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(joystick.getRightX() * drivetrain.swerveDampingFactor(0.5) * MaxSpeed) // Drive counterclockwise with negative X (left)
            )
        );

        joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        joystick.leftBumper().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        joystick.y().onTrue(drivetrain.driveToTag());
        joystick.povRight().onTrue(new TimedSwerve(drivetrain, 3.5, 0, 0.1));
        joystick.povLeft().onTrue(new TimedSwerve(drivetrain, 3.5, 0, -0.1));

        drivetrain.registerTelemetry(logger::telemeterize);

        leftYAxisActiveDown.whileTrue(new MoveArm(arm, -0.1));
        leftYAxisActiveUp.whileTrue(new MoveArm(arm, 0.1));

        padUp.whileTrue(new MoveElevator(elevator, arm, 0.1));
        padDown.whileTrue(new MoveElevator(elevator, arm, -0.1));

        // Outtake + Intake
        leftBumper.whileTrue(new MoveIntake(intake, 0.08));
        rightBumper.whileTrue(new MoveIntake(intake, -0.5));

        // Set Positions (Elevator)
        operatorA.onTrue(new ElevatorSetPosition(elevator, arm, Constants.TickValues.L1ElevatorTicks));
        operatorB.onTrue(new ElevatorSetPosition(elevator, arm, Constants.TickValues.L2ElevatorTicks));
        operatorY.onTrue(new SequentialCommandGroup(new ElevatorSetPosition(elevator, arm, Constants.TickValues.L3ElevatorTicks), new ArmSetPosition(elevator, arm, 6.22)));
        operatorX.onTrue(new SequentialCommandGroup(new ElevatorSetPosition(elevator, arm, 0).withTimeout(2), new setArmPositionNeutral(arm, elevator)));

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
