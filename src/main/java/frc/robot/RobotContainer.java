// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;
// import com.ctre.phoenix6.Utils;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;


import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
// import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import frc.robot.commands.Arm.*;
import frc.robot.commands.Elevator.CurrentSetter;
import frc.robot.commands.Elevator.ElevatorSetPosition;
import frc.robot.commands.Elevator.MoveElevator;
import frc.robot.commands.Intake.IntakeUntilDetected;
import frc.robot.commands.Intake.MoveIntake;
import frc.robot.commands.Intake.TimedIntake;
import frc.robot.commands.Swerve.AlignToReefTagRelative;
import frc.robot.commands.Swerve.AlignToSource;
import frc.robot.commands.Swerve.TimedSwerve;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.*;
import frc.robot.subsystems.SysId.SwerveDriveSysId;
import frc.robot.commands.Swerve.SeekUnseen;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.1).withRotationalDeadband(MaxAngularRate * 0.1) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();

    private final Telemetry logger = new Telemetry(MaxSpeed);
    private final SendableChooser<Command> autoChooser;
    private final SwerveDriveSysId m_swerveSysId;
    private static int currentMode;

    private final CommandXboxController joystick = new CommandXboxController(0);
    private final CommandXboxController joystick2 = new CommandXboxController(1);

    public final Swerve m_swerve = Constants.Subsystems.S_SWERVE;
    private final Arm m_arm = Constants.Subsystems.S_ARM;
    private final Elevator m_elevator = new Elevator();
    private final Intake m_intake = Constants.Subsystems.S_INTAKE;
    private final Sheet m_sheet = Constants.Subsystems.S_SHEET;
    private final Vision m_vision = Constants.Subsystems.S_VISION;

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
        NamedCommands.registerCommand("setL4", new ElevatorSetPosition(m_elevator, m_arm, Constants.TickValues.L3ElevatorTicks));
        //NamedCommands.registerCommand("poseEstimate", new InstantCommand(()->drivetrain.setTranslationToVision()));
        NamedCommands.registerCommand("AlignCoral", new TimedSwerve(m_swerve, 2.5, 0.1, 0.1));
        //NamedCommands.registerCommand("driveToB1",  drivetrain.createDriveToPose(7.960,6.608,-135.000));
        NamedCommands.registerCommand("setL3", new SequentialCommandGroup(new ElevatorSetPosition(m_elevator, m_arm, Constants.TickValues.L3ElevatorTicks), new ArmSetPosition(m_elevator, m_arm, 7)));
        NamedCommands.registerCommand("TimedIntake", new TimedIntake(m_intake,5,-0.5));
        NamedCommands.registerCommand("VisionCoral",new SequentialCommandGroup(
            new AlignToReefTagRelative(true, m_swerve),
            //new TimedSwerve(drivetrain, 0.3, 1, 0),
            new SequentialCommandGroup(
                new ArmSetPosition(m_elevator, m_arm, Constants.TickValues.armSafetyTicks),
                new ElevatorSetPosition(m_elevator, m_arm, Constants.TickValues.L3ElevatorTicks), 
                new ArmSetPosition(m_elevator, m_arm, Constants.TickValues.L3ArmTicks)
            ),
            new TimedIntake(m_intake, 0.5, -0.7)
        ));
        NamedCommands.registerCommand("Retreat", new SequentialCommandGroup(
            new ArmSetPosition(m_elevator, m_arm, Constants.TickValues.armSafetyTicks),
            new SequentialCommandGroup(
            new ElevatorSetPosition(m_elevator, m_arm, 0), 
            new ArmSetPosition(m_elevator, m_arm, 0))
        ));
        NamedCommands.registerCommand("IntakeUntilDetected", new IntakeUntilDetected(m_intake, -0.7));
        currentMode = 4;



        //NamedCommands.registerCommand("ResetPose LeftCoral", drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));


        // Build an auto chooser. This will use Commands.none() as the default option.
        autoChooser = AutoBuilder.buildAutoChooser();
        m_swerveSysId = new SwerveDriveSysId(m_swerve);

        // Another option that allows you to specify the default auto by its name
        // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

        SmartDashboard.putData("Auto Chooser", autoChooser);

        configureBindings();

        addSysIdDebugDebugTab();
    }

    
    

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        m_swerve.setDefaultCommand(
            // Drivetrain will execute this command periodically
            m_swerve.applyRequest(() ->
                drive.withVelocityX(-joystick.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(-joystick.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(joystick.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
            )
        );

        //elevator.setDefaultCommand(new CurrentSetter(elevator));

        // joystick.a().whileTrue(drivetrain.applyRequest(() -> brake));
        joystick.a().whileTrue(new AlignToSource(false, m_swerve));
        // joystick.b().whileTrue(drivetrain.applyRequest(() ->
        //     point.withModuleDirection(new Rotation2d(-joystick.getLeftY(), -joystick.getLeftX()))
        // ));
        joystick.rightBumper().onTrue(new SeekUnseen(m_swerve));

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        joystick.back().and(joystick.y()).whileTrue(m_swerve.sysIdDynamic(Direction.kForward));
        joystick.back().and(joystick.x()).whileTrue(m_swerve.sysIdDynamic(Direction.kReverse));
        joystick.start().and(joystick.y()).whileTrue(m_swerve.sysIdQuasistatic(Direction.kForward));
        joystick.start().and(joystick.x()).whileTrue(m_swerve.sysIdQuasistatic(Direction.kReverse));

        //joystick.b().whileTrue(drivetrain.createDriveToPose(new Pose2d(new Translation2d(0,0), new Rotation2d(0))));

        // reset the field-centric heading on left bumper press
        joystick.leftBumper().onTrue(m_swerve.runOnce(() -> m_swerve.seedFieldCentric()));
        // if (Utils.isSimulation()) {
        //     drivetrain.resetPose(new Pose2d(new Translation2d(), Rotation2d.fromDegrees(0)));
        // }

        joystick.povLeft().onTrue(new SequentialCommandGroup(
            new AlignToReefTagRelative(false, m_swerve),
            new TimedSwerve(m_swerve, 0.3, 1, 0),
            currentAction(getCurrentMode()),
            new TimedIntake(m_intake, 0.5, -0.7)
        ));
        
        joystick.povRight().onTrue(new SequentialCommandGroup(
            new AlignToReefTagRelative(true, m_swerve),
            new TimedSwerve(m_swerve, 0.3, 1, 0),
            currentAction(getCurrentMode()),
            new TimedIntake(m_intake, 0.5, -0.7)
        ));

        m_swerve.registerTelemetry(logger::telemeterize);

        leftYAxisActiveDown.whileTrue(new MoveArm(m_arm, -0.08));
        leftYAxisActiveUp.whileTrue(new MoveArm(m_arm, 0.08));
        padLeft.onTrue(new IntakeUntilDetected(m_intake, -0.5));

        padUp.whileTrue(new MoveElevator(m_elevator, m_arm, 0.15));
        padDown.whileTrue(new MoveElevator(m_elevator, m_arm, -0.15));

        rightTrigger.whileTrue(new MoveIntake(m_intake, 0.4));
        

        // Outtake + Intake
        rightBumper.whileTrue(new MoveIntake(m_intake, -0.1));
        leftBumper.whileTrue(new MoveIntake(m_intake, 0.1));
        rightTrigger.whileTrue(new MoveIntake(m_intake, -0.5));
        leftTrigger.whileTrue(new MoveIntake(m_intake,0.5));
        

        // Set Positions (Elevator)
        // operatorA.onTrue(new ElevatorSetPosition(elevator, arm, Constants.TickValues.L1ElevatorTicks));

        // operatorY.onTrue(new InstantCommand(() -> {currentMode = 4;}));
        // operatorB.onTrue(new InstantCommand(() -> {currentMode = 3;}));
        // operatorX.onTrue(new InstantCommand(() -> {currentMode = 2;}));
        operatorB.onTrue(new InstantCommand(()->{m_sheet.setSheetSpeed(0.1);}));

        // operatorX.onTrue(new ElevatorSetPosition(elevator, arm, Constants.TickValues.L2ElevatorTicks));
        // operatorY.onTrue(new SequentialCommandGroup(
        //     new ElevatorSetPosition(elevator, arm, Constants.TickValues.L3ElevatorTicks), 
        //     new ArmSetPosition(elevator, arm, 7)
        // ));
        operatorA.onTrue(new SequentialCommandGroup(
            new ElevatorSetPosition(m_elevator, m_arm, 0), 
            new ArmSetPosition(m_elevator, m_arm, 0)
        ));
        

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


    public SequentialCommandGroup currentAction(int mode) {
        System.out.println(mode);
        if (mode == 4) {
            return new SequentialCommandGroup(
                new ArmSetPosition(m_elevator, m_arm, Constants.TickValues.armSafetyTicks),
                new ElevatorSetPosition(m_elevator, m_arm, Constants.TickValues.L3ElevatorTicks), 
                new ArmSetPosition(m_elevator, m_arm, Constants.TickValues.L3ArmTicks)
            );
        }
        
        if (mode == 3) {
            return new SequentialCommandGroup(
                new ArmSetPosition(m_elevator, m_arm, Constants.TickValues.armSafetyTicks),
                new ElevatorSetPosition(m_elevator, m_arm, Constants.TickValues.L2ElevatorTicks) 
            );
        }
        
        if (mode == 2) {
            return new SequentialCommandGroup(
                new ArmSetPosition(m_elevator, m_arm, Constants.TickValues.armSafetyTicks),
                new ElevatorSetPosition(m_elevator, m_arm, Constants.TickValues.L1ElevatorTicks) 
            );
        }

        return new SequentialCommandGroup(
        );
    }

    public static int getCurrentMode() {
        return currentMode;
    }
}