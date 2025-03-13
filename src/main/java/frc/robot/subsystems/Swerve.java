package frc.robot.subsystems;

import static edu.wpi.first.units.Units.*;

import java.util.Arrays;
import java.util.function.Supplier;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.ElevatorConstants;
import frc.robot.generated.TunerConstants.TunerSwerveDrivetrain;


import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.commands.PathPlannerAuto;
// import com.pathplanner.lib.path.PathConstraints;
// import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import frc.robot.LimelightHelpers.PoseEstimate;
import edu.wpi.first.wpilibj.Timer;

/**
 * Class that extends the Phoenix 6 SwerveDrivetrain class and implements
 * Subsystem so it can easily be used in command-based projects.
 */
public class Swerve extends TunerSwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;
    RobotConfig config;

    /* Blue alliance sees forward as 0 degrees (toward red alliance wall) */
    private static final Rotation2d kBlueAlliancePerspectiveRotation = Rotation2d.kZero;
    /* Red alliance sees forward as 180 degrees (toward blue alliance wall) */
    private static final Rotation2d kRedAlliancePerspectiveRotation = Rotation2d.k180deg;
    /* Keep track if we've ever applied the operator perspective before or not */
    private boolean m_hasAppliedOperatorPerspective = false;

    /* Swerve requests to apply during SysId characterization */
    private final SwerveRequest.SysIdSwerveTranslation m_translationCharacterization = new SwerveRequest.SysIdSwerveTranslation();
    private final SwerveRequest.SysIdSwerveSteerGains m_steerCharacterization = new SwerveRequest.SysIdSwerveSteerGains();
    private final SwerveRequest.SysIdSwerveRotation m_rotationCharacterization = new SwerveRequest.SysIdSwerveRotation();
    public final Translation2d zeroPose = new Translation2d(0,0);

    private final SwerveRequest.ApplyRobotSpeeds AutoRequest = new SwerveRequest.ApplyRobotSpeeds();
    public final Pigeon2 m_gyro = new Pigeon2(0);
    public final Elevator m_elevator = new Elevator();

    // Locations for the swerve drive modules relative to the robot center.
    Translation2d m_frontLeftLocation = new Translation2d(Units.inchesToMeters(15), Units.inchesToMeters(14));
    Translation2d m_frontRightLocation = new Translation2d(Units.inchesToMeters(15), -Units.inchesToMeters(14));
    Translation2d m_backLeftLocation = new Translation2d(-Units.inchesToMeters(15), Units.inchesToMeters(14));
    Translation2d m_backRightLocation = new Translation2d(-Units.inchesToMeters(15), -Units.inchesToMeters(14));
    // Creating my kinematics object using the module locations
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );


    SwerveDrivePoseEstimator m_odometry = new SwerveDrivePoseEstimator(
    m_kinematics, 
    m_gyro.getRotation2d(),
    this.getState().ModulePositions, 
    new Pose2d(new Translation2d(0, 0), new Rotation2d(0)),
    VecBuilder.fill(0.1, 0.1, 0.1),
    VecBuilder.fill(0.1, 0.1, 100));
    private void configurePathPlanner() {
        try{
            config = RobotConfig.fromGUISettings();
        AutoBuilder.configure(
            ()->this.getState().Pose, // Robot pose supplier
            this::resetPose, // Method to reset odometry (will be called if your auto has a starting pose)
            ()->this.getState().Speeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            (speeds, feedforwards) -> this.setControl(AutoRequest.withSpeeds(speeds)), // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(2, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(1, 0.0, 0.0) // Rotation PID constants
            ),
            config, // The robot configuration
            () -> DriverStation.getAlliance().orElse(Alliance.Blue) == Alliance.Red,
            this
        );
        } catch (Exception e) {
            // Handle exception as needed
                e.printStackTrace();
        }
    }

    /* SysId routine for characterizing translation. This is used to find PID gains for the drive motors. */
    private final SysIdRoutine m_sysIdRoutineTranslation = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(4), // Reduce dynamic step voltage to 4 V to prevent brownout
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdTranslation_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            output -> setControl(m_translationCharacterization.withVolts(output)),
            null,
            this
        )
    );

    /* SysId routine for characterizing steer. This is used to find PID gains for the steer motors. */
    private final SysIdRoutine m_sysIdRoutineSteer = new SysIdRoutine(
        new SysIdRoutine.Config(
            null,        // Use default ramp rate (1 V/s)
            Volts.of(7), // Use dynamic voltage of 7 V
            null,        // Use default timeout (10 s)
            // Log state with SignalLogger class
            state -> SignalLogger.writeString("SysIdSteer_State", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            volts -> setControl(m_steerCharacterization.withVolts(volts)),
            null,
            this
        )
    );

    /*
     * SysId routine for characterizing rotation.
     * This is used to find PID gains for the FieldCentricFacingAngle HeadingController.
     * See the documentation of SwerveRequest.SysIdSwerveRotation for info on importing the log to SysId.
     */
    // private final SysIdRoutine m_sysIdRoutineRotation = new SysIdRoutine(
    //     new SysIdRoutine.Config(
    //         /* This is in radians per second², but SysId only supports "volts per second" */
    //         Volts.of(Math.PI / 6).per(Second),
    //         /* This is in radians per second, but SysId only supports "volts" */
    //         Volts.of(Math.PI),
    //         null, // Use default timeout (10 s)
    //         // Log state with SignalLogger class
    //         state -> SignalLogger.writeString("SysIdRotation_State", state.toString())
    //     ),
    //     new SysIdRoutine.Mechanism(
    //         output -> {
    //             /* output is actually radians per second, but SysId only supports "volts" */
    //             setControl(m_rotationCharacterization.withRotationalRate(output.in(Volts)));
    //             /* also log the requested output for SysId */
    //             SignalLogger.writeDouble("Rotational_Rate", output.in(Volts));
    //         },
    //         null,
    //         this
    //     )
    // );

    /* The SysId routine to test */
    private SysIdRoutine m_sysIdRoutineToApply = m_sysIdRoutineTranslation;

    public Swerve(
        SwerveDrivetrainConstants drivetrainConstants,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configurePathPlanner();
    }

    public Swerve(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Swerve(
        SwerveDrivetrainConstants drivetrainConstants,
        double odometryUpdateFrequency,
        Matrix<N3, N1> odometryStandardDeviation,
        Matrix<N3, N1> visionStandardDeviation,
        SwerveModuleConstants<?, ?, ?>... modules
    ) {
        super(drivetrainConstants, odometryUpdateFrequency, odometryStandardDeviation, visionStandardDeviation, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
        configurePathPlanner();
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return m_sysIdRoutineToApply.dynamic(direction);
    }

    @Override
    public void periodic() {
        m_odometry.updateWithTime(Timer.getFPGATimestamp(), m_gyro.getRotation2d(), this.getState().ModulePositions);
        //m_odometry.addVisionMeasurement(getEasyPose(1, 3, 0), Timer.getFPGATimestamp()-0.02);

        if (!m_hasAppliedOperatorPerspective || DriverStation.isDisabled()) {
            DriverStation.getAlliance().ifPresent(allianceColor -> {
                setOperatorPerspectiveForward(
                    allianceColor == Alliance.Red
                        ? kRedAlliancePerspectiveRotation
                        : kBlueAlliancePerspectiveRotation
                );
                m_hasAppliedOperatorPerspective = true;
            });
        }

        SmartDashboard.putNumber("PoseX", this.getState().Pose.getX());
        SmartDashboard.putNumber("PoseY", this.getState().Pose.getY());
        SmartDashboard.putNumber("PoseRotation", this.getState().Pose.getRotation().getDegrees());
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }

    public Command createDriveToPose(Pose2d pose) {
        PathConstraints swerveConstraints = new PathConstraints(
            12,
            12,
            540.0,
            720.0,
            12.0,
            false
        );
        return AutoBuilder.pathfindToPose(pose, swerveConstraints, 0.0);
        
    }

    public Command driveToTag() {
        LimelightHelpers.setPipelineIndex("limelight", 0);

        int[] validIDs = {6,7,8,9,10,11,17,18,19,20,21,22};
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);

        boolean doRejectUpdate = false;
        LimelightHelpers.SetRobotOrientation("limelight", m_gyro.getYaw().getValueAsDouble(), 0,
                0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");
        if (Math.abs(m_gyro.getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore
                                            // vision updates
        {
            doRejectUpdate = true;
        }
        //System.out.print(Math.abs(m_gyro.getAngularVelocityZWorld().getValueAsDouble()));

        if (mt2.tagCount <= 0) {
            doRejectUpdate = true;
        }
        
        System.out.println("Updating, " + mt2.tagCount);
        if (!doRejectUpdate) {
            return createDriveToPose(mt2.pose);
        }
        return (createDriveToPose(this.getState().Pose));
    }

    public double swerveDampingFactor(double maxDamp) {
        double currTicks;
        currTicks = m_elevator.getTicks();
        if (currTicks < 0) {
            currTicks = 0;
        } else if (currTicks > ElevatorConstants.kMaxRotations) {
            currTicks = ElevatorConstants.kMaxRotations;
        }
        return (1 - (maxDamp/ElevatorConstants.kMaxRotations)*(m_elevator.getTicks() > ElevatorConstants.kMaxRotations ? ElevatorConstants.kMaxRotations : m_elevator.getTicks()));
    }

    public void drive(Translation2d translationalVelocities, double rotationalVelocity) {
        SwerveRequest m_request = new SwerveRequest.RobotCentric()
            .withVelocityX(translationalVelocities.getX())
            .withVelocityY(translationalVelocities.getY())
            .withRotationalRate(rotationalVelocity);
        this.setControl(m_request);
    }

    public void updateMegaTagOdometry() {
        boolean doRejectUpdate = false;
        LimelightHelpers.SetRobotOrientation("limelight", m_gyro.getYaw().getValueAsDouble(), 0,
                0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        if (mt2 == null) {
            return;
        }
        if (Math.abs(m_gyro.getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore
                                            // vision updates
        {
            doRejectUpdate = true;
        }
        //System.out.print(Math.abs(m_gyro.getAngularVelocityZWorld().getValueAsDouble()));

        if (mt2.tagCount <= 0) {
            doRejectUpdate = true;
        }
        
        if (!doRejectUpdate) {
            // odometry.setVisionMeasurementStdDevs(VecBuilder.fill(2,2,2.0*PoseConstants.kVisionStdDevTheta));
            // m_odometry.setVisionMeasurementStdDevs(VecBuilder.fill(0.7, 0.7, 9999999));
            m_odometry.addVisionMeasurement(
                    mt2.pose,
                    Utils.fpgaToCurrentTime(mt2.timestampSeconds));
            
            System.out.println("m_odometry, " + m_odometry.getEstimatedPosition().getX() + ", " + m_odometry.getEstimatedPosition().getY());
        }
    }

    public Pose2d fetchPoseM2() {
        LimelightHelpers.setPipelineIndex("limelight", 0);
    
        int[] validIDs = {6,7,8,9,10,11,17,18,19,20,21,22};
        LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);

        boolean doRejectUpdate = false;
        LimelightHelpers.SetRobotOrientation("limelight", m_gyro.getYaw().getValueAsDouble(), 0,
                0, 0, 0, 0);
        LimelightHelpers.PoseEstimate mt2 = LimelightHelpers.getBotPoseEstimate_wpiBlue_MegaTag2("limelight");

        if (mt2 == null) {
            return this.getState().Pose;
        }
        if (Math.abs(m_gyro.getAngularVelocityZWorld().getValueAsDouble()) > 720) // if our angular velocity is greater than 720 degrees per second, ignore
                                            // vision updates
        {
            doRejectUpdate = true;
        }

        if (mt2.tagCount <= 0) {
            doRejectUpdate = true;
        }
        
        if (!doRejectUpdate) {

            return mt2.pose;
        }

        return this.getState().Pose;
    }

    public Pose2d getEstimatedPose() {
        return m_odometry.getEstimatedPosition();
    }

    public Pose2d getEasyPose(double x, double y, double theta) {
        return new Pose2d(new Translation2d(x, y), new Rotation2d(Math.toRadians(theta)));
    }
}