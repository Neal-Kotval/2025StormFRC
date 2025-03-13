package frc.robot.commands.Swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveModule.SteerRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.Vision;

import frc.robot.generated.TunerConstants;

public class SimpleAign extends Command {
    private final Vision m_Vision;
    private final Swerve m_Swerve;

    private static final double xTol = 1; // Proportional gain for aiming
    private static final double yTol = 1; // Proportional gain for ranging

    public AlignCommand(Vision vision, Swerve swerve, double targetDistance, double targetAngle) {
        m_Vision = vision;
        m_Swerve = swerve;
        m_alignRequest = new SwerveRequest.RobotCentric()
            .withDeadband(0.1)
            .withRotationalDeadband(0.1)
            .withDriveRequestType(DriveRequestType.Velocity)
            .withSteerRequestType(SteerRequestType.MotionMagicExpo);
    }

    @Override
    public void initialize() {
        System.out.println("AlignCommand initialized");
        lostDetectionTimer.reset();
        lostDetectionTimer.start();
    }

    @Override
    public void execute() {
        double currentTargetTX = m_Vision.getTargetTX();
        double currentTargetTY = m_Vision.getTargetTY();
        double currentTargetAngle = m_Vision.getTargetAngle();

        
    }

    @Override
    public boolean isFinished() {
        double iT = Timer.getFPGATimestamp();
        while (m_Vision.getTX() == m_Vision.getTX() || m_Vision.getTY() == m_Vision.getTY()) {
            if (Timer.getFPGATimestamp() - iT > 0.2) {
                return true;
            }
        }
        return (m_Vision.getTargetTX() < xTol && m_Vision.getTargetTY() < yTol);
    }

    @Override
    public void end(boolean interrupted) {
        System.out.println("AlignCommand ended");
        m_Swerve.setControl(
            m_alignRequest
                .withVelocityX(0)
                .withVelocityY(0)
                .withRotationalRate(0)
        );
        lostDetectionTimer.stop();
    }

    // Simple proportional turning control with Limelight
    double limelight_aim_proportional(double angleError) {
        double targetingAngularVelocity = angleError * kP_aim;
        System.out.println("Calculated targetingAngularVelocity: " + targetingAngularVelocity);
        targetingAngularVelocity *= MaxAngularRate;
        targetingAngularVelocity *= -1.0;
        return targetingAngularVelocity;
    }

    // Simple proportional ranging control with Limelight's "ty" value
    double limelight_range_proportional(double distanceError) {
        double targetingForwardSpeed = distanceError * kP_range;
        System.out.println("Calculated targetingForwardSpeed: " + targetingForwardSpeed);
        targetingForwardSpeed *= MaxSpeed;
        targetingForwardSpeed *= -1.0;
        return targetingForwardSpeed;
    }
}