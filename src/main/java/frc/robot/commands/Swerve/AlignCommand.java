package frc.robot.commands.Swerve;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.Swerve;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.LimelightHelpers.RawFiducial;
import edu.wpi.first.math.controller.PIDController;
import frc.robot.Constants.PIDvalues;;

class PIDControllerConfigurable extends PIDController {
  public PIDControllerConfigurable(double kP, double kI, double kD) {
      super(kP, kI, kD);
  }
  
  public PIDControllerConfigurable(double kP, double kI, double kD, double tolerance) {
      super(kP, kI, kD);
      this.setTolerance(tolerance);
  }
}
public class AlignCommand extends Command {
  private final Swerve m_drivetrain;
  private final VisionSubsystem m_Limelight;
  private final int tagID;

  private static final PIDControllerConfigurable rotationalPidController = new PIDControllerConfigurable(
    PIDvalues.rotationalKP,
    PIDvalues.rotationalKI,
    PIDvalues.rotationalKD,
    PIDvalues.rotationalTolerance
  );

  private static final PIDControllerConfigurable xPidController = new PIDControllerConfigurable(
    PIDvalues.XtranslationalKP,
    PIDvalues.XtranslationalKI,
    PIDvalues.XtranslationalKD,
    PIDvalues.XtranslationalTolerance
  );

  private static final PIDControllerConfigurable yPidController = new PIDControllerConfigurable(
    PIDvalues.YtranslationalKP,
    PIDvalues.YtranslationalKI,
    PIDvalues.YtranslationalKD,
    PIDvalues.YtranslationalTolerance
  );

  private static final SwerveRequest.RobotCentric alignRequest = new SwerveRequest.RobotCentric().withDriveRequestType(DriveRequestType.OpenLoopVoltage);
  private static final SwerveRequest.Idle idleRequest = new SwerveRequest.Idle();

  public AlignCommand(Swerve drivetrain, VisionSubsystem limelight, int TAGID) {
    this.m_drivetrain = drivetrain;
    this.m_Limelight = limelight;
    this.tagID = TAGID;
  }


  @Override
  public void initialize() {}

  @Override
  public void execute() {
    
    RawFiducial fiducial;
        
    try {
      fiducial = m_Limelight.getFiducialWithId(this.tagID);
      final double rotationalRate = rotationalPidController.calculate(2*fiducial.txnc, 0.0) * 0.75* 0.9;
      final double velocityX = xPidController.calculate(fiducial.distToRobot, 0.5) * TunerConstants.kSpeedAt12Volts.in(MetersPerSecond) * 0.7;
      if (rotationalPidController.atSetpoint() && xPidController.atSetpoint() && yPidController.atSetpoint()) {
        this.end(true);
      }

      m_drivetrain.setControl(
          alignRequest.withRotationalRate(-rotationalRate).withVelocityX(-velocityX));
 
    } catch (VisionSubsystem.NoSuchTargetException nste) {
    }
  }

  @Override
  public boolean isFinished() {
    return rotationalPidController.atSetpoint() && xPidController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrain.applyRequest(() -> idleRequest);
    
  }
}