package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.VisionConstants;
import frc.robot.LimelightHelpers.*;
import frc.robot.LimelightHelpers;

public class Vision extends SubsystemBase {
  private RawFiducial[] fiducials;

  public Vision() {
    config();
  }

  public static class NoSuchTargetException extends RuntimeException {
    public NoSuchTargetException(String message) {
      super(message);
    }
  }

  //Configurate Limelight Position
  public void config() {
    LimelightHelpers.setPipelineIndex("limelight", 0);
    int[] validIDs = {6,7,8,9,10,11,17,18,19,20,21,22};
    LimelightHelpers.SetFiducialIDFiltersOverride("limelight", validIDs);
  }

  @Override
  public void periodic() {
    fiducials = LimelightHelpers.getRawFiducials("limelight");

  }
  public RawFiducial getClosestFiducial() {
    if (fiducials == null || fiducials.length == 0) {
        throw new NoSuchTargetException("No fiducials found.");
    }

    RawFiducial closest = fiducials[0];
    double minDistance = closest.ta;

    for (RawFiducial fiducial : fiducials) {
        if (fiducial.ta > minDistance) {
            closest = fiducial;
            minDistance = fiducial.ta;
        }
    }

    return closest;
  }

  public RawFiducial getFiducialWithId(int id) {
    fiducials = LimelightHelpers.getRawFiducials("limelight");
    for (RawFiducial fiducial : fiducials) {
        if (fiducial.id == id) {
            return fiducial;
        }
    }
    throw new NoSuchTargetException("Can't find ID: " + id);
  }

public RawFiducial getFiducialWithId(int id, boolean verbose) {
  StringBuilder availableIds = new StringBuilder();

  for (RawFiducial fiducial : fiducials) {
      if (availableIds.length() > 0) {
          availableIds.append(", ");
      } //Error reporting
      availableIds.append(fiducial.id);
      
      if (fiducial.id == id) {
          return fiducial;
      }
  }
  throw new NoSuchTargetException("Cannot find: " + id + ". In view:: " + availableIds.toString());
  }

  public double getTX(){
    return LimelightHelpers.getTX(VisionConstants.LIMELIGHT_NAME);
  }
  public double getTY(){
    return LimelightHelpers.getTY(VisionConstants.LIMELIGHT_NAME);
  }
  public double getTA(){
    return LimelightHelpers.getTA(VisionConstants.LIMELIGHT_NAME);
  }
  public boolean getTV(){
    return LimelightHelpers.getTV(VisionConstants.LIMELIGHT_NAME);
  }
    public double getTargetAngle() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
    }

    public double getTargetTY() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0.0);
    }

    public double getTargetTX() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0.0);
    }

    public double[] get3DPose() {
        return NetworkTableInstance.getDefault().getTable("limelight").getEntry("camtran").getDoubleArray(new double[6]);
    }
  public double getClosestTX(){
    return getClosestFiducial().txnc;
  }
  public double getClosestTY(){
    return getClosestFiducial().tync;
  }
  public double getClosestTA(){
    return getClosestFiducial().ta;
  }
}