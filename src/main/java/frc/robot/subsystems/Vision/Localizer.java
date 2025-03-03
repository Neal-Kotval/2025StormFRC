package frc.robot.subsystems.Vision;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.subsystems.Swerve;
import frc.lib.util.*;
import frc.lib.util.Vision.LimelightUtil;


import java.util.Optional;

public class Localizer {
    private enum LocalizationStrategy {
        POSE_ESTIMATOR,
    }

    // localizer is a dependent of swerve
    private final Swerve swerve;
    // private final LocalizationTelemetry localizationTelemetry = new LocalizationTelemetry(this);
    private final SendableChooser<LocalizationStrategy> localizationChooser = new SendableChooser<>();

    private final SwerveDrivePoseEstimator poseEstimator;

    // The pose extrapolation method that the robot will use. It will be set to QuestNav by default.
    private LocalizationStrategy strategy = LocalizationStrategy.POSE_ESTIMATOR;

    private boolean hasCalibratedOnceWhenNear = false;

    public Localizer(Swerve swerve) {
        this.swerve = swerve;

        localizationChooser.setDefaultOption("Pose Estimator", LocalizationStrategy.POSE_ESTIMATOR);
        SmartDashboard.putData("Localization Strategy Chooser", localizationChooser);

        poseEstimator = new SwerveDrivePoseEstimator(
                this.swerve.getKinematics(),
                this.swerve.getState().RawHeading,
                this.swerve.getState().ModulePositions,
                this.swerve.getState().Pose,
                Constants.VisionConstantsTest.ODOM_STD_DEV,
                Constants.VisionConstantsTest.VISION_STD_DEV_MULTITAG_FUNCTION.apply(1.0)
        );

        // configureQuestOffset();
        LimelightUtil.configureRobotToCameraOffset();
    }

    public Pose2d getStrategyPose() {
        return getEstimatedPose();
    }

    public String getLocalizationStrategy() {
        return "Pose Estimator";
    }

    public boolean hasCalibratedOnceWhenNear() {
        return hasCalibratedOnceWhenNear;
    }

    public Pose2d getEstimatedPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public Translation2d getTranslationToNearestCoralStation() {
        Pose2d currentPose = getStrategyPose();
        Translation2d tagTranslation = FieldUtil.CoralStation.getNearestCoralStationTagPose(currentPose).getTranslation();
        return tagTranslation.minus(currentPose.getTranslation());
    }

    public double getAngleToNearestCoralStation() {
        return getTranslationToNearestCoralStation().getAngle().getDegrees();
    }

    public double getNearestCoralStationHeading() {
        return FieldUtil.CoralStation.getNearestCoralStationTagPose(getStrategyPose()).getRotation().getDegrees();
    }

    public Translation2d getTranslationToNearestReefSide() {
        Pose2d currentPose = getStrategyPose();
        Translation2d tagTranslation = FieldUtil.Reef.getNearestReefTagPose(currentPose).getTranslation();
        return tagTranslation.minus(currentPose.getTranslation());
    }

    public double getAngleToNearestReefSide() {
        return getTranslationToNearestReefSide().getAngle().getDegrees();
    }

    public double getNearestReefSideHeading() {
        return FieldUtil.Reef.getNearestReefTagPose(getStrategyPose()).getRotation().rotateBy(Rotation2d.kPi).getDegrees();
    }

    public Translation2d getTranslationToNearestBranch() {
        Pose2d currentPose = getStrategyPose();
        Translation2d nearestBranch = FieldUtil.Reef.getNearestBranchPose(currentPose).getTranslation();
        return nearestBranch.minus(currentPose.getTranslation());
    }

    public double getAngleToNearestBranch() {
        return getTranslationToNearestBranch().getAngle().getDegrees();
    }

    public boolean atBranch() {
        return getStrategyPose().getTranslation().getDistance(
                FieldUtil.Reef.getNearestRobotPoseAtBranch(getStrategyPose()).getTranslation()
        ) < Constants.AutoConstants.TRANSLATION_TOLERANCE_TO_ACCEPT;
    }

    public Translation2d getTranslationToNearestAlgaeScoringLocation() {
        Pose2d currentPose = getStrategyPose();
        Translation2d tagTranslation = FieldUtil.AlgaeScoring.getNearestAlgaeScoringTagPose(currentPose).getTranslation();
        return tagTranslation.minus(currentPose.getTranslation());
    }

    public double getAngleToNearestAlgaeScoringLocation() {
        return getTranslationToNearestAlgaeScoringLocation().getAngle().getDegrees();
    }

    public double getNearestAlgaeScoringHeading() {
        return FieldUtil.AlgaeScoring.getNearestAlgaeScoringTagPose(getStrategyPose()).getRotation().rotateBy(Rotation2d.kPi).getDegrees();
    }

    public double getProcessorScoringHeading() {
        return FieldUtil.AlgaeScoring.getProcessorTagPose().getRotation().rotateBy(Rotation2d.kPi).getDegrees();
    }

    public double getNetScoringHeading() {
        return FieldUtil.AlgaeScoring.getNetTagPose().getRotation().rotateBy(Rotation2d.kPi).getDegrees();
    }

    public boolean nearestAlgaeIsHigh() {
        return FieldUtil.Reef.getAlgaeReefLevelFromTag(FieldUtil.Reef.getNearestReefTag(getStrategyPose())) == FieldUtil.Reef.AlgaeLocation.HIGH;
    }

    public boolean nearestAlgaeScoringIsNet() {
        return FieldUtil.AlgaeScoring.getAlgaeScoringFromTag(FieldUtil.AlgaeScoring.getNearestAlgaeScoringTag(getStrategyPose())) == FieldUtil.AlgaeScoring.ScoringLocation.NET;
    }

    public void setLocalizationStrategyFromChooser() {
        LocalizationStrategy strategy = localizationChooser.getSelected();
        if (this.strategy != strategy) {
            this.strategy = strategy;
        }
    }

    public void setRotations(Rotation2d heading) {
        swerve.resetRotation(heading);
        poseEstimator.resetRotation(heading);
    }

    public void syncRotations() {
        setRotations(poseEstimator.getEstimatedPosition().getRotation());
    }

    public void updateLimelightPoseEstimation() {
        if (LimelightUtil.isMultiTag() && LimelightUtil.isTagClear()) {
            Pose2d megaTagPose = LimelightUtil.getMegaTagOnePose();
            poseEstimator.addVisionMeasurement(
                    megaTagPose,
                    Timer.getFPGATimestamp() - LimelightUtil.getLatency(),
                    Constants.VisionConstants.VISION_STD_DEV_MULTITAG_FUNCTION.apply(LimelightUtil.getNearestTagDist())
            );
        }
    }

    public void periodic() {
        poseEstimator.update(this.swerve.getState().RawHeading, this.swerve.getState().ModulePositions);
        setLocalizationStrategyFromChooser();
        // localizationTelemetry.publishValues();
    }
}