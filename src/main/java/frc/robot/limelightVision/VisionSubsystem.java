package frc.robot.limelightVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class VisionSubsystem extends SubsystemBase {
  private ShuffleboardTab tab;

  public VisionSubsystem() {
    setUpShuffleboard();
  }

  private String getLimelightName() {
    return VisionConstants.LIMELIGHT_NAME;
  }

  public double getTX() {
    return LimelightHelpers.getTX(getLimelightName());
  }

  public double getTY() {
    return LimelightHelpers.getTY(getLimelightName());
  }

  public Pose2d getPose2dFromLimelight() {
    return LimelightHelpers.getBotPose2d(getLimelightName());
  }

  public double getLatencyCapture() {
    return LimelightHelpers.getLatency_Capture(getLimelightName());
  }

  public double getLatencyPipeline() {
    return LimelightHelpers.getLatency_Pipeline(getLimelightName());
  }

  public boolean hasTarget() {
    return LimelightHelpers.getTV(getLimelightName());
  }

  public double getAprilTagId() {
    return LimelightHelpers.getFiducialID(getLimelightName());
  }

  public double getDistanceToTarget() {
    double cameraHeight = 28;
    double targetHeight = 60;
    double heightDiff = targetHeight - cameraHeight;
    double cameraAngle = 0;
    double theta = Math.toRadians(cameraAngle + getTY());
    return heightDiff / Math.tan(theta);
  }

  @Override
  public void periodic() {}

  private void setUpShuffleboard() {
    tab = Shuffleboard.getTab("Vision");
    tab.addDouble("Tx", () -> this.getTX());
    tab.addDouble("Ty", () -> this.getTY());
    tab.addDouble("Latency Capture", () -> this.getLatencyCapture());
    tab.addDouble("Latency Pipeline", () -> this.getLatencyPipeline());
    tab.addBoolean("Has Target", () -> this.hasTarget());
    tab.addDouble("April Tag", () -> this.getAprilTagId());
  }
}
