package frc.robot.limelightVision.ApriltagVision;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.limelightVision.LimelightHelpers;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class VisionApriltagSubsystem extends SubsystemBase {
  private ShuffleboardTab tab;
  private double lastDistance;

  @AutoLog
  public static class VisionApriltagSubsystemIOInput {
    double Distance = 0.0;
    double LatencyCapture = 0.0;
    double LatencyPipline = 0.0;
    double TX = 0.0;
    double TY = 0.0;
    boolean hasTarget = false;
    boolean hasStageTarget = false;
    String pipelineString = "Unknown";
  }

  private VisionApriltagSubsystemIOInputAutoLogged inputs =
      new VisionApriltagSubsystemIOInputAutoLogged();

  public VisionApriltagSubsystem() {
    setUpShuffleboard();
  }

  private String getLimelightName() {
    return VisionApriltagConstants.LIMELIGHT_NAME;
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

  public boolean hasStageTarget() {
    return LimelightHelpers.getTV(getLimelightName())
        && getAprilTagId() >= 11
        && getAprilTagId() <= 16;
  }

  public boolean hasNoteTarget() {
    return LimelightHelpers.getNeuralClassID(getLimelightName()) > 0;
  }

  public double getAprilTagId() {
    return LimelightHelpers.getFiducialID(getLimelightName());
  }

  public void setPipeline(VisionApriltagConstants.Pipelines pipeline) {
    LimelightHelpers.setPipelineIndex(getLimelightName(), pipeline.ordinal());
  }

  public String getPipelineAsString() {
    double index = LimelightHelpers.getCurrentPipelineIndex(getLimelightName());

    if (index < VisionApriltagConstants.Pipelines.values().length) {
      return VisionApriltagConstants.Pipelines.values()[(int) index].toString();
    }
    return "Unknown";
  }

  public double getDistanceToTarget() {
    if (!hasTarget()) {
      return lastDistance;
    }
    double cameraHeight = 22;
    double targetHeight = 56.375;
    double heightDiff = targetHeight - cameraHeight;
    double cameraAngle = 23;
    double theta = Math.toRadians(cameraAngle + getTY());
    lastDistance = heightDiff / Math.tan(theta);
    return lastDistance;
  }

  @Override
  public void periodic() {
    if (VisionApriltagConstants.LOGGING) {
      updateInputs();
    }
  }

  private void setUpShuffleboard() {
    tab = Shuffleboard.getTab("VisionApriltag");
    tab.addDouble("Tx", () -> this.getTX());
    tab.addDouble("Ty", () -> this.getTY());
    tab.addDouble("Distance", () -> this.getDistanceToTarget());
    tab.addDouble("Latency Capture", () -> this.getLatencyCapture());
    tab.addDouble("Latency Pipeline", () -> this.getLatencyPipeline());
    tab.addBoolean("Has Target", () -> this.hasTarget());
    tab.addBoolean("Has Stage Target", () -> this.hasStageTarget());
    tab.addDouble("April Tag", () -> this.getAprilTagId());
    tab.addString("Pipeline", () -> this.getPipelineAsString());
  }

  private void updateInputs() {
    inputs.Distance = getDistanceToTarget();
    inputs.LatencyCapture = getLatencyCapture();
    inputs.LatencyPipline = getLatencyPipeline();
    inputs.TX = getTX();
    inputs.TY = getTY();
    inputs.hasTarget = hasTarget();
    inputs.hasStageTarget = hasStageTarget();
    inputs.pipelineString = getPipelineAsString();

    Logger.processInputs("VisionApriltag", inputs);
  }
}
