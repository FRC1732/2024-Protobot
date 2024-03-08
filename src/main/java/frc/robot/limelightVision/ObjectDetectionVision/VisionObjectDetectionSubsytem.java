package frc.robot.limelightVision.ObjectDetectionVision;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.limelightVision.LimelightHelpers;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class VisionObjectDetectionSubsytem extends SubsystemBase {
  private ShuffleboardTab tab;
  private double lastDistance;

  @AutoLog
  public static class VisionObjectDetectionSubsystemIOInput {
    double Distance = 0.0;
    double LatencyCapture = 0.0;
    double LatencyPipline = 0.0;
    double TX = 0.0;
    double TY = 0.0;
    boolean hasTarget = false;
  }

  private VisionObjectDetectionSubsystemIOInputAutoLogged inputs =
      new VisionObjectDetectionSubsystemIOInputAutoLogged();

  public VisionObjectDetectionSubsytem() {
    setUpShuffleboard();
  }

  private String getLimelightName() {
    return VisionObjectDetectionConstants.LIMELIGHT_NAME;
  }

  public double getTX() {
    return LimelightHelpers.getTX(getLimelightName());
  }

  public double getTY() {
    return LimelightHelpers.getTY(getLimelightName());
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
    if (VisionObjectDetectionConstants.LOGGING) {
      updateInputs();
    }
  }

  private void setUpShuffleboard() {
    tab = Shuffleboard.getTab("Vision");
    tab.addDouble("Tx", () -> this.getTX());
    tab.addDouble("Ty", () -> this.getTY());
    tab.addDouble("Distance", () -> this.getDistanceToTarget());
    tab.addDouble("Latency Capture", () -> this.getLatencyCapture());
    tab.addDouble("Latency Pipeline", () -> this.getLatencyPipeline());
    tab.addBoolean("Has Target", () -> this.hasTarget());
  }

  private void updateInputs() {
    inputs.Distance = getDistanceToTarget();
    inputs.LatencyCapture = getLatencyCapture();
    inputs.LatencyPipline = getLatencyPipeline();
    inputs.TX = getTX();
    inputs.TY = getTY();
    inputs.hasTarget = hasTarget();

    Logger.processInputs("Visions", inputs);
  }
}
