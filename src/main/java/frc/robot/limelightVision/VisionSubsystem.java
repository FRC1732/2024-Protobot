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

  @Override
  public void periodic() {
    // TODO Auto-generated method stub
    super.periodic();
  }

  private void setUpShuffleboard() {
    tab = Shuffleboard.getTab("Vision");
    tab.addDouble("Tx", () -> LimelightHelpers.getTX(getLimelightName()));
    tab.addDouble("Ty", () -> LimelightHelpers.getTY(getLimelightName()));
    tab.addDouble("Latency Capture", () -> LimelightHelpers.getLatency_Capture(getLimelightName()));
    tab.addDouble(
        "Latency Pipeline", () -> LimelightHelpers.getLatency_Pipeline(getLimelightName()));
  }
}
