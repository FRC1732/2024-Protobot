package frc.robot.subsystems.shooterPose;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;

public class ShooterPose extends SubsystemBase {

  private CANSparkMax shooterHeightLeftMotor;
  private CANSparkMax shooterHeightRightMotor;

  private CANSparkMax shooterTiltMotor;

  private ProfiledPIDController shooterHeightPID;
  private ElevatorFeedforward shooterHeightFeedForward;
  private ProfiledPIDController shooterTiltPID;
  private ArmFeedforward shooterTiltFeedforward;

  private ShuffleboardTab ShooterPoseTab;

  private TunableNumber shooterHeightP;
  private TunableNumber shooterHeightI;
  private TunableNumber shooterHeightD;

  private TunableNumber shooterTiltP;
  private TunableNumber shooterTiltI;
  private TunableNumber shooterTiltD;

  private double period = 0.02;

  private Pose pose;

  public ShooterPose() {
    shooterHeightLeftMotor =
        new CANSparkMax(
            ShooterPoseConstants.SHOOTER_HEIGHT_LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
    shooterHeightRightMotor =
        new CANSparkMax(
            ShooterPoseConstants.SHOOTER_HEIGHT_RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);

    shooterHeightLeftMotor.setInverted(true);
    shooterHeightLeftMotor.follow(shooterHeightRightMotor);

    shooterTiltMotor =
        new CANSparkMax(ShooterPoseConstants.SHOOTER_TILT_MOTOR_CAN_ID, MotorType.kBrushless);

    pose = Pose.HANDOFF;

    shooterTiltPID =
        new ProfiledPIDController(
            ShooterPoseConstants.SHOOTER_TILT_KP,
            ShooterPoseConstants.SHOOTER_TILT_KI,
            ShooterPoseConstants.SHOOTER_TILT_KD,
            new TrapezoidProfile.Constraints(
                ShooterPoseConstants.SHOOTER_TILT_MAX_VELOCITY,
                ShooterPoseConstants.SHOOTER_TILT_MAX_ACCELERATION));

    shooterTiltFeedforward =
        new ArmFeedforward(
            ShooterPoseConstants.SHOOTER_TILT_KS,
            ShooterPoseConstants.SHOOTER_TILT_KG,
            ShooterPoseConstants.SHOOTER_TILT_KV,
            ShooterPoseConstants.SHOOTER_TILT_KA);

    shooterHeightPID =
        new ProfiledPIDController(
            ShooterPoseConstants.SHOOTER_HEIGHT_KP,
            ShooterPoseConstants.SHOOTER_HEIGHT_KI,
            ShooterPoseConstants.SHOOTER_HEIGHT_KD,
            new TrapezoidProfile.Constraints(
                ShooterPoseConstants.SHOOTER_HEIGHT_MAX_VELOCITY,
                ShooterPoseConstants.SHOOTER_HEIGHT_MAX_ACCELERATION));

    shooterHeightFeedForward =
        new ElevatorFeedforward(
            ShooterPoseConstants.SHOOTER_HEIGHT_KS,
            ShooterPoseConstants.SHOOTER_HEIGHT_KG,
            ShooterPoseConstants.SHOOTER_HEIGHT_KV,
            ShooterPoseConstants.SHOOTER_HEIGHT_KA);

    if (ShooterPoseConstants.SHOOTER_POSE_TESTING) {
      setUpShuffleboard();
    }
  }

  public void setShooterPose(Pose pose) {
    this.pose = pose;
    setToSetpointByPose();
  }

  // @Deprecated(since = "Use setShooterPose(Pose pose) instead")
  // public void setShooterPoseSetpointSpeaker() {
  //   pose = Pose.SUBWOOFER;
  //   shooterHeightPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_SPEAKER_SETPOINT);
  // }

  // @Deprecated(since = "Use setShooterPose(Pose pose) instead")
  // public void setShooterPoseSetpointAmp() {
  //   pose = Pose.AMP;
  //   shooterHeightPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_AMP_SETPOINT);
  // }

  // @Deprecated(since = "Use setShooterPose(Pose pose) instead")
  // public void setShooterPoseSetpointTrap() {
  //   pose = Pose.TRAP;
  //   shooterHeightPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_TRAP_SETPOINT);
  // }

  // @Deprecated(since = "Use setShooterPose(Pose pose) instead")
  // public void setShooterTiltSetpointSpeaker115() {
  //   pose = Pose.DISTANCE_115;
  //   shooterTiltPID.setSetpoint(ShooterPoseConstants.SHOOTER_TILT_SPEAKER_115_SETPOINT);
  // }

  // @Deprecated(since = "Use setShooterPose(Pose pose) instead")
  // public void setShooterTiltSetpointSpeaker125() {
  //   pose = Pose.DISTANCE_125;
  //   shooterTiltPID.setSetpoint(ShooterPoseConstants.SHOOTER_TILT_SPEAKER_125_SETPOINT);
  // }

  // @Deprecated(since = "Use setShooterPose(Pose pose) instead")
  // public void setShooterTiltSetpointSpeaker150() {
  //   pose = Pose.DISTANCE_150;
  //   shooterTiltPID.setSetpoint(ShooterPoseConstants.SHOOTER_TILT_SPEAKER_150_SETPOINT);
  // }

  public Pose getCurrentPose() {
    return pose;
  }

  private void setToSetpointByPose() {
    // switch (pose) {
    //   case HANDOFF:
    //     shooterHeightPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_NEUTRAL_SETPOINT);
    //     shooterTiltPID.setSetpoint(ShooterPoseConstants.SHOOTER_TILT_NEUTRAL_SETPOINT);
    //     break;

    //   case SUBWOOFER:
    //     shooterHeightPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_SPEAKER_SETPOINT);
    //     shooterTiltPID.setSetpoint(ShooterPoseConstants.SHOOTER_TILT_SPEAKER_115_SETPOINT);
    //     break;

    //   case AMP:
    //     shooterHeightPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_AMP_SETPOINT);
    //     shooterTiltPID.setSetpoint(ShooterPoseConstants.SHOOTER_TILT_AMP_SETPOINT);
    //     break;

    //   case TRAP:
    //     shooterHeightPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_TRAP_SETPOINT);
    //     shooterTiltPID.setSetpoint(ShooterPoseConstants.SHOOTER_TILT_TRAP_SETPOINT);
    //     break;

    //   case SOURCE:
    //     shooterHeightPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_SOURCE_SETPOINT);
    //     shooterTiltPID.setSetpoint(ShooterPoseConstants.SHOOTER_TILT_SOURCE_SETPOINT);
    //     break;

    //   case DISTANCE_VISION:
    //     shooterHeightPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_SPEAKER_SETPOINT);
    //     break;

    //   case DISTANCE_115:
    //     shooterHeightPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_SPEAKER_SETPOINT);
    //     shooterTiltPID.setSetpoint(ShooterPoseConstants.SHOOTER_TILT_SPEAKER_115_SETPOINT);
    //     break;

    //   case DISTANCE_125:
    //     shooterHeightPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_SPEAKER_SETPOINT);
    //     shooterTiltPID.setSetpoint(ShooterPoseConstants.SHOOTER_TILT_SPEAKER_125_SETPOINT);
    //     break;

    //   case DISTANCE_150:
    //     shooterHeightPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_SPEAKER_SETPOINT);
    //     shooterTiltPID.setSetpoint(ShooterPoseConstants.SHOOTER_TILT_SPEAKER_150_SETPOINT);
    //     break;

    //   case FEEDING:
    //     shooterHeightPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_FEEDING_SETPOINT);
    //     shooterTiltPID.setSetpoint(ShooterPoseConstants.SHOOTER_TILT_FEEDING_SETPOINT);
    //     break;
    // }
  }

  public void setShooterTiltVision(double angle) { // Don't change the setpoint of tilt behavior
    // pose = Pose.DISTANCE_VISION;
    // shooterTiltPID.setSetpoint(angle);
  }

  private void setUpShuffleboard() {
    shooterHeightP =
        new TunableNumber("Elevator Right Motor P", ShooterPoseConstants.SHOOTER_HEIGHT_KP);
    shooterHeightI =
        new TunableNumber("Elevator Right Motor I", ShooterPoseConstants.SHOOTER_HEIGHT_KI);
    shooterHeightD =
        new TunableNumber("Elevator Right Motor D", ShooterPoseConstants.SHOOTER_HEIGHT_KD);

    shooterTiltP = new TunableNumber("Shooter Tilt P", ShooterPoseConstants.SHOOTER_TILT_KP);
    shooterTiltI = new TunableNumber("Shooter Tilt I", ShooterPoseConstants.SHOOTER_TILT_KI);
    shooterTiltD = new TunableNumber("Shooter Tilt D", ShooterPoseConstants.SHOOTER_TILT_KD);

    ShooterPoseTab = Shuffleboard.getTab("Elevator");

    ShooterPoseTab.add("Elevator Right Motor P", shooterHeightP);
    ShooterPoseTab.add("Elevator Right Motor I", shooterHeightI);
    ShooterPoseTab.add("Elevator Right Motor D", shooterHeightD);

    ShooterPoseTab.add("Shooter Tilt P", shooterTiltP);
    ShooterPoseTab.add("Shooter Tilt I", shooterTiltI);
    ShooterPoseTab.add("Shooter Tilt D", shooterTiltD);
  }

  public void periodic() {

    shooterHeightRightMotor.set(
        shooterHeightPID.calculate(
            shooterHeightRightMotor.getEncoder().getPosition(), shooterHeightPID.getSetpoint()));
    shooterTiltMotor.set(
        shooterTiltPID.calculate(
            shooterTiltMotor.getEncoder().getPosition(), shooterTiltPID.getSetpoint()));

    if (ShooterPoseConstants.SHOOTER_POSE_TESTING) {
      shooterHeightPID.setP(shooterHeightP.get());
      shooterHeightPID.setI(shooterHeightI.get());
      shooterHeightPID.setD(shooterHeightD.get());

      shooterTiltPID.setP(shooterTiltP.get());
      shooterTiltPID.setI(shooterTiltI.get());
      shooterTiltPID.setD(shooterTiltD.get());
    }
  }
}
