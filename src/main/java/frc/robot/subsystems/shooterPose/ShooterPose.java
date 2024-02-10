package frc.robot.subsystems.shooterPose;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;

public class ShooterPose extends SubsystemBase {

  private CANSparkMax shooterElevatorLeftMotor;
  private CANSparkMax shooterElevtorRightMotor;

  private CANSparkMax shooterTiltMotor;

  private PIDController shooterElevatorPID;
  private PIDController shooterTiltPID;

  private ShuffleboardTab ShooterPoseTab;

  private TunableNumber shooterElevatorP;
  private TunableNumber shooterElevtorI;
  private TunableNumber shooterElevatorD;

  private TunableNumber shooterTiltP;
  private TunableNumber shooterTiltI;
  private TunableNumber shooterTiltD;

  private Pose pose;

  public ShooterPose() {
    shooterElevatorLeftMotor = new CANSparkMax(ShooterPoseConstants.SHOOTER_POSE_LEFT_MOTOR_CAN_ID,
        MotorType.kBrushless);
    shooterElevtorRightMotor = new CANSparkMax(ShooterPoseConstants.SHOOTER_POSE_RIGHT_MOTOR_CAN_ID,
        MotorType.kBrushless);

    shooterElevatorLeftMotor.setInverted(true);
    shooterElevatorLeftMotor.follow(shooterElevtorRightMotor);

    shooterTiltMotor = new CANSparkMax(ShooterPoseConstants.SHOOTER_TILT_MOTOR_CAN_ID, MotorType.kBrushless);

    pose = Pose.NEUTRAL;

    shooterTiltPID = new PIDController(
        ShooterPoseConstants.SHOOTER_TILT_P,
        ShooterPoseConstants.SHOOTER_TILT_I,
        ShooterPoseConstants.SHOOTER_TILT_D);

    shooterElevatorPID = new PIDController(
        ShooterPoseConstants.SHOOTER_POSE_RIGHT_MOTOR_P,
        ShooterPoseConstants.SHOOTER_POSE_RIGHT_MOTOR_I,
        ShooterPoseConstants.SHOOTER_POSE_RIGHT_MOTOR_D);

    shooterElevatorPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_SPEAKER_SETPOINT);

    if (ShooterPoseConstants.SHOOTER_POSE_TESTING) {
      setUpShuffleboard();
    }
  }

  public void setShooterPose(Pose pose) {
    this.pose = pose;
    setToSetpointByPose();
  }

  @Deprecated ( since = "Use setShooterPose(Pose pose) instead")
  public void setShooterPoseSetpointSpeaker() {
    pose = Pose.SPEAKER;
    shooterElevatorPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_SPEAKER_SETPOINT);
  }

  @Deprecated ( since = "Use setShooterPose(Pose pose) instead")
  public void setShooterPoseSetpointAmp() {
    pose = Pose.AMP;
    shooterElevatorPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_AMP_SETPOINT);
  }

  @Deprecated   ( since = "Use setShooterPose(Pose pose) instead")
  public void setShooterPoseSetpointTrap() {
    pose = Pose.TRAP;
    shooterElevatorPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_TRAP_SETPOINT);
  }

  @Deprecated  ( since = "Use setShooterPose(Pose pose) instead")
  public void setShooterTiltSetpointSpeaker115() {
    pose = Pose.DISTANCE_115;
    shooterTiltPID.setSetpoint(ShooterPoseConstants.SHOOTER_TILT_SPEAKER_115_SETPOINT);
  }

  @Deprecated   ( since = "Use setShooterPose(Pose pose) instead")
  public void setShooterTiltSetpointSpeaker125() {
    pose = Pose.DISTANCE_125;
    shooterTiltPID.setSetpoint(ShooterPoseConstants.SHOOTER_TILT_SPEAKER_125_SETPOINT);
  }

  @Deprecated  ( since = "Use setShooterPose(Pose pose) instead")
  public void setShooterTiltSetpointSpeaker150() {
    pose = Pose.DISTANCE_150;
    shooterTiltPID.setSetpoint(ShooterPoseConstants.SHOOTER_TILT_SPEAKER_150_SETPOINT);
  }

  private void setToSetpointByPose() {
    switch (pose) {
      case NEUTRAL:
        shooterElevatorPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_NEUTRAL_SETPOINT);
        shooterTiltPID.setSetpoint(ShooterPoseConstants.SHOOTER_TILT_NEUTRAL_SETPOINT);
        break;

      case SPEAKER:
        shooterElevatorPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_SPEAKER_SETPOINT);
        shooterTiltPID.setSetpoint(ShooterPoseConstants.SHOOTER_TILT_SPEAKER_115_SETPOINT);
        break;

      case AMP:
        shooterElevatorPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_AMP_SETPOINT);
        shooterTiltPID.setSetpoint(ShooterPoseConstants.SHOOTER_TILT_AMP_SETPOINT);
        break;

      case TRAP:
        shooterElevatorPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_TRAP_SETPOINT);
        shooterTiltPID.setSetpoint(ShooterPoseConstants.SHOOTER_TILT_TRAP_SETPOINT);
        break;

      case SOURCE:
        shooterElevatorPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_SOURCE_SETPOINT);
        shooterTiltPID.setSetpoint(ShooterPoseConstants.SHOOTER_TILT_SOURCE_SETPOINT);
        break;

      case DISTANCE_VISION:
        shooterElevatorPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_SPEAKER_SETPOINT);
        break;

      case DISTANCE_115:
        shooterElevatorPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_SPEAKER_SETPOINT);
        shooterTiltPID.setSetpoint(ShooterPoseConstants.SHOOTER_TILT_SPEAKER_115_SETPOINT);
        break;

      case DISTANCE_125:
        shooterElevatorPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_SPEAKER_SETPOINT);
        shooterTiltPID.setSetpoint(ShooterPoseConstants.SHOOTER_TILT_SPEAKER_125_SETPOINT);
        break;

      case DISTANCE_150:
        shooterElevatorPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_SPEAKER_SETPOINT);
        shooterTiltPID.setSetpoint(ShooterPoseConstants.SHOOTER_TILT_SPEAKER_150_SETPOINT);
        break;

      case FEEDING:
        shooterElevatorPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_FEEDING_SETPOINT);
        shooterTiltPID.setSetpoint(ShooterPoseConstants.SHOOTER_TILT_FEEDING_SETPOINT);
        break;
    }
  }

  public void setShooterTiltVision(double angle) { // Don't change the setpoint of tilt behavior
    pose = Pose.DISTANCE_VISION;
    shooterTiltPID.setSetpoint(angle);
  }

  private void setUpShuffleboard() {
    shooterElevatorP = new TunableNumber(
        "Elevator Right Motor P", ShooterPoseConstants.SHOOTER_POSE_RIGHT_MOTOR_P);
    shooterElevtorI = new TunableNumber(
        "Elevator Right Motor I", ShooterPoseConstants.SHOOTER_POSE_RIGHT_MOTOR_I);
    shooterElevatorD = new TunableNumber(
        "Elevator Right Motor D", ShooterPoseConstants.SHOOTER_POSE_RIGHT_MOTOR_D);

    shooterTiltP = new TunableNumber("Shooter Tilt P", ShooterPoseConstants.SHOOTER_TILT_P);
    shooterTiltI = new TunableNumber("Shooter Tilt I", ShooterPoseConstants.SHOOTER_TILT_I);
    shooterTiltD = new TunableNumber("Shooter Tilt D", ShooterPoseConstants.SHOOTER_TILT_D);

    ShooterPoseTab = Shuffleboard.getTab("Elevator");

    ShooterPoseTab.add("Elevator Right Motor P", shooterElevatorP);
    ShooterPoseTab.add("Elevator Right Motor I", shooterElevtorI);
    ShooterPoseTab.add("Elevator Right Motor D", shooterElevatorD);

    ShooterPoseTab.add("Shooter Tilt P", shooterTiltP);
    ShooterPoseTab.add("Shooter Tilt I", shooterTiltI);
    ShooterPoseTab.add("Shooter Tilt D", shooterTiltD);
  }

  public void periodic() {

    shooterElevtorRightMotor.set(
        shooterElevatorPID.calculate(
            shooterElevtorRightMotor.getEncoder().getPosition(),
            shooterElevatorPID.getSetpoint()));
    shooterTiltMotor.set(
        shooterTiltPID.calculate(
            shooterTiltMotor.getEncoder().getPosition(), shooterTiltPID.getSetpoint()));

    if (ShooterPoseConstants.SHOOTER_POSE_TESTING) {
      shooterElevatorPID.setP(shooterElevatorP.get());
      shooterElevatorPID.setI(shooterElevtorI.get());
      shooterElevatorPID.setD(shooterElevatorD.get());

      shooterTiltPID.setP(shooterTiltP.get());
      shooterTiltPID.setI(shooterTiltI.get());
      shooterTiltPID.setD(shooterTiltD.get());
    }
  }
}
