package frc.robot.subsystems.shooterPose;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;

public class ShooterPose extends SubsystemBase {

  private CANSparkMax shooterPoseLeftMotor;
  private CANSparkMax shooterPoseRightMotor;

  private CANSparkMax shooterTiltMotor;
  
  private PIDController shooterPoseRightMotorPID;
  private PIDController shooterTiltPID;

  private ShuffleboardTab ShooterPoseTab;

  private TunableNumber shooterPoseRightMotorP;
  private TunableNumber shooterPoseRightMotorI;
  private TunableNumber shooterPoseRightMotorD;

  private TunableNumber shooterTiltP;
  private TunableNumber shooterTiltI;
  private TunableNumber shooterTiltD;

  public ShooterPose() {
    shooterPoseLeftMotor =
        new CANSparkMax(ShooterPoseConstants.SHOOTER_POSE_LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
    shooterPoseRightMotor =
        new CANSparkMax(ShooterPoseConstants.SHOOTER_POSE_RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);

    shooterTiltMotor = new CANSparkMax(ShooterPoseConstants.SHOOTER_TILT_MOTOR_CAN_ID, MotorType.kBrushless);

    shooterTiltPID = new PIDController(ShooterPoseConstants.SHOOTER_TILT_P, ShooterPoseConstants.SHOOTER_TILT_I, ShooterPoseConstants.SHOOTER_TILT_D);
    
    shooterPoseLeftMotor.setInverted(true);
    shooterPoseLeftMotor.follow(shooterPoseRightMotor);

    shooterPoseRightMotorPID =
        new PIDController(
            ShooterPoseConstants.SHOOTER_POSE_RIGHT_MOTOR_P,
            ShooterPoseConstants.SHOOTER_POSE_RIGHT_MOTOR_I,
            ShooterPoseConstants.SHOOTER_POSE_RIGHT_MOTOR_D);

    shooterPoseRightMotorPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_SPEAKER_SETPOINT);

    if (ShooterPoseConstants.SHOOTER_POSE_TESTING) {
      setUpShuffleboard();
    }
  }

  public void setElevatorSetpointSpeaker() {
    shooterPoseRightMotorPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_SPEAKER_SETPOINT);
  }

  public void setElevatorSetpointAmp() {
    shooterPoseRightMotorPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_AMP_SETPOINT);
  }

  public void setElevatorSetpointTrap() {
    shooterPoseRightMotorPID.setSetpoint(ShooterPoseConstants.SHOOTER_POSE_TRAP_SETPOINT);
  }

  public void setShooterTiltSetpointSpeaker115() {
    shooterTiltPID.setSetpoint(ShooterPoseConstants.SHOOTER_TILT_SPEAKER_115_SETPOINT);
  }

  public void setShooterTiltSetpointSpeaker125() {
    shooterTiltPID.setSetpoint(ShooterPoseConstants.SHOOTER_TILT_SPEAKER_125_SETPOINT);
  }

  public void setShooterTiltSetpointSpeaker150() {
    shooterTiltPID.setSetpoint(ShooterPoseConstants.SHOOTER_TILT_SPEAKER_150_SETPOINT);
  }

  public void setShooterTiltAngle(double angle) {
    shooterTiltPID.setSetpoint(angle);
  }

  private void setUpShuffleboard() {
    shooterPoseRightMotorP =
        new TunableNumber("Elevator Right Motor P", ShooterPoseConstants.SHOOTER_POSE_RIGHT_MOTOR_P);
    shooterPoseRightMotorI =
        new TunableNumber("Elevator Right Motor I", ShooterPoseConstants.SHOOTER_POSE_RIGHT_MOTOR_I);
    shooterPoseRightMotorD =
        new TunableNumber("Elevator Right Motor D", ShooterPoseConstants.SHOOTER_POSE_RIGHT_MOTOR_D);

    shooterTiltP = new TunableNumber("Shooter Tilt P", ShooterPoseConstants.SHOOTER_TILT_P);
    shooterTiltI = new TunableNumber("Shooter Tilt I", ShooterPoseConstants.SHOOTER_TILT_I);
    shooterTiltD = new TunableNumber("Shooter Tilt D", ShooterPoseConstants.SHOOTER_TILT_D);

    ShooterPoseTab = Shuffleboard.getTab("Elevator");

    ShooterPoseTab.add("Elevator Right Motor P", shooterPoseRightMotorP);
    ShooterPoseTab.add("Elevator Right Motor I", shooterPoseRightMotorI);
    ShooterPoseTab.add("Elevator Right Motor D", shooterPoseRightMotorD);

    ShooterPoseTab.add("Shooter Tilt P", shooterTiltP);
    ShooterPoseTab.add("Shooter Tilt I", shooterTiltI);
    ShooterPoseTab.add("Shooter Tilt D", shooterTiltD);
  }

  public void periodic() {
    shooterPoseRightMotor.set(
        shooterPoseRightMotorPID.calculate(
            shooterPoseRightMotor.getEncoder().getPosition(), shooterPoseRightMotorPID.getSetpoint()));
    shooterTiltMotor.set(shooterTiltPID.calculate(shooterTiltMotor.getEncoder().getPosition(), shooterTiltPID.getSetpoint()));

    if (ShooterPoseConstants.SHOOTER_POSE_TESTING) {
      shooterPoseRightMotorPID.setP(shooterPoseRightMotorP.get());
      shooterPoseRightMotorPID.setI(shooterPoseRightMotorI.get());
      shooterPoseRightMotorPID.setD(shooterPoseRightMotorD.get());

      shooterTiltPID.setP(shooterTiltP.get());
      shooterTiltPID.setI(shooterTiltI.get());
      shooterTiltPID.setD(shooterTiltD.get());
    }
  }
}