package frc.robot.subsystems.shooterPose;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkLimitSwitch;
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
  private SparkLimitSwitch shooterHeightLimitSwitch;
  private RelativeEncoder shooterHeightEncoder;

  private CANSparkMax shooterTiltMotor;
  private RelativeEncoder shooterTiltEncoder;
  private AbsoluteEncoder shooterTiltAbsoluteEncoder;

  private ProfiledPIDController shooterHeightPID;
  private ElevatorFeedforward shooterHeightFeedForward;
  private ProfiledPIDController shooterTiltPID;
  private ArmFeedforward shooterTiltFeedforward;

  private ShuffleboardTab shooterPoseTab;

  private TunableNumber shooterHeightP;
  private TunableNumber shooterHeightI;
  private TunableNumber shooterHeightD;

  private TunableNumber shooterTiltP;
  private TunableNumber shooterTiltI;
  private TunableNumber shooterTiltD;

  public ShooterPose() {
    shooterHeightLeftMotor =
        new CANSparkMax(
            ShooterPoseConstants.SHOOTER_HEIGHT_LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
    shooterHeightRightMotor =
        new CANSparkMax(
            ShooterPoseConstants.SHOOTER_HEIGHT_RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);

    shooterHeightRightMotor.restoreFactoryDefaults();
    shooterHeightLeftMotor.restoreFactoryDefaults();

    shooterHeightRightMotor.enableVoltageCompensation(12);
    shooterHeightLeftMotor.enableVoltageCompensation(12);

    // shooterHeightRightMotor.setIdleMode(IdleMode.kBrake);
    // shooterHeightLeftMotor.setIdleMode(IdleMode.kBrake);

    // testing only
    shooterHeightRightMotor.setIdleMode(IdleMode.kCoast);
    shooterHeightLeftMotor.setIdleMode(IdleMode.kCoast);

    shooterHeightLeftMotor.follow(shooterHeightRightMotor, true);

    // shooterHeightRightMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    // shooterHeightRightMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    // shooterHeightRightMotor.setSoftLimit(
    //     SoftLimitDirection.kForward, (float) ShooterPoseConstants.MAX_SHOOTER_HEIGHT_INCHES);
    // shooterHeightRightMotor.setSoftLimit(
    //     SoftLimitDirection.kReverse, (float) ShooterPoseConstants.MIN_SHOOTER_HEIGHT_INCHES);

    shooterHeightEncoder = shooterHeightRightMotor.getEncoder();
    shooterHeightEncoder.setPositionConversionFactor(
        ShooterPoseConstants.SHOOTER_HEIGHT_INCHES_PER_ROTATION);
    shooterHeightEncoder.setPosition(0);

    shooterTiltMotor =
        new CANSparkMax(ShooterPoseConstants.SHOOTER_TILT_MOTOR_CAN_ID, MotorType.kBrushless);

    shooterTiltMotor.restoreFactoryDefaults();
    shooterTiltMotor.enableVoltageCompensation(12);
    // shooterTiltMotor.setIdleMode(IdleMode.kBrake);
    shooterTiltMotor.setIdleMode(IdleMode.kCoast);

    // shooterTiltMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    // shooterTiltMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);
    // shooterTiltMotor.setSoftLimit(
    //     SoftLimitDirection.kForward, (float) ShooterPoseConstants.MAX_SHOOTER_TILT_DEGREES);
    // shooterTiltMotor.setSoftLimit(
    //     SoftLimitDirection.kReverse, (float) ShooterPoseConstants.MIN_SHOOTER_TILT_DEGREES);

    shooterTiltAbsoluteEncoder =
        shooterTiltMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    shooterTiltAbsoluteEncoder.setPositionConversionFactor(
        ShooterPoseConstants.SHOOTER_TILT_DEGREES_PER_ROTATION);
    shooterTiltAbsoluteEncoder.setZeroOffset(ShooterPoseConstants.SHOOTER_TILT_ABSOLUTE_OFFSET);

    shooterTiltEncoder = shooterTiltMotor.getEncoder();
    shooterTiltEncoder.setPositionConversionFactor(
        ShooterPoseConstants.SHOOTER_TILT_DEGREES_PER_ROTATION);
    shooterTiltEncoder.setPosition(shooterTiltAbsoluteEncoder.getPosition());

    shooterTiltPID =
        new ProfiledPIDController(
            ShooterPoseConstants.SHOOTER_TILT_KP,
            ShooterPoseConstants.SHOOTER_TILT_KI,
            ShooterPoseConstants.SHOOTER_TILT_KD,
            new TrapezoidProfile.Constraints(
                ShooterPoseConstants.SHOOTER_TILT_MAX_VELOCITY,
                ShooterPoseConstants.SHOOTER_TILT_MAX_ACCELERATION),
            ShooterPoseConstants.SHOOTER_PID_PERIOD_SEC);
    shooterTiltPID.setTolerance(ShooterPoseConstants.SHOOTER_TILT_GOAL_TOLERANCE_DEGREES);

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
                ShooterPoseConstants.SHOOTER_HEIGHT_MAX_ACCELERATION),
            ShooterPoseConstants.SHOOTER_PID_PERIOD_SEC);
    shooterHeightPID.setTolerance(ShooterPoseConstants.SHOOTER_HEIGHT_GOAL_TOLERANCE_INCHES);

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

  public void setShooterDistance(double distanceInches) {
    shooterHeightPID.reset(distanceInches);
    shooterHeightPID.setGoal(ShooterPoseConstants.SHOOTER_POSE_HANDOFF_SETPOINT);
    double speakerHeight = 83, shooterHeight = 27;
    double targetAngle =
        -1 * Math.toDegrees(Math.atan((speakerHeight - shooterHeight) / distanceInches));
    shooterTiltPID.setGoal(targetAngle);
  }

  public void setShooterPose(Pose pose) {
    switch (pose) {
      case HANDOFF:
        shooterHeightPID.setGoal(ShooterPoseConstants.SHOOTER_POSE_HANDOFF_SETPOINT);
        shooterTiltPID.setGoal(ShooterPoseConstants.SHOOTER_TILT_HANDOFF_SETPOINT);
        break;

      case SUBWOOFER:
        shooterHeightPID.setGoal(ShooterPoseConstants.SHOOTER_POSE_SUBWOOFER_SETPOINT);
        shooterTiltPID.setGoal(ShooterPoseConstants.SHOOTER_TILT_SUBWOOFER_SETPOINT);
        break;

      case AMP:
        shooterHeightPID.setGoal(ShooterPoseConstants.SHOOTER_POSE_AMP_SETPOINT);
        shooterTiltPID.setGoal(ShooterPoseConstants.SHOOTER_TILT_AMP_SETPOINT);
        break;

      case TRAP:
        shooterHeightPID.setGoal(ShooterPoseConstants.SHOOTER_POSE_TRAP_SETPOINT);
        shooterTiltPID.setGoal(ShooterPoseConstants.SHOOTER_TILT_TRAP_SETPOINT);
        break;

      case SOURCE:
        shooterHeightPID.setGoal(ShooterPoseConstants.SHOOTER_POSE_SOURCE_SETPOINT);
        shooterTiltPID.setGoal(ShooterPoseConstants.SHOOTER_TILT_SOURCE_SETPOINT);
        break;
    }
  }

  private void setUpShuffleboard() {
    shooterHeightP = new TunableNumber("Elevator Height P", ShooterPoseConstants.SHOOTER_HEIGHT_KP);
    shooterHeightI = new TunableNumber("Elevator Height I", ShooterPoseConstants.SHOOTER_HEIGHT_KI);
    shooterHeightD = new TunableNumber("Elevator Height D", ShooterPoseConstants.SHOOTER_HEIGHT_KD);

    shooterTiltP = new TunableNumber("Shooter Tilt P", ShooterPoseConstants.SHOOTER_TILT_KP);
    shooterTiltI = new TunableNumber("Shooter Tilt I", ShooterPoseConstants.SHOOTER_TILT_KI);
    shooterTiltD = new TunableNumber("Shooter Tilt D", ShooterPoseConstants.SHOOTER_TILT_KD);

    shooterPoseTab = Shuffleboard.getTab("Elevator");

    //ShooterPoseTab.add("Elevator Height P", shooterHeightP);
    //ShooterPoseTab.add("Elevator Height I", shooterHeightI);
    //ShooterPoseTab.add("Elevator Height D", shooterHeightD);

    //ShooterPoseTab.add("Shooter Tilt P", shooterTiltP);
    //ShooterPoseTab.add("Shooter Tilt I", shooterTiltI);
    //ShooterPoseTab.add("Shooter Tilt D", shooterTiltD);
  }

  public void periodic() {
    // if (shooterHeightLimitSwitch.isPressed()) {
    //   shooterHeightEncoder.setPosition(0);
    // } else if (shooterHeightPID.atGoal() && shooterHeightPID.getGoal().position == 0) {
    //   // move elevator manually
    // }

    /*shooterHeightRightMotor.set(
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
    */
  }
}
