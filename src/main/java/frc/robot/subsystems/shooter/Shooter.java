package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;


public class Shooter extends SubsystemBase { 
  private CANSparkFlex shooterHighMotor;
  private CANSparkFlex shooterLowMotor;
  private CANSparkMax shooterTiltMotor;

  private ShooterSetpoint shooterSetpoint;

  private PIDController shooterTiltPID;

  private TunableNumber shooterTiltP;
  private TunableNumber shooterTiltI;
  private TunableNumber shooterTiltD;

  private ShuffleboardTab shooterTab;

  public Shooter() {
    shooterHighMotor = new CANSparkFlex(ShooterConstants.SHOOTER_HIGH_MOTOR_CAN_ID, CANSparkMax.MotorType.kBrushless);
    shooterLowMotor = new CANSparkFlex(ShooterConstants.SHOOTER_LOW_MOTOR_CAN_ID, CANSparkMax.MotorType.kBrushless);
    shooterTiltMotor = new CANSparkMax(ShooterConstants.SHOOTER_TILT_MOTOR_CAN_ID, CANSparkMax.MotorType.kBrushless);

    shooterHighMotor.setInverted(ShooterConstants.SHOOTER_HIGH_MOTOR_INVERTED);
    shooterHighMotor.follow(shooterLowMotor);

    shooterSetpoint = ShooterSetpoint.RANGE_115;

    shooterTiltPID = new PIDController(ShooterConstants.SHOOTER_TILT_P, ShooterConstants.SHOOTER_TILT_I, ShooterConstants.SHOOTER_TILT_D);

    if (TESTING) {
      setUpShuffleBoard();
    }
  }

  public void setShooterSpeed115() {
    shooterSetpoint = ShooterSetpoint.RANGE_115;
    shooterTiltPID.setSetpoint(ShooterConstants.SHOOTER_TILT_SPEAKER_115_SETPOINT);
  }

  public void setShooterSpeed125() {
    shooterSetpoint = ShooterSetpoint.RANGE_125;
    shooterTiltPID.setSetpoint(ShooterConstants.SHOOTER_TILT_SPEAKER_125_SETPOINT);
  }

  public void setShooterSpeed150() {
    shooterSetpoint = ShooterSetpoint.RANGE_150;
    shooterTiltPID.setSetpoint(ShooterConstants.SHOOTER_TILT_SPEAKER_150_SETPOINT);
  }

  public void rampUpShooter() {
    switch (shooterSetpoint) {
      case RANGE_115:
        shooterHighMotor.set(ShooterConstants.SHOOTER_SPEED_115);
        break;
      case RANGE_125:
        shooterHighMotor.set(ShooterConstants.SHOOTER_SPEED_125);
        break;
      case RANGE_150:
        shooterHighMotor.set(ShooterConstants.SHOOTER_SPEED_150);
        break;
      default:
        shooterHighMotor.set(0);
        break;
    }
  }

  public void rampDownShooter() {
    shooterHighMotor.set(0);
  }

  public void setUpShuffleBoard() {
    shooterTab = Shuffleboard.getTab(SUBSYSTEM_NAME);

    shooterTiltP = new TunableNumber("Shooter Tilt P", ShooterConstants.SHOOTER_TILT_P);
    shooterTiltI = new TunableNumber("Shooter Tilt I", ShooterConstants.SHOOTER_TILT_I);
    shooterTiltD = new TunableNumber("Shooter Tilt D", ShooterConstants.SHOOTER_TILT_D);

    shooterTab.add("Shooter Tilt P", shooterTiltP);
    shooterTab.add("Shooter Tilt I", shooterTiltI);
    shooterTab.add("Shooter Tilt D", shooterTiltD);
  }

  public void periodic() {
    if (TESTING) {
      shooterTiltPID.setP(shooterTiltP.get());
      shooterTiltPID.setI(shooterTiltI.get());
      shooterTiltPID.setD(shooterTiltD.get());
    }

    shooterTiltMotor.set(shooterTiltPID.calculate(shooterTiltMotor.getEncoder().getPosition(), shooterTiltPID.getSetpoint()));
  }
}

enum ShooterSetpoint {
  RANGE_115,
  RANGE_125,
  RANGE_150
}