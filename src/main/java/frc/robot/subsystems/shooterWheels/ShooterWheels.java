package frc.robot.subsystems.shooterWheels;

import static frc.robot.subsystems.shooterWheels.ShooterWheelsConstants.*;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;

public class ShooterWheels extends SubsystemBase {
  private CANSparkFlex shooterHighMotor;
  private CANSparkFlex shooterLowMotor;

  private ShooterSetpoint shooterSetpoint;

  private ShuffleboardTab shooterWheelsTab;

  private TunableNumber shooterSpeedAmp;
  private TunableNumber shooterSpeed115;
  private TunableNumber shooterSpeed125;
  private TunableNumber shooterSpeed150;

  public ShooterWheels() {
    shooterHighMotor =
        new CANSparkFlex(
            ShooterWheelsConstants.SHOOTER_HIGH_MOTOR_CAN_ID, CANSparkMax.MotorType.kBrushless);
    shooterLowMotor =
        new CANSparkFlex(
            ShooterWheelsConstants.SHOOTER_LOW_MOTOR_CAN_ID, CANSparkMax.MotorType.kBrushless);

    shooterHighMotor.setInverted(ShooterWheelsConstants.SHOOTER_HIGH_MOTOR_INVERTED);
    shooterHighMotor.follow(shooterLowMotor);

    shooterSetpoint = ShooterSetpoint.RANGE_115;

    shooterSpeedAmp = new TunableNumber("Shooter Speed Amp", ShooterWheelsConstants.SHOOTER_SPEED_AMP);
    shooterSpeed115 =
        new TunableNumber("Shooter Speed 115", ShooterWheelsConstants.SHOOTER_SPEED_115);
    shooterSpeed125 =
        new TunableNumber("Shooter Speed 125", ShooterWheelsConstants.SHOOTER_SPEED_125);
    shooterSpeed150 =
        new TunableNumber("Shooter Speed 150", ShooterWheelsConstants.SHOOTER_SPEED_150);

    if (TESTING) {
      setUpShuffleBoard();
    }
  }

  public void setShooterSpeedAmp() {
    shooterSetpoint = ShooterSetpoint.RANGE_AMP;
  }

  public void setShooterSpeed115() {
    shooterSetpoint = ShooterSetpoint.RANGE_115;
  }

  public void setShooterSpeed125() {
    shooterSetpoint = ShooterSetpoint.RANGE_125;
  }

  public void setShooterSpeed150() {
    shooterSetpoint = ShooterSetpoint.RANGE_150;
  }

  public void rampUpShooter() {
    if (ShooterWheelsConstants.TESTING) {
      switch (shooterSetpoint) {
        case RANGE_115:
          shooterHighMotor.set(shooterSpeed115.get());
          break;
        case RANGE_125:
          shooterHighMotor.set(shooterSpeed125.get());
          break;
        case RANGE_150:
          shooterHighMotor.set(shooterSpeed150.get());
          break;
        default:
          shooterHighMotor.set(0);
          break;
      }
    } else {
      switch (shooterSetpoint) {
        case RANGE_AMP:
          shooterHighMotor.set(ShooterWheelsConstants.SHOOTER_SPEED_AMP);
          break;
        case RANGE_115:
          shooterHighMotor.set(ShooterWheelsConstants.SHOOTER_SPEED_115);
          break;
        case RANGE_125:
          shooterHighMotor.set(ShooterWheelsConstants.SHOOTER_SPEED_125);
          break;
        case RANGE_150:
          shooterHighMotor.set(ShooterWheelsConstants.SHOOTER_SPEED_150);
          break;
        default:
          shooterHighMotor.set(0);
          break;
      }
    }
  }

  public void rampDownShooter() {
    shooterHighMotor.set(0);
  }

  public void setUpShuffleBoard() {
    shooterWheelsTab = Shuffleboard.getTab("Shooter Wheels");

    shooterWheelsTab.add("Shooter Speed Amp", shooterSpeedAmp);
    shooterWheelsTab.add("Shooter Speed 115", shooterSpeed115);
    shooterWheelsTab.add("Shooter Speed 125", shooterSpeed125);
    shooterWheelsTab.add("Shooter Speed 150", shooterSpeed150);
  }

  public void periodic() {}
}

enum ShooterSetpoint {
  RANGE_AMP,
  RANGE_115,
  RANGE_125,
  RANGE_150
}
