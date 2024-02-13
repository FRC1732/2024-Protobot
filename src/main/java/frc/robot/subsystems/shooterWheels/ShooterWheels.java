package frc.robot.subsystems.shooterWheels;

import static frc.robot.subsystems.shooterWheels.ShooterWheelsConstants.*;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;

public class ShooterWheels extends SubsystemBase {
  private CANSparkFlex shooterHighMotor;
  private CANSparkFlex shooterLowMotor;

  private ShooterSpeed shooterSpeed;

  private ShuffleboardTab shooterWheelsTab;

  private TunableNumber shooterSpeedAmp;
  private TunableNumber shooterSpeedTrap;
  private TunableNumber shooterSpeedBackwards;
  private TunableNumber shooterSpeed115;
  private TunableNumber shooterSpeed125;
  private TunableNumber shooterSpeed150;

  public ShooterWheels() {
    shooterHighMotor =
        new CANSparkFlex(
            ShooterWheelsConstants.SHOOTER_HIGH_MOTOR_CAN_ID, MotorType.kBrushless);
    shooterLowMotor =
        new CANSparkFlex(
            ShooterWheelsConstants.SHOOTER_LOW_MOTOR_CAN_ID, MotorType.kBrushless);

    shooterLowMotor.follow(shooterHighMotor, true);
    

    shooterSpeed = ShooterSpeed.RANGE_115;

    shooterSpeedAmp =
        new TunableNumber("Shooter Speed Amp", ShooterWheelsConstants.SHOOTER_SPEED_AMP);
    shooterSpeedTrap =
        new TunableNumber("Shooter Speed Trap", ShooterWheelsConstants.SHOOTER_SPEED_TRAP);
    shooterSpeedBackwards =
        new TunableNumber(
            "Shooter Speed Backwards", ShooterWheelsConstants.SHOOTER_SPEED_BACKWARDS);
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

  public void setShooterSpeed(ShooterSpeed speed) {
    shooterHighMotor.set(SHOOTER_SPEED_115);
    shooterSpeed = speed;
  }

   public void setShooterSpeedTesting() {
    shooterHighMotor.set(-SHOOTER_SPEED_115);
  }

   public void setShooterSpeedTesting2() {
    shooterLowMotor.set(-SHOOTER_SPEED_115);
  }

  @Deprecated(since = "use setShooterSpeed(ShooterSpeed speed) instead")
  public void setShooterSpeedAmp() {
    shooterSpeed = ShooterSpeed.RANGE_AMP;
    rampUpShooter();
  }

  @Deprecated(since = "use setShooterSpeed(ShooterSpeed speed) instead")
  public void setShooterSpeed115() {
    shooterSpeed = ShooterSpeed.RANGE_115;
    rampUpShooter();
  }

  @Deprecated(since = "use setShooterSpeed(ShooterSpeed speed) instead")
  public void setShooterSpeed125() {
    shooterSpeed = ShooterSpeed.RANGE_125;
    rampUpShooter();
  }

  @Deprecated(since = "use setShooterSpeed(ShooterSpeed speed) instead")
  public void setShooterSpeed150() {
    shooterSpeed = ShooterSpeed.RANGE_150;
    rampUpShooter();
  }

  public void rampUpShooter() {
    if (ShooterWheelsConstants.TESTING) {
      switch (shooterSpeed) {
        case RANGE_115:
          shooterHighMotor.set(shooterSpeed115.get());
          break;
        case RANGE_125:
          shooterHighMotor.set(shooterSpeed125.get());
          break;
        case RANGE_150:
          shooterHighMotor.set(shooterSpeed150.get());
          break;
        case RANGE_AMP:
          shooterHighMotor.set(shooterSpeedAmp.get());
          break;
        case RANGE_TRAP:
          shooterHighMotor.set(shooterSpeedTrap.get());
          break;
        case RANGE_BACKWARDS:
          shooterHighMotor.set(shooterSpeedBackwards.get());
          break;
        case STOPPED:
          shooterHighMotor.set(0);
          break;
      }
    } else {
      switch (shooterSpeed) {
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
        case RANGE_TRAP:
          shooterHighMotor.set(ShooterWheelsConstants.SHOOTER_SPEED_TRAP);
          break;
        case RANGE_BACKWARDS:
          shooterHighMotor.set(ShooterWheelsConstants.SHOOTER_SPEED_BACKWARDS);
          break;
        case STOPPED:
          shooterHighMotor.set(0);
          break;
      }
    }
  }

  @Deprecated(since = "use setShooterSpeed(STOPPED) instead")
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
