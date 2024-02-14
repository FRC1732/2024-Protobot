package frc.robot.subsystems.shooterWheels;

import static frc.robot.subsystems.shooterWheels.ShooterWheelsConstants.*;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;

public class ShooterWheels extends SubsystemBase {
  private CANSparkFlex shooterHighMotor;
  private CANSparkFlex shooterLowMotor;

  private ShuffleboardTab shooterWheelsTab;

  private TunableNumber shooterSpeedBackwards;
  private TunableNumber shooterSpeedSlow;
  private TunableNumber shooterSpeedFast;
  private TunableNumber shooterSpeedStopped;

  public ShooterWheels() {
    shooterHighMotor =
        new CANSparkFlex(ShooterWheelsConstants.SHOOTER_HIGH_MOTOR_CAN_ID, MotorType.kBrushless);
    shooterLowMotor =
        new CANSparkFlex(ShooterWheelsConstants.SHOOTER_LOW_MOTOR_CAN_ID, MotorType.kBrushless);

    shooterLowMotor.follow(shooterHighMotor, true);

    shooterSpeedBackwards =
        new TunableNumber(
            "Shooter Speed Backwards", ShooterWheelsConstants.SHOOTER_SPEED_BACKWARDS);
    shooterSpeedFast =
        new TunableNumber("Shooter Speed High", ShooterWheelsConstants.SHOOTER_SPEED_FAST);
    shooterSpeedSlow =
        new TunableNumber("Shooter Speed Low", ShooterWheelsConstants.SHOOTER_SPEED_SLOW);
    shooterSpeedStopped =
        new TunableNumber("Shooter Speed Stopped", ShooterWheelsConstants.SHOOTER_SPEED_STOPPED);

    if (TESTING) {
      setUpShuffleBoard();
    }
  }

  public void setShooterSpeedFast() {
    shooterHighMotor.set(shooterSpeedFast.get());
  }

  public void setShooterSpeedSlow() {
    shooterHighMotor.set(shooterSpeedSlow.get());
  }

  public void setShooterSpeedBackwards() {
    shooterHighMotor.set(shooterSpeedBackwards.get());
  }

  public void stopShooter() {
    shooterHighMotor.set(shooterSpeedStopped.get());
  }

  public void setShooterSpeed(double speed) {
    shooterHighMotor.set(speed);
  }

  public void setUpShuffleBoard() {
    shooterWheelsTab = Shuffleboard.getTab("Shooter Wheels");

    shooterWheelsTab.add("Shooter Speed Backwards", shooterSpeedBackwards);
    shooterWheelsTab.add("Shooter Speed Slow", shooterSpeedSlow);
    shooterWheelsTab.add("Shooter Speed Fast", shooterSpeedFast);
    shooterWheelsTab.add("Shooter Speed Stopped", shooterSpeedStopped);
  }

  public void periodic() {}
}
