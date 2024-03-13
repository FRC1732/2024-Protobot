package frc.robot.subsystems.shooterWheels;

import static frc.robot.subsystems.shooterWheels.ShooterWheelsConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class ShooterWheels extends SubsystemBase {

  @AutoLog
  public static class ShooterWheelsIOInput {
    double shooterHighMotorVelocity = 0.0;
    double shooterLowMotorVelocity = 0.0;
  }

  private ShooterWheelsIOInputAutoLogged input = new ShooterWheelsIOInputAutoLogged();

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

    shooterHighMotor.restoreFactoryDefaults();

    // leader
    shooterHighMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    shooterHighMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 50);
    shooterHighMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 60);
    shooterHighMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 70);
    // follower
    shooterLowMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    shooterLowMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    shooterLowMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 600);
    shooterLowMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 700);

    Timer.delay(0.050);

    shooterHighMotor.setInverted(true);
    shooterLowMotor.follow(shooterHighMotor, true);

    shooterHighMotor.enableVoltageCompensation(12);
    shooterHighMotor.setIdleMode(IdleMode.kCoast);
    shooterHighMotor.setOpenLoopRampRate(0.5);
    shooterHighMotor.stopMotor();

    shooterSpeedBackwards =
        new TunableNumber(
            "Shooter Speed Backwards", ShooterWheelsConstants.SHOOTER_SPEED_BACKWARDS);
    shooterSpeedFast =
        new TunableNumber("Shooter Speed High", ShooterWheelsConstants.SHOOTER_SPEED_FAST);
    shooterSpeedSlow =
        new TunableNumber("Shooter Speed Low", ShooterWheelsConstants.SHOOTER_SPEED_SLOW);
    shooterSpeedStopped =
        new TunableNumber("Shooter Speed Stopped", ShooterWheelsConstants.SHOOTER_SPEED_STOPPED);

    if (SHOOTER_WHEELS_TESTING) {
      setUpShuffleBoard();
    }
    /*Timer.delay(0.25);
    shooterHighMotor.burnFlash();
    Timer.delay(0.25);
    shooterLowMotor.burnFlash();
    Timer.delay(0.25);*/
  }

  public void setShooterSpeedFast() {
    shooterHighMotor.set(ShooterWheelsConstants.SHOOTER_SPEED_FAST);
  }

  public void setShooterSpeedSlow() {
    shooterHighMotor.set(ShooterWheelsConstants.SHOOTER_SPEED_SLOW);
  }

  public void setShooterSpeedBackwards() {
    shooterHighMotor.set(ShooterWheelsConstants.SHOOTER_SPEED_BACKWARDS);
  }

  public void stopShooter() {
    shooterHighMotor.stopMotor();
  }

  public void setShooterSpeed(double speed) {
    shooterHighMotor.set(speed);
  }

  public double getShooterSpeed() {
    return shooterHighMotor.get();
  }

  public boolean isShooterAtFastVelocity() {
    return shooterHighMotor.getEncoder().getVelocity() >= 5400;
  }

  public void setUpShuffleBoard() {
    shooterWheelsTab = Shuffleboard.getTab("Shooter Wheels");

    shooterWheelsTab.add("Current Shooter Speed", shooterHighMotor.get());
    shooterWheelsTab.add("Shooter Speed Backwards", shooterSpeedBackwards);
    shooterWheelsTab.add("Shooter Speed Slow", shooterSpeedSlow);
    shooterWheelsTab.add("Shooter Speed Fast", shooterSpeedFast);
    shooterWheelsTab.add("Shooter Speed Stopped", shooterSpeedStopped);
  }

  public void periodic() {
    if (SHOOTER_WHEELS_LOGGING) {
      updateInputs();
    }
  }

  private void updateInputs() {
    input.shooterHighMotorVelocity = shooterHighMotor.getEncoder().getVelocity();
    input.shooterLowMotorVelocity = shooterLowMotor.getEncoder().getVelocity();

    Logger.processInputs("Shooter Wheels", input);
  }
}
