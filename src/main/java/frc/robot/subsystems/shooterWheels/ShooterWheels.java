package frc.robot.subsystems.shooterWheels;

import static frc.robot.subsystems.shooterWheels.ShooterWheelsConstants.*;

import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.SparkPIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class ShooterWheels extends SubsystemBase {
  private WheelMode wheelMode = WheelMode.STOP;

  @AutoLog
  public static class ShooterWheelsIOInput {
    double shooterHighMotorVelocity = 0.0;
    double shooterLowMotorVelocity = 0.0;
    double shooterHighMotorPower = 0.0;
    double shooterLowMotorPower = 0.0;
    double shooterHighMotorAppliedOutput = 0.0;
    double shooterLowMotorAppliedOutput = 0.0;
    boolean shooterIsAtFastVelocity = false;
  }

  private ShooterWheelsIOInputAutoLogged input = new ShooterWheelsIOInputAutoLogged();

  private CANSparkFlex shooterHighMotor;
  private CANSparkFlex shooterLowMotor;

  private ShuffleboardTab shooterWheelsTab;

  private SparkPIDController shooterPidController;

  // private TunableNumber shooterSpeedBackwards;
  // private TunableNumber shooterSpeedSlow;
  // private TunableNumber shooterSpeedFast;
  // private TunableNumber shooterSpeedStopped;

  private GenericEntry shooterP;
  private GenericEntry shooterI;
  private GenericEntry shooterD;
  private boolean wasFast;
  public ShooterWheels() {
    shooterHighMotor =
        new CANSparkFlex(ShooterWheelsConstants.SHOOTER_HIGH_MOTOR_CAN_ID, MotorType.kBrushless);
    shooterLowMotor =
        new CANSparkFlex(ShooterWheelsConstants.SHOOTER_LOW_MOTOR_CAN_ID, MotorType.kBrushless);

    shooterHighMotor.restoreFactoryDefaults();

    shooterPidController = shooterHighMotor.getPIDController();

    shooterPidController.setP(ShooterWheelsConstants.SHOOTER_SPEED_P);
    shooterPidController.setI(ShooterWheelsConstants.SHOOTER_SPEED_I);
    shooterPidController.setD(ShooterWheelsConstants.SHOOTER_SPEED_D);
    shooterPidController.setFeedbackDevice(shooterHighMotor.getEncoder());
    shooterPidController.setReference(0, ControlType.kVelocity);
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
    shooterLowMotor.follow(shooterHighMotor, false);

    shooterHighMotor.enableVoltageCompensation(12);
    shooterHighMotor.setIdleMode(IdleMode.kCoast);
    shooterHighMotor.setOpenLoopRampRate(0.5);
    shooterHighMotor.stopMotor();

    wasFast = true;

    /*
     * shooterSpeedBackwards =
     * new TunableNumber(
     * "Shooter Speed Backwards", ShooterWheelsConstants.SHOOTER_SPEED_BACKWARDS);
     * shooterSpeedFast =
     * new TunableNumber("Shooter Speed High",
     * ShooterWheelsConstants.SHOOTER_SPEED_FAST);
     * shooterSpeedSlow =
     * new TunableNumber("Shooter Speed Low",
     * ShooterWheelsConstants.SHOOTER_SPEED_SLOW);
     * shooterSpeedStopped =
     * new TunableNumber("Shooter Speed Stopped",
     * ShooterWheelsConstants.SHOOTER_SPEED_STOPPED);
     */

    if (SHOOTER_WHEELS_TESTING) {
      setUpShuffleBoard();
    }
    /*
     * Timer.delay(0.25);
     * shooterHighMotor.burnFlash();
     * Timer.delay(0.25);
     * shooterLowMotor.burnFlash();
     * Timer.delay(0.25);
     */
  }

  public void setShooterSpeedFast() {
    if(!wasFast){
      shooterPidController.setP(ShooterWheelsConstants.SHOOTER_SPEED_P);
      shooterPidController.setI(ShooterWheelsConstants.SHOOTER_SPEED_I);
      shooterPidController.setD(ShooterWheelsConstants.SHOOTER_SPEED_D);
      wasFast = true;
    }
    // shooterPidController.setP(shooterP.getDouble(ShooterWheelsConstants.SHOOTER_SPEED_P));
    // shooterPidController.setI(shooterI.getDouble(ShooterWheelsConstants.SHOOTER_SPEED_I));
    // shooterPidController.setD(shooterD.getDouble(ShooterWheelsConstants.SHOOTER_SPEED_D));
    wheelMode = WheelMode.FAST;

    shooterPidController.setReference(6200, ControlType.kVelocity);
  }

  public void setShooterSpeedMedium() {
    shooterHighMotor.set(ShooterWheelsConstants.SHOOTER_SPEED_MEDIUM);
    
    wheelMode = WheelMode.MEDIUM;
  }

  public void setShooterSpeedSlow() {
    shooterHighMotor.set(ShooterWheelsConstants.SHOOTER_SPEED_SLOW);
    wheelMode = WheelMode.SLOW;
  }

  public void setShooterSpeedSpoil() {
    if(wasFast){
      shooterPidController.setP(ShooterWheelsConstants.SHOOTER_SLOW_SPEED_P);
      shooterPidController.setI(ShooterWheelsConstants.SHOOTER_SLOW_SPEED_I);
      shooterPidController.setD(ShooterWheelsConstants.SHOOTER_SLOW_SPEED_D);
      wasFast = false;
    }
    shooterPidController.setReference(300, ControlType.kVelocity);
    wheelMode = WheelMode.SPOIL;
  }

  public void setShooterSpeedBackwards() {
    shooterHighMotor.set(ShooterWheelsConstants.SHOOTER_SPEED_BACKWARDS);
    wheelMode = WheelMode.REVERSE;
  }

  public void stopShooter() {
    shooterHighMotor.stopMotor();
    wheelMode = WheelMode.STOP;
  }

  public void setShooterSpeed(double speed) {
    shooterHighMotor.set(speed);
  }

  public double getShooterSpeed() {
    return shooterHighMotor.get();
  }

  public boolean isShooterAtFastVelocity() {
    return shooterHighMotor.getEncoder().getVelocity() >= 5500;
  }

  public boolean isAtSpeed() {
    switch (wheelMode) {
      case FAST:
        return isShooterAtFastVelocity();

      case REVERSE:
      case SLOW:
      case MEDIUM:
        return true;

      case STOP:
      default:
        return false;
    }
  }

  public void setUpShuffleBoard() {
    shooterWheelsTab = Shuffleboard.getTab("Shooter Wheels");

    // TODO: Tunable numbers add themselves. Need to change these to generic entries
    // to be able to add them to shuffleboard here.
    // shooterWheelsTab.add("Current Shooter Speed", shooterHighMotor.get());
    // shooterWheelsTab.add("Shooter Speed Backwards", shooterSpeedBackwards);
    // shooterWheelsTab.add("Shooter Speed Slow", shooterSpeedSlow);
    // shooterWheelsTab.add("Shooter Speed Fast", shooterSpeedFast);
    // shooterWheelsTab.add("Shooter Speed Stopped", shooterSpeedStopped);

    shooterP = shooterWheelsTab.add("Shooter P", ShooterWheelsConstants.SHOOTER_SPEED_P).getEntry();
    shooterI = shooterWheelsTab.add("Shooter I", ShooterWheelsConstants.SHOOTER_SPEED_I).getEntry();
    shooterD = shooterWheelsTab.add("Shooter D", ShooterWheelsConstants.SHOOTER_SPEED_D).getEntry();

    shooterWheelsTab.addDouble(
        "Current Shooter Velocity", () -> shooterHighMotor.getEncoder().getVelocity());
    shooterWheelsTab.addDouble("Current Shooter Power", () -> shooterHighMotor.get());
    // shooterWheelsTab.addDouble("Current Shooter Velocity", () ->
    // shooterHighMotor.getEncoder().getVelocity());
    shooterWheelsTab.addBoolean("Is At Fast Velocity", () -> isShooterAtFastVelocity());
  }

  public void periodic() {

    if (SHOOTER_WHEELS_LOGGING) {
      updateInputs();
    }
  }

  private void updateInputs() {
    input.shooterHighMotorVelocity = shooterHighMotor.getEncoder().getVelocity();
    input.shooterLowMotorVelocity = shooterLowMotor.getEncoder().getVelocity();
    input.shooterHighMotorPower = shooterHighMotor.get();
    input.shooterLowMotorPower = shooterLowMotor.get();
    input.shooterHighMotorAppliedOutput = shooterHighMotor.getAppliedOutput();
    input.shooterLowMotorAppliedOutput = shooterLowMotor.getAppliedOutput();
    input.shooterIsAtFastVelocity = isShooterAtFastVelocity();

    Logger.processInputs("Shooter Wheels", input);
  }

  private enum WheelMode {
    SLOW,
    SPOIL,
    MEDIUM,
    FAST,
    REVERSE,
    STOP;
  }
}
