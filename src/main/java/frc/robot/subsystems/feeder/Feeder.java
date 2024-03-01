package frc.robot.subsystems.feeder;

import static frc.robot.subsystems.feeder.FeederConstants.*;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class Feeder extends SubsystemBase {

  @AutoLog
  public static class FeederIOInput {
    double feederMotorSpeed = 0.0;
    double analogBeamBreakSensor = 0.0;
  }

  private FeederIOInputAutoLogged inputs = new FeederIOInputAutoLogged();

  // these Tunables are convenient when testing as they provide direct control of the subsystem's
  // motor
  private final TunableNumber feederMotorSpeed = new TunableNumber("Feeder/leftSpeed", 0.0);

  public final CANSparkMax feederMotor;

  private final AnalogInput analog;

  private double previousValue;

  public Feeder() {

    feederMotor =
        new CANSparkMax(FeederConstants.FEEDER_MOTOR_CAN_ID, CANSparkMax.MotorType.kBrushless);

    feederMotor.restoreFactoryDefaults();
    feederMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    feederMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 500);
    feederMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 600);
    feederMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 700);

    Timer.delay(0.050);
    feederMotor.setInverted(FeederConstants.SHOOTER_MOTOR_LEFT_INVERTED);
    feederMotor.enableVoltageCompensation(12);
    feederMotor.setIdleMode(IdleMode.kBrake);
    feederMotor.stopMotor();

    analog = new AnalogInput(FeederConstants.ANALOG_INPUT_LOCATION);

    previousValue = 0.0;

    // Create a Shuffleboard tab for this subsystem if testing is enabled. Add additional indicators
    // and controls as needed.
    if (TESTING) {
      ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
      tab.add(SUBSYSTEM_NAME, this);

      tab.addBoolean("Has Note", this::hasNote);
      tab.addDouble("Sensor Value", () -> analog.getValue());
    }

    Timer.delay(0.25);
    feederMotor.burnFlash();
    Timer.delay(0.25);

    // FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getSystemCheckCommand());
  }

  public void runFeeder() {
    feederMotor.set(FeederConstants.FEEDER_MOTOR_SPEED);
  }

  public void brakeFeeder() {
    feederMotor.set(FeederConstants.FEEDER_BRAKE_SPEED);
  }

  public void reverseFeeder() {
    feederMotor.set(FeederConstants.FEEDER_MOTOR_SPEED * -1);
  }

  public void stopFeeder() {
    feederMotor.stopMotor();
  }

  public boolean hasNote() {
    double currentValue = analog.getValue();
    double result = (currentValue + previousValue) /2.0;
    previousValue = currentValue;
    return result > 900;
  }

  public boolean checkStopped() {
    return feederMotor.get() == 0;
  }

  @Override
  public void periodic() {
    if (TESTING) {
      if (feederMotorSpeed.get() != 0) {
        feederMotor.set(feederMotorSpeed.get());
      }
    }

    if (FeederConstants.FEEDER_LOGGING) {
      updateInputs();
    }
  }

  private void updateInputs() {
    inputs.feederMotorSpeed = feederMotor.get();
    inputs.analogBeamBreakSensor = analog.getValue();

    Logger.processInputs("Feeder", inputs);
  }
}
