package frc.robot.subsystems.feeder;

import static frc.robot.subsystems.feeder.FeederConstants.*;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.AnalogInput;
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

  public static Double FEEDER_MOTOR_SPEED = .4;
  public static Double FEEDER_MOTOR_RIGHT_SPEED = .8;

  private final AnalogInput analog;

  public Feeder() {

    feederMotor =
        new CANSparkMax(FeederConstants.FEEDER_MOTOR_CAN_ID, CANSparkMax.MotorType.kBrushless);

    feederMotor.setInverted(FeederConstants.SHOOTER_MOTOR_LEFT_INVERTED);

    analog = new AnalogInput(FeederConstants.ANALOG_INPUT_LOCATION);

    // Create a Shuffleboard tab for this subsystem if testing is enabled. Add additional indicators
    // and controls as needed.
    if (TESTING) {
      ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
      tab.add(SUBSYSTEM_NAME, this);

      tab.addBoolean("Has Note", this::hasNote);
      tab.addDouble("Sensor Value", () -> analog.getValue());
    }

    // FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getSystemCheckCommand());
  }

  public void runFeederIn() {
    feederMotor.set(FEEDER_MOTOR_SPEED);
  }

  public void runFeederIntakeSpeedIn() {
    feederMotor.set(FEEDER_MOTOR_SPEED);
  }

  public void runFeederOut() {
    feederMotor.set(FEEDER_MOTOR_SPEED * -1);
  }

  public void stopFeederIn() {
    feederMotor.set(0);
  }

  public boolean hasNote() {
    return analog.getValue() > 300;
  }

  public boolean checkStopped() {
    return feederMotor.get() == 0;
  }

  @Override
  public void periodic() {
    // FEEDER_MOTOR_LEFT_SPEED = feederLEntry.getDouble(0);
    // FEEDER_MOTOR_RIGHT_SPEED = feederREntry.getDouble(0);
    if (TESTING) {
      if (feederMotorSpeed.get() != 0) {
        feederMotor.set(feederMotorSpeed.get());
      }
    }

    if (FeederConstants.FEEDER_LOGGING) {
      updateInputs();
    }
    // FEEDER_MOTOR_LEFT_SPEED = feederLEntry.getDouble(0);
    // FEEDER_MOTOR_RIGHT_SPEED = feederREntry.getDouble(0);
  }

  private void updateInputs() {
    inputs.feederMotorSpeed = feederMotor.get();
    inputs.analogBeamBreakSensor = analog.getValue();

    Logger.processInputs("Feeder", inputs);
  }
}
