package frc.robot.subsystems.feeder;

import static frc.robot.subsystems.feeder.FeederConstants.*;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;

public class Feeder extends SubsystemBase {

  // these Tunables are convenient when testing as they provide direct control of the subsystem's
  // motor
  private final TunableNumber leftMotorSpeed = new TunableNumber("Feeder/leftSpeed", 0.0);
  private final TunableNumber rightMotorSpeed = new TunableNumber("Feeder/rightSpeed", 0.0);

  public final CANSparkMax feederMotorLeft;
  public final CANSparkMax feederMotorRight;

  public static Double FEEDER_MOTOR_LEFT_SPEED = .3;
  public static Double FEEDER_MOTOR_RIGHT_SPEED = .3;

  public Feeder() {

    feederMotorLeft =
        new CANSparkMax(FeederConstants.FEEDER_MOTOR_LEFT_CAN_ID, CANSparkMax.MotorType.kBrushed);
    feederMotorRight =
        new CANSparkMax(FeederConstants.FEEDER_MOTOR_RIGHT_CAN_ID, CANSparkMax.MotorType.kBrushed);

    feederMotorLeft.setInverted(FeederConstants.SHOOTER_MOTOR_LEFT_INVERTED);
    feederMotorRight.setInverted(FeederConstants.SHOOTER_MOTOR_RIGHT_INVERTED);

    // Create a Shuffleboard tab for this subsystem if testing is enabled. Add additional indicators
    // and controls as needed.
    if (TESTING) {
      ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
      tab.add(SUBSYSTEM_NAME, this);
    }

    // FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getSystemCheckCommand());
  }

  public void runFeederIn() {
    feederMotorLeft.set(FEEDER_MOTOR_LEFT_SPEED);
    feederMotorRight.set(FEEDER_MOTOR_RIGHT_SPEED);
  }

  public void stopFeederIn() {
    feederMotorLeft.set(0);
    feederMotorRight.set(0);
  }

  @Override
  public void periodic() {
    //FEEDER_MOTOR_LEFT_SPEED = feederLEntry.getDouble(0);
    //FEEDER_MOTOR_RIGHT_SPEED = feederREntry.getDouble(0);
    if (TESTING) {
      if (leftMotorSpeed.get() != 0) {
        feederMotorLeft.set(leftMotorSpeed.get());
      }

      if (rightMotorSpeed.get() != 0) {
        feederMotorRight.set(rightMotorSpeed.get());
      }
    }
    // FEEDER_MOTOR_LEFT_SPEED = feederLEntry.getDouble(0);
    // FEEDER_MOTOR_RIGHT_SPEED = feederREntry.getDouble(0);
  }

  /*
  *  private Command getSystemCheckCommand() {
     return Commands.sequence(
             Commands.run(() -> io.setMotorPower(0.3)).withTimeout(1.0),
             Commands.runOnce(
                 () -> {
                   if (inputs.velocityRPM < 2.0) {
                     FaultReporter.getInstance()
                         .addFault(
                             SUBSYSTEM_NAME,
                             "[System Check] Subsystem motor not moving as fast as expected",
                             false,
                             true);
                   }
                 }),
             Commands.run(() -> io.setMotorPower(-0.2)).withTimeout(1.0),
             Commands.runOnce(
                 () -> {
                   if (inputs.velocityRPM > -2.0) {
                     FaultReporter.getInstance()
                         .addFault(
                             SUBSYSTEM_NAME,
                             "[System Check] Subsystem motor moving too slow or in the wrong direction",
                             false,
                             true);
                   }
                 }))
         .until(() -> !FaultReporter.getInstance().getFaults(SUBSYSTEM_NAME).isEmpty())
         .andThen(Commands.runOnce(() -> io.setMotorPower(0.0)));
   }
  */

}
