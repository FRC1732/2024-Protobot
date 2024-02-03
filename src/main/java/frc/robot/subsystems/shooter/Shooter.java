package frc.robot.subsystems.shooter;

import static frc.robot.subsystems.shooter.ShooterConstants.*;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;

/**
 * Models a generic subsystem for a rotational mechanism. The other subsystems defined in this
 * library aren't good examples for typical robot subsystems. This class can serve as an example or
 * be used for quick prototyping.
 */
public class Shooter extends SubsystemBase {

  // these Tunables are convenient when testing as they provide direct control of the subsystem's
  // motor
  private final TunableNumber motorPower = new TunableNumber("Shooter/power", 0.0);
  private final TunableNumber motorCurrent = new TunableNumber("Shooter/current", 0.0);
  private final TunableNumber motorPosition = new TunableNumber("Shooter/position", 0.0);

  private CANSparkFlex shooterMotorHigh;
  private CANSparkFlex shooterMotorLow;
  private GenericEntry speedEntryHigh;
  private GenericEntry speedEntryLow;

  // private GenericEntry speedEntryTilt;

  /** Creates a new Shooter. */
  public static Double SHOOTER_MOTOR_HIGH_SPEED = 0.80;

  public static Double SHOOTER_MOTOR_LOW_SPEED = 0.80;
  public static Double SHOOTER_MOTOR_TILT_SPEED = 0.50;

  public Shooter() {

    shooterMotorHigh =
        new CANSparkFlex(
            ShooterConstants.SHOOTER_MOTOR_HIGH_CAN_ID, CANSparkMax.MotorType.kBrushless);
    shooterMotorLow =
        new CANSparkFlex(
            ShooterConstants.SHOOTER_MOTOR_LOW_CAN_ID, CANSparkMax.MotorType.kBrushless);
    // shooterMotorTilt = new CANSparkMax(ShooterConstants.SHOOTER_MOTOR_TILT_CAN_ID,
    // CANSparkMax.MotorType.kBrushless);

    shooterMotorHigh.setInverted(ShooterConstants.SHOOTER_MOTOR_HIGH_INVERTED);
    shooterMotorLow.setInverted(ShooterConstants.SHOOTER_MOTOR_LOW_INVERTED);
    // shooterMotorTilt.setInverted(ShooterConstants.SHOOTER_MOTOR_TILT_INVERTED);
    // setUpShuffleboard();

    // Create a Shuffleboard tab for this subsystem if testing is enabled. Add additional indicators
    // and controls as needed.

    if (TESTING) {
      ShuffleboardTab tab = Shuffleboard.getTab(SUBSYSTEM_NAME);
      tab.add(SUBSYSTEM_NAME, this);
    }

    // FaultReporter.getInstance().registerSystemCheck(SUBSYSTEM_NAME, getSystemCheckCommand());
  }

  /**
   * The subsystem's periodic method needs to update and process the inputs from the hardware
   * interface object.
   */
  public void runShooter() {
    shooterMotorHigh.set(SHOOTER_MOTOR_HIGH_SPEED);
    shooterMotorLow.set(SHOOTER_MOTOR_LOW_SPEED);
  }

  public void stopShooter() {
    shooterMotorHigh.set(0);
    shooterMotorLow.set(0);
  }

  public void setUpShuffleboard() {
    /*ShuffleboardTab shooterMotors;
      shooterMotors = Shuffleboard.getTab("Shooter Motors");
      shooterMotors.addDouble("HighVelocity", () -> shooterMotorHigh.getEncoder().getVelocity());
      shooterMotors.addDouble("LowVelocity", () -> shooterMotorLow.getEncoder().getVelocity());
     // shooterMotors.addDouble("TiltVelocity", () -> shooterMotorTilt.getEncoder().getVelocity());
      //shooterMotors.addDouble("TiltAngle", () -> shooterMotorTilt.getEncoder().getPosition());
      //shooterMotors.addDouble("HighAcceleration", () -> getAcceleration());
      //shooterMotors.addDouble("LowAcceleration", () -> getAcceleration());

      speedEntryHigh = shooterMotors.add("MotorHighSpeed", SHOOTER_MOTOR_HIGH_SPEED ).getEntry();
      speedEntryLow = shooterMotors.add("MotorLowSpeed", SHOOTER_MOTOR_LOW_SPEED ).getEntry();
      //speedEntryTilt = shooterMotors.add("MotorTiltSpeed", SHOOTER_MOTOR_TILT_SPEED ).getEntry();
    */
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SHOOTER_MOTOR_HIGH_SPEED = speedEntryHigh.getDouble(0);
    // SHOOTER_MOTOR_LOW_SPEED = speedEntryLow.getDouble(0);
    // SHOOTER_MOTOR_TILT_SPEED = speedEntryTilt.getDouble(0);
  }

  /*
    private Command getSystemCheckCommand() {
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
