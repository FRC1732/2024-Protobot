package frc.robot.subsystems.feeder;

public class FeederConstants {

  private static final String CONSTRUCTOR_EXCEPTION = "constant class";

  private FeederConstants() {
    throw new IllegalStateException(CONSTRUCTOR_EXCEPTION);
  }

  public static final boolean DEBUGGING = false;
  public static final boolean TESTING = false;
  public static final String SUBSYSTEM_NAME = "Feeder";

  public static final int FEEDER_MOTOR_LEFT_CAN_ID = 51;
  public static final int FEEDER_MOTOR_RIGHT_CAN_ID = 54;
  public static final boolean SHOOTER_MOTOR_LEFT_INVERTED = false;
  public static final boolean SHOOTER_MOTOR_RIGHT_INVERTED = false;
}
