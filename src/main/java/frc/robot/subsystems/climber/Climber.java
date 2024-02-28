package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

public class Climber extends SubsystemBase {

  @AutoLog
  public static class ClimberIOInput {
    double climberLeftMotorSpeed = 0.0;
    double climberRightMotorSpeed = 0.0;
    double climberLeftMotorPosition = 0.0;
    double climberRightMotorPosition = 0.0;
    double climberLeftMotorSetpoint = 0.0;
    double climberRightMotorSetpoint = 0.0;
  }

  private ClimberIOInputAutoLogged inputs = new ClimberIOInputAutoLogged();

  private CANSparkMax climberLeftMotor;
  private CANSparkMax climberRightMotor;

  private GenericEntry climberLeftP, climberLeftI, climberLeftD;
  private GenericEntry climberRightP, climberRightI, climberRightD;
  private PIDController climberLeftPID, climberRightPID;

  private double leftClimberHeight;
  private RelativeEncoder climberLeftEncoder;

  private boolean isClimbing = false;

  private ShuffleboardTab tab;

  public Climber() {
    climberLeftMotor =
        new CANSparkMax(
            ClimberConstants.CLIMBER_LEFT_MOTOR_CAN_ID, CANSparkMax.MotorType.kBrushless);
    climberRightMotor =
        new CANSparkMax(
            ClimberConstants.CLIMBER_RIGHT_MOTOR_CAN_ID, CANSparkMax.MotorType.kBrushless);
    climberLeftMotor.setIdleMode(IdleMode.kBrake);
    climberRightMotor.setIdleMode(IdleMode.kBrake);
    climberLeftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    climberLeftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 50);
    climberLeftMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);

    climberRightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 20);
    climberRightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 50);
    climberRightMotor.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 50);

    climberLeftMotor
        .getEncoder()
        .setPositionConversionFactor(ClimberConstants.CLIMBER_CONVERSION_FACTOR);
    climberLeftEncoder = climberLeftMotor.getEncoder();
    climberLeftEncoder.setPosition(0);

    climberLeftMotor.setInverted(true);

    climberRightMotor.follow(climberLeftMotor, true);

    // climberLeftPID =
    //     new PIDController(
    //         ClimberConstants.CLIMBER_LEFT_P,
    //         ClimberConstants.CLIMBER_LEFT_I,
    //         ClimberConstants.CLIMBER_LEFT_D);
    // climberRightPID =
    //     new PIDController(
    //         ClimberConstants.CLIMBER_RIGHT_P,
    //         ClimberConstants.CLIMBER_RIGHT_I,
    //         ClimberConstants.CLIMBER_RIGHT_D);

    if (ClimberConstants.CLIMBER_TESTING) {
      setUpShuffleBoard();
    }
  }

  public void ClimberUp() {
    climberLeftMotor.set(.3);
    // climberRightMotor.set(.1);

    isClimbing = true;
  }

  public void ClimberDown() {
    climberLeftMotor.set(-.3);
    // climberRightMotor.set(-.1);

    isClimbing = true;
  }

  public void ClimberStop() {
    climberLeftMotor.set(0);
    // climberRightMotor.set(-.1);

    isClimbing = true;
  }

  // public void ExtendClimber() {
  //   climberLeftPID.setSetpoint(ClimberConstants.HIGH_SETPOINT_INCHES);
  //   climberRightPID.setSetpoint(ClimberConstants.HIGH_SETPOINT_INCHES);

  //   isClimbing = true;
  // }

  // public void RetractClimber() {
  //   climberLeftPID.setSetpoint(ClimberConstants.LOW_SETPOINT_INCHES);
  //   climberRightPID.setSetpoint(ClimberConstants.LOW_SETPOINT_INCHES);

  //   isClimbing = true;
  // }

  public void setUpShuffleBoard() {
    tab = Shuffleboard.getTab("Climber");
    tab.addDouble("Climber Height (inches)", () -> climberLeftEncoder.getPosition());
    /*
     * climberLeftP = new TunableNumber("Climber Left P",
     * ClimberConstants.CLIMBER_LEFT_P);
     * climberLeftI = new TunableNumber("Climber Left I",
     * ClimberConstants.CLIMBER_LEFT_I);
     * climberLeftD = new TunableNumber("Climber Left D",
     * ClimberConstants.CLIMBER_LEFT_D);
     * climberRightP = new TunableNumber("Climber Right P",
     * ClimberConstants.CLIMBER_RIGHT_P);
     * climberRightI = new TunableNumber("Climber Right I",
     * ClimberConstants.CLIMBER_RIGHT_I);
     * climberRightD = new TunableNumber("Climber Right D",
     * ClimberConstants.CLIMBER_RIGHT_D);
     */
    // climberLeftP = tab.add("Climber Left P", 0).getEntry();
    // climberLeftI = tab.add("Climber Left I", 0).getEntry();
    // climberLeftD = tab.add("Climber Left D", 0).getEntry();
    // climberRightP = tab.add("Climber Right P", 0).getEntry();
    // climberRightI = tab.add("Climber Right I", 0).getEntry();
    // climberRightD = tab.add("Climber Right D", 0).getEntry();
  }

  @Override
  public void periodic() {
    if (isClimbing && DriverStation.isDisabled()) {
      isClimbing = false;
    }

    leftClimberHeight = climberLeftMotor.getEncoder().getPosition();
    // if (leftClimberHeight >= ClimberConstants.MAX_SETPOINT_INCHES) {
    //   ClimberStop();
    // }
    // if (leftClimberHeight <= ClimberConstants.MIN_SETPOINT_INCHES) {
    //   ClimberStop();
    // }

    /*
     * climberLeftMotor.set(
     * climberLeftPID.calculate(
     * climberLeftMotor.getEncoder().getPosition(), climberLeftPID.getSetpoint()));
     * climberRightMotor.set(
     * climberRightPID.calculate(
     * climberRightMotor.getEncoder().getPosition(),
     * climberRightPID.getSetpoint()));
     *
     * if (ClimberConstants.CLIMBER_TESTING) {
     * climberLeftPID.setP(climberLeftP.getDouble(0));
     * climberLeftPID.setI(climberLeftI.getDouble(0));
     * climberLeftPID.setD(climberLeftD.getDouble(0));
     * climberRightPID.setP(climberRightP.getDouble(0));
     * climberRightPID.setI(climberRightI.getDouble(0));
     * climberRightPID.setD(climberRightD.getDouble(0));
     * }
     */

    if (ClimberConstants.CLIMBER_LOGGING) {
      updateInputs();
    }
  }

  public boolean isClimbing() {
    return isClimbing;
  }

  private void updateInputs() {
    inputs.climberLeftMotorSpeed = climberLeftMotor.get();
    inputs.climberRightMotorSpeed = climberRightMotor.get();
    inputs.climberLeftMotorPosition = climberLeftMotor.getEncoder().getPosition();
    inputs.climberRightMotorPosition = climberRightMotor.getEncoder().getPosition();
    // inputs.climberLeftMotorSetpoint = climberLeftPID.getSetpoint();
    // inputs.climberRightMotorSetpoint = climberRightPID.getSetpoint();

    Logger.processInputs("Climber", inputs);
  }
}
