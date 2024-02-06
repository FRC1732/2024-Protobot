package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;

public class Elevator extends SubsystemBase {

  private CANSparkMax ElevatorLeftMotor;
  private CANSparkMax ElevatorRightMotor;

  private PIDController ElevatorRightMotorPID;

  private ShuffleboardTab ElevatorTab;

  private TunableNumber ElevatorRightMotorP;
  private TunableNumber ElevatorRightMotorI;
  private TunableNumber ElevatorRightMotorD;

  public Elevator() {
    ElevatorLeftMotor =
        new CANSparkMax(ElevatorConstants.ELEVATOR_LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
    ElevatorRightMotor =
        new CANSparkMax(ElevatorConstants.ELEVATOR_RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);

    ElevatorLeftMotor.setInverted(true);
    ElevatorLeftMotor.follow(ElevatorRightMotor);

    ElevatorRightMotorPID =
        new PIDController(
            ElevatorConstants.ELEVATOR_RIGHT_MOTOR_P,
            ElevatorConstants.ELEVATOR_RIGHT_MOTOR_I,
            ElevatorConstants.ELEVATOR_RIGHT_MOTOR_D);

    ElevatorRightMotorPID.setSetpoint(ElevatorConstants.ELEVATOR_SPEAKER_SETPOINT);

    if (ElevatorConstants.ELEVATOR_TESTING) {
      setUpShuffleboard();
    }
  }

  public void setElevatorSetpointSpeaker() {
    ElevatorRightMotorPID.setSetpoint(ElevatorConstants.ELEVATOR_SPEAKER_SETPOINT);
  }

  public void setElevatorSetpointAmp() {
    ElevatorRightMotorPID.setSetpoint(ElevatorConstants.ELEVATOR_AMP_SETPOINT);
  }

  public void setElevatorSetpointTrap() {
    ElevatorRightMotorPID.setSetpoint(ElevatorConstants.ELEVATOR_TRAP_SETPOINT);
  }

  private void setUpShuffleboard() {
    ElevatorRightMotorP =
        new TunableNumber("Elevator Right Motor P", ElevatorConstants.ELEVATOR_RIGHT_MOTOR_P);
    ElevatorRightMotorI =
        new TunableNumber("Elevator Right Motor I", ElevatorConstants.ELEVATOR_RIGHT_MOTOR_I);
    ElevatorRightMotorD =
        new TunableNumber("Elevator Right Motor D", ElevatorConstants.ELEVATOR_RIGHT_MOTOR_D);

    ElevatorTab = Shuffleboard.getTab("Elevator");

    ElevatorTab.add("Elevator Right Motor P", ElevatorRightMotorP);
    ElevatorTab.add("Elevator Right Motor I", ElevatorRightMotorI);
    ElevatorTab.add("Elevator Right Motor D", ElevatorRightMotorD);
  }

  public void periodic() {
    ElevatorRightMotor.set(
        ElevatorRightMotorPID.calculate(
            ElevatorRightMotor.getEncoder().getPosition(), ElevatorRightMotorPID.getSetpoint()));

    if (ElevatorConstants.ELEVATOR_TESTING) {
      ElevatorRightMotorPID.setP(ElevatorRightMotorP.get());
      ElevatorRightMotorPID.setI(ElevatorRightMotorI.get());
      ElevatorRightMotorPID.setD(ElevatorRightMotorD.get());
    }
  }
}