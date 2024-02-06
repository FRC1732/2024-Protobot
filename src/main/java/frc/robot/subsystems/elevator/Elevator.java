package frc.robot.subsystems.elevator;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;

public class Elevator extends SubsystemBase{

    private CANSparkMax ElevatorLeftMotor;
    private CANSparkMax ElevatorRightMotor;

    private PIDController ElevatorLeftMotorPID;
    private PIDController ElevatorRightMotorPID;

    private ShuffleboardTab ElevatorTab;
    
    private TunableNumber ElevatorLeftMotorP;
    private TunableNumber ElevatorLeftMotorI;
    private TunableNumber ElevatorLeftMotorD;

    private TunableNumber ElevatorRightMotorP;
    private TunableNumber ElevatorRightMotorI;
    private TunableNumber ElevatorRightMotorD;

    public Elevator() {
        ElevatorLeftMotor = new CANSparkMax(ElevatorConstants.ELEVATOR_LEFT_MOTOR_CAN_ID, MotorType.kBrushless);
        ElevatorRightMotor = new CANSparkMax(ElevatorConstants.ELEVATOR_RIGHT_MOTOR_CAN_ID, MotorType.kBrushless);

        ElevatorLeftMotorP = new TunableNumber("Elevator Left Motor P", ElevatorConstants.ELEVATOR_LEFT_MOTOR_P);
        ElevatorLeftMotorI = new TunableNumber("Elevator Left Motor I", ElevatorConstants.ELEVATOR_LEFT_MOTOR_I);
        ElevatorLeftMotorD = new TunableNumber("Elevator Left Motor D", ElevatorConstants.ELEVATOR_LEFT_MOTOR_D);

        ElevatorRightMotorP = new TunableNumber("Elevator Right Motor P", ElevatorConstants.ELEVATOR_RIGHT_MOTOR_P);
        ElevatorRightMotorI = new TunableNumber("Elevator Right Motor I", ElevatorConstants.ELEVATOR_RIGHT_MOTOR_I);
        ElevatorRightMotorD = new TunableNumber("Elevator Right Motor D", ElevatorConstants.ELEVATOR_RIGHT_MOTOR_D);

        ElevatorLeftMotorPID = new PIDController(ElevatorLeftMotorP.get(), ElevatorLeftMotorI.get(), ElevatorLeftMotorD.get());
        ElevatorRightMotorPID = new PIDController(ElevatorRightMotorP.get(), ElevatorRightMotorI.get(), ElevatorRightMotorD.get());

        
        
        if(ElevatorConstants.ELEVATOR_TESTING) {
            setUpShuffleboard();
        }
    }



    private void setUpShuffleboard() {
        ElevatorTab = Shuffleboard.getTab("Elevator");

        ElevatorTab.add("Elevator Left Motor P", ElevatorLeftMotorP);
        ElevatorTab.add("Elevator Left Motor I", ElevatorLeftMotorI);
        ElevatorTab.add("Elevator Left Motor D", ElevatorLeftMotorD);

        ElevatorTab.add("Elevator Right Motor P", ElevatorRightMotorP);
        ElevatorTab.add("Elevator Right Motor I", ElevatorRightMotorI);
        ElevatorTab.add("Elevator Right Motor D", ElevatorRightMotorD);
    }

    public void periodic() {
        // TODO Auto-generated method stub
        super.periodic();
    }
}