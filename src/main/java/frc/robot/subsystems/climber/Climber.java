package frc.robot.subsystems.climber;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.team6328.util.TunableNumber;

public class Climber extends SubsystemBase{
    
    private CANSparkMax climberLeftMotor;
    private CANSparkMax climberRightMotor;

    private TunableNumber climberLeftP, climberLeftI, climberLeftD;
    private TunableNumber climberRightP, climberRightI, climberRightD;

    private PIDController climberLeftPID, climberRightPID;

    private ShuffleboardTab tab;

    public Climber(){
        climberLeftMotor = new CANSparkMax(ClimberConstants.CLIMBER_LEFT_MOTOR_CAN_ID, CANSparkMax.MotorType.kBrushless);
        climberRightMotor = new CANSparkMax(ClimberConstants.CLIMBER_RIGHT_MOTOR_CAN_ID, CANSparkMax.MotorType.kBrushless);

        climberLeftPID = new PIDController(ClimberConstants.CLIMBER_LEFT_P, ClimberConstants.CLIMBER_LEFT_I, ClimberConstants.CLIMBER_LEFT_D);
        climberRightPID = new PIDController(ClimberConstants.CLIMBER_RIGHT_P, ClimberConstants.CLIMBER_RIGHT_I, ClimberConstants.CLIMBER_RIGHT_D);

        if(ClimberConstants.CLIMBER_TESTING){
            setUpShuffleBoard();
        }
    }

    public void ExtendClimber() {
        climberLeftPID.setSetpoint(ClimberConstants.HIGH_SETPOINT_INCHES);
        climberRightPID.setSetpoint(ClimberConstants.HIGH_SETPOINT_INCHES);
    }

    public void RetractClimber() {
        climberLeftPID.setSetpoint(ClimberConstants.LOW_SETPOINT_INCHES);
        climberRightPID.setSetpoint(ClimberConstants.LOW_SETPOINT_INCHES);
    }

    public void setUpShuffleBoard() {
        tab = Shuffleboard.getTab("Climber");

        climberLeftP = new TunableNumber("Climber Left P", ClimberConstants.CLIMBER_LEFT_P);
        climberLeftI = new TunableNumber("Climber Left I", ClimberConstants.CLIMBER_LEFT_I);
        climberLeftD = new TunableNumber("Climber Left D", ClimberConstants.CLIMBER_LEFT_D);
        climberRightP = new TunableNumber("Climber Right P", ClimberConstants.CLIMBER_RIGHT_P);
        climberRightI = new TunableNumber("Climber Right I", ClimberConstants.CLIMBER_RIGHT_I);
        climberRightD = new TunableNumber("Climber Right D", ClimberConstants.CLIMBER_RIGHT_D);

        tab.add("Climber Left P", climberLeftP);
        tab.add("Climber Left I", climberLeftI);
        tab.add("Climber Left D", climberLeftD);
        tab.add("Climber Right P", climberRightP);
        tab.add("Climber Right I", climberRightI);
        tab.add("Climber Right D", climberRightD);
    }

    @Override
    public void periodic() {
        climberLeftMotor.set(climberLeftPID.calculate(climberLeftMotor.getEncoder().getPosition(), climberLeftPID.getSetpoint()));
        climberRightMotor.set(climberRightPID.calculate(climberRightMotor.getEncoder().getPosition(), climberRightPID.getSetpoint()));

        if(ClimberConstants.CLIMBER_TESTING){
            climberLeftPID.setP(climberLeftP.get());
            climberLeftPID.setI(climberLeftI.get());
            climberLeftPID.setD(climberLeftD.get());
            climberRightPID.setP(climberRightP.get());
            climberRightPID.setI(climberRightI.get());
            climberRightPID.setD(climberRightD.get());
        }
    }
}
