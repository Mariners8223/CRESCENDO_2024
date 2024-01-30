package frc.robot.subsystem.Arm.Intake;

import frc.robot.Constants.ArmConstants.IntakeConstants;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class Intake {
    private CANSparkFlex intakeMotor;
    private static Intake instance;

    public Intake() {
        intakeMotor = new CANSparkFlex(IntakeConstants.intakeMotorID, MotorType.kBrushless);
        intakeMotor.setInverted(IntakeConstants.intakeMotorIsInverted);
    }

    public void setMotor(double speed){
        intakeMotor.set(speed);
    }

    public void stopMotor(){
        intakeMotor.stopMotor();
    }

    public double getCurrent(){
        return intakeMotor.getOutputCurrent();
    }

    public static Intake getInstance(){
        if (instance == null){
            instance = new Intake();
        }

        return instance;
    } 
}
