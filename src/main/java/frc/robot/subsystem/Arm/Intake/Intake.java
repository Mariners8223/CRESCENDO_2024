package frc.robot.subsystem.Arm.Intake;

import frc.robot.Constants.ArmConstants.IntakeConstants;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class Intake {
    private CANSparkFlex intakeMotor;
    private ColorSensorV3 colorSensor;

    public Intake() {
        intakeMotor = new CANSparkFlex(IntakeConstants.intakeMotorID, MotorType.kBrushless);
        intakeMotor.setInverted(IntakeConstants.intakeMotorIsInverted);

        colorSensor = new ColorSensorV3(IntakeConstants.ColorSensorPort);
    }

    public double getProximity(){
        return colorSensor.getProximity();
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
}
