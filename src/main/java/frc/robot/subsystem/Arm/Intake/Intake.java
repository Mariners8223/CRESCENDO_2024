package frc.robot.subsystem.Arm.Intake;

import frc.robot.Constants.ArmConstants.IntakeConstants;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class Intake {
    private static Intake instance;

    private CANSparkFlex intakeMotor;
    private ColorSensorV3 colorSensor;

    private Intake() {
        intakeMotor = new CANSparkFlex(IntakeConstants.intakeMotorID, MotorType.kBrushless);
        intakeMotor.setInverted(IntakeConstants.intakeMotorIsInverted);

        colorSensor = new ColorSensorV3(IntakeConstants.ColorSensorPort);
    }

    public static Intake getInstance(){
        if(instance == null){
            instance = new Intake();
        }

        return instance;
    } 

    public double getProximity(){
        return colorSensor.getProximity();
    }

    public boolean isGamePieceDetected(){
        return colorSensor.getProximity() < 0;
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
