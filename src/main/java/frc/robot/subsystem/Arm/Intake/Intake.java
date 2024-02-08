package frc.robot.subsystem.Arm.Intake;

import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkLowLevel.MotorType;


public class Intake {
    private CANSparkFlex intakeMotor;
    private ColorSensorV3 colorSensor;

    public Intake() {
        intakeMotor = new CANSparkFlex(ArmConstants.Intake.intakeMotorID, MotorType.kBrushless);
        intakeMotor.setInverted(ArmConstants.Intake.intakeMotorIsInverted);

        colorSensor = new ColorSensorV3(ArmConstants.Intake.ColorSensorPort);
    }

    public double getProximity(){
        return colorSensor.getProximity();
    }

    public boolean isGamePieceDetected(){
        return colorSensor.getProximity() > Constants.ArmConstants.Intake.CloseProximity;
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
