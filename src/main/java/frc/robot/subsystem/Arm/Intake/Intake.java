package frc.robot.subsystem.Arm.Intake;

import frc.robot.Constants;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;




public class Intake {
    @AutoLog
    public static class IntakeInputs{
        double motorPower;
        double motorPosition;
        boolean WasGamePieceDetected;
        double current;
    }

    private CANSparkFlex intakeMotor;
    private DigitalInput laser;
    
    private IntakeInputsAutoLogged inputs;

    public Intake() {
        intakeMotor = new CANSparkFlex(Constants.Intake.intakeMotorID, MotorType.kBrushless);
        intakeMotor.setInverted(Constants.Intake.intakeMotorIsInverted);

        laser = new DigitalInput(Constants.Intake.laserPort);

        intakeMotor.enableVoltageCompensation(12);

        intakeMotor.getPIDController().setP(5);

        inputs = new IntakeInputsAutoLogged();
        inputs.WasGamePieceDetected = false;
    }

    public boolean getLaserReading(){
        return laser.get();
    }
    public void setIsGamePieceDetected(boolean Detected){
        inputs.WasGamePieceDetected = Detected;
    }

    public boolean isGamePieceDetected(){
        return inputs.WasGamePieceDetected;
    }

    public double getMotorPosition(){
        return inputs.motorPosition;
    }

    public void setPosition(double position){
        intakeMotor.getPIDController().setReference(position, ControlType.kPosition);
    }

    public void setMotor(double speed){
        intakeMotor.set(speed);
    }

    public void stopMotor(){
        intakeMotor.stopMotor();
    }

    public double getCurrent(){
        return inputs.current;
    }

    public void update(){
        inputs.motorPower = intakeMotor.getAppliedOutput();
        inputs.motorPosition = intakeMotor.getEncoder().getPosition();
        inputs.current = intakeMotor.getOutputCurrent();

        Logger.processInputs("intake", inputs);
    }
}
