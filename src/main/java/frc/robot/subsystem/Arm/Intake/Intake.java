package frc.robot.subsystem.Arm.Intake;

import frc.robot.Constants;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class Intake {
    @AutoLog
    public static class IntakeInputs{
        double motorPower;
        double motorPosition;
        double proximity;
        double current;
    }

    private CANSparkFlex intakeMotor;
    private ColorSensorV3 colorSensor;
    
    private IntakeInputsAutoLogged inputs;

    public Intake() {
        intakeMotor = new CANSparkFlex(Constants.Intake.intakeMotorID, MotorType.kBrushless);
        intakeMotor.setInverted(Constants.Intake.intakeMotorIsInverted);

        intakeMotor.getPIDController().setP(5);

        colorSensor = new ColorSensorV3(Constants.Intake.ColorSensorPort);

        inputs = new IntakeInputsAutoLogged();

        // new Trigger(() -> !colorSensor.isConnected()).debounce(0.5).onTrue(new InstantCommand(() -> colorSensor = new ColorSensorV3(Constants.Intake.ColorSensorPort)).ignoringDisable(true));

        // new Trigger(() -> !colorSensor.isConnected()).whileTrue(new RepeatCommand(new InstantCommand(() -> colorSensor = new ColorSensorV3(Constants.Intake.ColorSensorPort)).ignoringDisable(true)).ignoringDisable(true));
    }

    public double getProximity(){
        return inputs.proximity;
    }

    public boolean isGamePieceDetected(){
        return inputs.proximity > Constants.Intake.CloseProximity;
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
        inputs.proximity = colorSensor.getProximity();
        inputs.current = intakeMotor.getOutputCurrent();

        Logger.processInputs("intake", inputs);
    }
}
