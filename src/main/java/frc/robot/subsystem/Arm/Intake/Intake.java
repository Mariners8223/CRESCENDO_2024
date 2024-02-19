package frc.robot.subsystem.Arm.Intake;

import frc.robot.Constants;

import java.util.concurrent.TimeUnit;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.armabot.lidar.arcompat.RoboRioPort;
import com.armabot.lidar.impl.vl53l1x.DistanceMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;


public class Intake {
    @AutoLog
    public static class IntakeInputs{
        double motorPower;
        double motorPosition;
        double proximity;
        double current;
    }

    private CANSparkFlex intakeMotor;
    private com.armabot.lidar.impl.vl53l1x.Vl53l1xI2c proxSensor;
    
    private IntakeInputsAutoLogged inputs;

    public Intake() {
        intakeMotor = new CANSparkFlex(Constants.Intake.intakeMotorID, MotorType.kBrushless);
        intakeMotor.setInverted(Constants.Intake.intakeMotorIsInverted);

        intakeMotor.getPIDController().setP(5);

        proxSensor = new com.armabot.lidar.impl.vl53l1x.Vl53l1xI2c(RoboRioPort.MXP);
        proxSensor.getI2c().setAddress((byte) 0x29);
        proxSensor.setTimeout(10, TimeUnit.SECONDS);
        while (true) {
            var error = proxSensor.initialize();
            error.ifPresent(err ->
                DriverStation.reportError("Unable to initialize VL53L1X: " + err, false)
            );
            if (error.isEmpty()) {
                break;
            }
            try {
                Thread.sleep(100);
            } catch (InterruptedException e) {
            }
        }
        proxSensor.setDistanceMode(DistanceMode.SHORT);
        proxSensor.startContinuous(20);

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
        inputs.current = intakeMotor.getOutputCurrent();

        if(proxSensor.dataReady() && !proxSensor.dataReady()) inputs.proximity = proxSensor.read();

        Logger.processInputs("intake", inputs);
    }
}
