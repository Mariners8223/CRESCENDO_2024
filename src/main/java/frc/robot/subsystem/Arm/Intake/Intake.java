package frc.robot.subsystem.Arm.Intake;

import frc.robot.Constants;


import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.armabot.lidar.arcompat.RoboRioPort;
import com.armabot.lidar.impl.vl53l1x.DistanceMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;



public class Intake {
    @AutoLog
    public static class IntakeInputs{
        double motorPower;
        double motorPosition;
        double proximity;
        double current;
    }

    private CANSparkFlex intakeMotor;
    // private com.armabot.lidar.impl.vl53l1x.Vl53l1xI2c proxSensor;
    
    private IntakeInputsAutoLogged inputs;

    public Intake() {
        intakeMotor = new CANSparkFlex(Constants.Intake.intakeMotorID, MotorType.kBrushless);
        intakeMotor.setInverted(Constants.Intake.intakeMotorIsInverted);

        intakeMotor.getPIDController().setP(5);
        // proxSensor = new com.armabot.lidar.impl.vl53l1x.Vl53l1xI2c(RoboRioPort.ONBOARD);
        // proxSensor.getI2c().setAddress((byte) 0x29);
        // proxSensor.setTimeout(10, TimeUnit.SECONDS);
        // while (true) {
        //     var error = proxSensor.initialize();
        //     error.ifPresent(err ->
        //         DriverStation.reportError("Unable to initialize VL53L1X: " + err, false)
        //     );
        //     if (error.isEmpty()) {
        //         break;
        //     }
        //     try {
        //         Thread.sleep(100);
        //     } catch (InterruptedException e) {
        //     }
        // }
        // proxSensor.setDistanceMode(DistanceMode.SHORT);
        // proxSensor.startContinuous(100);

        inputs = new IntakeInputsAutoLogged();
    }

    public double getProximity(){
        return inputs.proximity;
    }

    public boolean isGamePieceDetected(){
        return inputs.proximity <= Constants.Intake.CloseProximity;
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

        // if(proxSensor.dataReady() && !proxSensor.timeoutOccurred()) inputs.proximity = proxSensor.read();

        Logger.processInputs("intake", inputs);
    }
}
