// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.Arm.Shooter;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.util.PIDFGains;

/** Add your docs here. */
public class Shooter {
    @AutoLog
    public static class ShooterInputs{
        double shooterPower;

        double motor1Velocity;
        double motor2Velocity;

        double motor1RPM;
        double motor2RPM;

        double motor1Current;
        double motor2Current;

        double motor1Tempture;
        double motor2Tempture;

        double RPMTarget;
    }
    // initialize motors
    private CANSparkFlex shooterMotor1;
    private CANSparkFlex shooterMotor2;

    private ShooterInputsAutoLogged inputs;

    public Shooter(){
        shooterMotor1 = configMotor(Constants.Shooter.shooter1PID, Constants.Shooter.shooterMotor1ID, Constants.Shooter.shooter1Inverted);
        shooterMotor2 = configMotor(Constants.Shooter.shooter2PID, Constants.Shooter.shooterMotor2ID, Constants.Shooter.shooter2Inverted);

        inputs = new ShooterInputsAutoLogged();

        new Trigger(RobotState::isDisabled).onTrue(new InstantCommand(() -> stopMotors()).ignoringDisable(true));
    }

    /**
     * sets the shooter power
     * @param power power to set the shooter to (precent output)
     */
    public void setShooterPower(double power) {
        power = MathUtil.clamp(power, -Constants.Shooter.shooterMaxPower, Constants.Shooter.shooterMaxPower);

        shooterMotor1.set(power);
        shooterMotor2.set(power);
    }

    public void setShooterRPM(double rpm){
        inputs.RPMTarget = rpm;

        shooterMotor1.getPIDController().setReference(rpm, ControlType.kVelocity);
        shooterMotor2.getPIDController().setReference(rpm, ControlType.kVelocity);
    }

    public void setShooterVelocity(double velocity){
        inputs.RPMTarget = Units.radiansPerSecondToRotationsPerMinute(velocity / Constants.Shooter.wheelRadius);

        SmartDashboard.putNumber("SHooter RPM", inputs.RPMTarget);
        shooterMotor1.getPIDController().setReference(inputs.RPMTarget, ControlType.kVelocity);
        shooterMotor2.getPIDController().setReference(inputs.RPMTarget, ControlType.kVelocity);
    }

    public boolean isAtSelctedVelocity(){
        // return Math.abs((inputs.motor1RPM + inputs.motor2RPM) / 2 - inputs.RPMTarget) <= 250;
        return Math.abs(inputs.motor1RPM - inputs.RPMTarget) <= 65 && Math.abs(inputs.motor2RPM - inputs.RPMTarget) <= 65;
    }
    public boolean isMotorsAtSameSpeed(){
        return Math.abs(inputs.motor1RPM - inputs.motor2RPM) <= 65;
    }

    /**
     * stops the shooter motors
     */
    public void stopMotors(){
        shooterMotor1.stopMotor();
        shooterMotor2.stopMotor();
        inputs.RPMTarget = 0;
        Logger.processInputs("shooter", inputs);
    }

    // // get shooter power
    // public double getShooterVelocity() {//dis is good
    //     // return velocity in meters per second
    //     return Constants.Shooter.RPMforShooterZone1 * Constants.Shooter.wheelRadius
    //      * Constants.Shooter.frictionPowerParameterForGPVelocity;
    // }

    // public double getTrueFullGPVelociti_SideView(){//dis is not good
    //     return Math.hypot(getTrueZAxisVelocity_RobotRelative(), getTrueXAxisVelocity_RobotRelative());
    // }
    /**
     * gets the avrage velocity of the shooter wheels
     * @return avrage velocity in meters per second
     */
    public double getAvrageShooterVelocity(){
        return (inputs.motor1Velocity + inputs.motor2Velocity) / 2;
    }

    public double getTrueXAxisVelocity_RobotRelative(){
        return RobotContainer.driveBase.getChassisSpeeds().vxMetersPerSecond + getAvrageShooterVelocity();
    }

    // public double getTrueZAxisVelocity_RobotRelative(){//dis is not good
    //     return Math.sin(RobotContainer.arm.getShooterPosition().rotation) * getShooterVelocity();
    // }
    
    public double getTrueYAxisVelocity_RobotRelative(){//dis is good
        return RobotContainer.driveBase.getChassisSpeeds().vyMetersPerSecond +
        RobotContainer.driveBase.getChassisSpeeds().omegaRadiansPerSecond * RobotContainer.arm.getIntakePosition().x;
    }
    public double getTrueGamePieceVelocityAngle_RobotRelative_ArialView(){//dis is not good but is needed
        try {
            return Math.atan(getTrueYAxisVelocity_RobotRelative()/getTrueXAxisVelocity_RobotRelative());
        } catch (Exception e) {
            return 0.0;
        }
    }

    /**
     * updates the inputs of the shooter
     */
    public void update(){
        inputs.shooterPower = shooterMotor1.getAppliedOutput();

        inputs.motor1Velocity = Units.rotationsPerMinuteToRadiansPerSecond(shooterMotor1.getEncoder().getVelocity()) * Constants.Shooter.wheelRadius
         * Constants.Shooter.frictionPowerParameterForGPVelocity;

        inputs.motor2Velocity = Units.rotationsPerMinuteToRadiansPerSecond(shooterMotor2.getEncoder().getVelocity()) * Constants.Shooter.wheelRadius
         * Constants.Shooter.frictionPowerParameterForGPVelocity;

        inputs.motor1RPM = shooterMotor1.getEncoder().getVelocity();
        inputs.motor2RPM = shooterMotor2.getEncoder().getVelocity();

        inputs.motor1Current = shooterMotor1.getOutputCurrent();
        inputs.motor2Current = shooterMotor2.getOutputCurrent();

        inputs.motor1Tempture = shooterMotor1.getMotorTemperature();
        inputs.motor2Tempture = shooterMotor2.getMotorTemperature();

        Logger.processInputs("shooter", inputs);
    }

        // configures motor with PID gains
    private CANSparkFlex configMotor(PIDFGains pidGains, int motorID, boolean isInverted){

        CANSparkFlex motor = new CANSparkFlex(motorID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();

        motor.setInverted(isInverted);

        motor.enableVoltageCompensation(12);
        
        SparkPIDController pidController = motor.getPIDController();
        pidController.setP(pidGains.getP());
        pidController.setI(pidGains.getI());
        pidController.setD(pidGains.getD());
        pidController.setFF(pidGains.getF());
        pidController.setIZone(pidGains.getIZone());

        motor.setSmartCurrentLimit(60);
        motor.setSecondaryCurrentLimit(70);
        return motor;
    }
}
