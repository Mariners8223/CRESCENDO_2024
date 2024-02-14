// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.Arm.Shooter;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
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
    }
    // initialize motors
    private CANSparkFlex shooterMotor1;
    private CANSparkFlex shooterMotor2;

    private ShooterInputsAutoLogged inputs;

    public Shooter(){
        shooterMotor1 = configMotor(Constants.Arm.Shooter.shooterPID, Constants.Arm.Shooter.shooterMotor1ID, Constants.Arm.Shooter.shooter1Inverted);
        shooterMotor2 = configMotor(Constants.Arm.Shooter.shooterPID, Constants.Arm.Shooter.shooterMotor2ID, Constants.Arm.Shooter.shooter2Inverted);

        inputs = new ShooterInputsAutoLogged();
    }

    /**
     * sets the shooter power
     * @param power power to set the shooter to (precent output)
     */
    public void setShooterPower(double power) {
        power = MathUtil.clamp(power, -Constants.Arm.Shooter.shooterMaxPower, Constants.Arm.Shooter.shooterMaxPower);

        shooterMotor1.set(power);
        shooterMotor2.set(power);
    }

    /**
     * stops the shooter motors
     */
    public void stopMotors(){
        shooterMotor1.stopMotor();
        shooterMotor2.stopMotor();
    }

    /**
     * gets the linear velocity of the shooter wheel
     * @param motor motor to get the velocity of
     * @return velocity in meters per second
     */
    public double getShooterVelocity(int motor) {
        // return Units.rotationsPerMinuteToRadiansPerSecond(shooterMotor1.getEncoder().getVelocity()) * Constants.Arm.Shooter.wheelRadius
        //  * Constants.Arm.Shooter.frictionPowerParameterForGPVelocity;
        switch (motor) {
            case 1:
                return inputs.motor1Velocity;
        
            case 2:
                return inputs.motor2Velocity;
            default:
                return 0;
        }
    }

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
    public double getTrueYAxisVelocity_RobotRelative(){
        return RobotContainer.driveBase.getChassisSpeeds().vyMetersPerSecond +
        RobotContainer.driveBase.getChassisSpeeds().omegaRadiansPerSecond * RobotContainer.arm.getIntakePosition().x;
    }
    public double getTrueGamePieceVelocityAngle_RobotRelative(){
        return Math.atan(getTrueYAxisVelocity_RobotRelative()/getTrueXAxisVelocity_RobotRelative());
    }

    /**
     * updates the inputs of the shooter
     */
    public void update(){
        inputs.shooterPower = shooterMotor1.getAppliedOutput();

        inputs.motor1Velocity = Units.rotationsPerMinuteToRadiansPerSecond(shooterMotor1.getEncoder().getVelocity()) * Constants.Arm.Shooter.wheelRadius
         * Constants.Arm.Shooter.frictionPowerParameterForGPVelocity;

        inputs.motor2Velocity = Units.rotationsPerMinuteToRadiansPerSecond(shooterMotor2.getEncoder().getVelocity()) * Constants.Arm.Shooter.wheelRadius
         * Constants.Arm.Shooter.frictionPowerParameterForGPVelocity;

        inputs.motor1RPM = shooterMotor1.getEncoder().getVelocity();
        inputs.motor2RPM = shooterMotor2.getEncoder().getVelocity();

        inputs.motor1Current = shooterMotor1.getOutputCurrent();
        inputs.motor2Current = shooterMotor2.getOutputCurrent();

        Logger.processInputs("shooter", inputs);
    }

        // configures motor with PID gains
    private CANSparkFlex configMotor(PIDFGains pidGains, int motorID, boolean isInverted){

        CANSparkFlex motor = new CANSparkFlex(motorID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();

        motor.setInverted(isInverted);
        
        SparkPIDController pidController = motor.getPIDController();
        pidController.setP(pidGains.getP());
        pidController.setI(pidGains.getI());
        pidController.setD(pidGains.getD());
        pidController.setFF(pidGains.getF());
        pidController.setIZone(pidGains.getIZone());
        return motor;
    }
}
