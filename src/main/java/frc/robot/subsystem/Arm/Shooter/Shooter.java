// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.Arm.Shooter;

import com.fasterxml.jackson.core.json.DupDetector;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystem.DriveTrain.DriveBase;
import frc.util.PIDFGains;

/** Add your docs here. */
public class Shooter {
    // initialize motors
    private CANSparkFlex shooterMotor1;
    private CANSparkFlex shooterMotor2;

    // constructor for shooter - configures motors
    public Shooter(){
        shooterMotor1 = configMotor(Constants.ArmConstants.Shooter.shooterPIDGains, Constants.ArmConstants.Shooter.shooterMotor1ID);
        shooterMotor2 = configMotor(Constants.ArmConstants.Shooter.shooterPIDGains, Constants.ArmConstants.Shooter.shooterMotor2ID);
    }

    // set shooter power
    public void setShooterPower(double power) {
        power = MathUtil.clamp(power, -1, 1);

        shooterMotor1.set(power);
        shooterMotor2.set(power);
    }

    public void stopMotors(){
        shooterMotor1.stopMotor();
        shooterMotor2.stopMotor();
    }

    // get shooter power
    public double getShooterVelocity() {
        // return velocity in meters per second
        return Units.rotationsPerMinuteToRadiansPerSecond(shooterMotor1.getEncoder().getVelocity()) * Constants.ArmConstants.Shooter.wheelRadius
         * Constants.ArmConstants.Shooter.frictionPowerParameterForGPVelocity;
    }

    public double getTrueXAxisVelocity_RobotRelative(){
        return RobotContainer.driveBase.getChassisSpeeds().vxMetersPerSecond + getShooterVelocity();
    }
    public double getTrueYAxisVelocity_RobotRelative(){
        return RobotContainer.driveBase.getChassisSpeeds().vyMetersPerSecond +
        RobotContainer.driveBase.getChassisSpeeds().omegaRadiansPerSecond * RobotContainer.arm.getIntakePosition().x;
    }
    public double getTrueGamePieceVelocityAngle_RobotRelative(){
        return Math.atan(getTrueYAxisVelocity_RobotRelative()/getTrueXAxisVelocity_RobotRelative());
    }

        // configures motor with PID gains
    private CANSparkFlex configMotor(PIDFGains pidGains, int motorID){

        CANSparkFlex motor = new CANSparkFlex(motorID, MotorType.kBrushless);
        motor.restoreFactoryDefaults();
        
        SparkPIDController pidController = motor.getPIDController();
        pidController.setP(pidGains.getP());
        pidController.setI(pidGains.getI());
        pidController.setD(pidGains.getD());
        pidController.setFF(pidGains.getF());
        pidController.setIZone(pidGains.getIZone());
        return motor;
    }
}
