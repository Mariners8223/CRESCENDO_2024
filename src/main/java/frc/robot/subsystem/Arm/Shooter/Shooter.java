// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.Arm.Shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
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

    // set shooter power
    public void setShooterPower(double power) {
        // clamp power to max power
        if (Math.abs(power) > Constants.ArmConstants.Shooter.shooterMaxPower) {
            if (power < 0) power = Constants.ArmConstants.Shooter.shooterMaxPower * -1;
            else power = Constants.ArmConstants.Shooter.shooterMaxPower;
        }

        // set power
        shooterMotor1.getPIDController().setReference(power, CANSparkFlex.ControlType.kPosition);
        shooterMotor2.getPIDController().setReference(power, CANSparkFlex.ControlType.kPosition);
      }
    
    // get shooter power
    public double getShooterPower() {
        // return velocity in meters per second
        return Units.rotationsPerMinuteToRadiansPerSecond(shooterMotor1.getEncoder().getVelocity()) * Constants.ArmConstants.Shooter.wheelRadius;
    }
}
