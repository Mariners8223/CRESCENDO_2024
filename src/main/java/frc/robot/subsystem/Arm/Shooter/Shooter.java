// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.Arm.Shooter;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.Constants;
import frc.util.PIDFGains;

/** Add your docs here. */
public class Shooter {

    private CANSparkFlex shooterMotor1;
    private CANSparkFlex shooterMotor2;


    public Shooter(){
        shooterMotor1 = configMotor(Constants.ArmConstants.Shooter.shooterPIDGains, Constants.ArmConstants.Shooter.shooterMotor1ID);
        shooterMotor2 = configMotor(Constants.ArmConstants.Shooter.shooterPIDGains, Constants.ArmConstants.Shooter.shooterMotor2ID);
    }


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

    public void setShooterPower(double power) {

        if (Math.abs(power) > Constants.ArmConstants.Shooter.shooterMaxPower) {
            if (power < 0) power = Constants.ArmConstants.Shooter.shooterMaxPower * -1;
            else power = Constants.ArmConstants.Shooter.shooterMaxPower;
        }

        shooterMotor1.getPIDController().setReference(power, CANSparkFlex.ControlType.kPosition);
        shooterMotor2.getPIDController().setReference(power, CANSparkFlex.ControlType.kPosition);
      }

    public double getShooterPower() {
        return 0; // TODO: return the speed of th1e shooter
    }

}
