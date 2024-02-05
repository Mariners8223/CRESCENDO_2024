// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.Arm.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.ControlModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elavator extends SubsystemBase {
  private static Elavator instance;

  private TalonFX ClimbingMotor;
  private TalonFX SlidingMotor;
  private TalonFXConfiguration ClimbingMotorConfiguration;
  private TalonFXConfiguration slidingMotorConfiguration;

  /** Creates a new Elavater. */
  private Elavator() {
    ClimbingMotor = new TalonFX(Constants.ClimbConstants.ClimbingMotorID);
    SlidingMotor = new TalonFX(Constants.ClimbConstants.SlidingMotorID);

    ClimbingMotorConfiguration = new TalonFXConfiguration();
    slidingMotorConfiguration = new TalonFXConfiguration();

    ClimbingMotorConfiguration.Slot0.kP = Constants.ClimbConstants.ClimbingMotorPID.kP;
    ClimbingMotorConfiguration.Slot0.kI = Constants.ClimbConstants.ClimbingMotorPID.kI;
    ClimbingMotorConfiguration.Slot0.kD = Constants.ClimbConstants.ClimbingMotorPID.kD;
    ClimbingMotorConfiguration.Slot0.kS = Constants.ClimbConstants.ClimbingMotorPID.kF;

    slidingMotorConfiguration.Slot0.kP = Constants.ClimbConstants.SlidingMotorPID.kP;
    slidingMotorConfiguration.Slot0.kI = Constants.ClimbConstants.SlidingMotorPID.kI;
    slidingMotorConfiguration.Slot0.kD = Constants.ClimbConstants.SlidingMotorPID.kD;
    slidingMotorConfiguration.Slot0.kS = Constants.ClimbConstants.SlidingMotorPID.kF;

    ClimbingMotor.getConfigurator().apply(ClimbingMotorConfiguration);
    SlidingMotor.getConfigurator().apply(slidingMotorConfiguration);
  }

  public static Elavator getInstance(){
    if(instance == null){
      instance = new Elavator();
    }
    return instance;
  }

  public void SetClimbingMode(double height){
    //TODO
    // ClimbingMotor.setControl(ControlModeValue.PositionDutyCycle);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
