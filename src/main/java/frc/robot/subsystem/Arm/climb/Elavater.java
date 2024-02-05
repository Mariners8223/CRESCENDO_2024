// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.Arm.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elavater extends SubsystemBase {
  private static Elavater instance;

  private TalonFX ClimbingMotor;
  private TalonFX SlidingMotor;
  private TalonFXConfiguration ClimbingMotorConfiguration;
  private TalonFXConfiguration slidingMotorConfiguration;

  /** Creates a new Elavater. */
  private Elavater() {
    ClimbingMotor = new TalonFX(Constants.ClimbConstants.ClimbingMotorID);
    SlidingMotor = new TalonFX(Constants.ClimbConstants.SlidingMotorID);

    ClimbingMotorConfiguration = new TalonFXConfiguration();
    slidingMotorConfiguration = new TalonFXConfiguration();
  }

  public static Elavater getInstance(){
    if(instance == null){
      instance = new Elavater();
    }
    return instance;
  }

  public void SetClimbingMode(double hight){
    //TODO
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
