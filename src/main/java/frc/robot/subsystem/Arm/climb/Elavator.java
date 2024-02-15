// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.Arm.climb;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Elavator{
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

  public void SetClimbingHight(PositionDutyCycle height){
    ClimbingMotor.setControl(height);
  }
  public void SetLocationOnRope(Translation2d target){
    // double MotorRotations = Math.sqrt(Math.pow(RobotContainer.driveBase.getPose().getX() - target.getX(), 2)
    //  + Math.pow(RobotContainer.driveBase.getPose().getY() - target.getY(), 2)) * Constants.ClimbConstants.MotorRotationsToAirialMeters;
    double MotorRotations = Math.hypot(RobotContainer.driveBase.getPose().getX() - target.getX(), RobotContainer.driveBase.getPose().getY() - target.getY())
    * Constants.ClimbConstants.AirialMetersToRopeLength;

    if (target.getY() < RobotContainer.driveBase.getPose().getY()) {
      MotorRotations = -MotorRotations;
    }
    //make talon move MotorRotations rotations.
    moveRobotOnRope(MotorRotations);
  }
  public void moveRobotOnRope(double SpinsToTravel){ //right < 0 left > 0
    SlidingMotor.setControl(new PositionDutyCycle(SpinsToTravel + Constants.ClimbConstants.SlidingMotorOffset));
  }
  
  public void SetSlidingFromRotations(double ArialLength){
    ArialLength = ArialLength * Constants.ClimbConstants.AirialMetersToRopeLength * Constants.ClimbConstants.RopeLengthToMotorRotaions;
    SlidingMotor.setControl(new PositionDutyCycle(ArialLength + Constants.ClimbConstants.SlidingMotorOffset));
    //SlidingMotor.setControl(ControlModeValue.DutyCycleOut).setPosition(length); //move motor said rotations or move motor to said location
  }
  public void HoldInPlace(){
    SlidingMotor.setControl(new PositionDutyCycle(SlidingMotor.getPosition().getValueAsDouble()));
  }

  public double getRollerPosition(){
    return SlidingMotor.getPosition().getValueAsDouble();
  }
}
