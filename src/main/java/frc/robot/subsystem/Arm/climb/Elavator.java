// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.Arm.climb;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import frc.robot.Constants;
import frc.util.PIDFGains;

public class Elavator {
  @AutoLog
  public static class ElavatorInputs{
    double railMotorPosition;
    double rollerMotorPosition;

    double railMotorTarget;
    double rollerMotorTarget;

    boolean isRailMotorInPosition;
    boolean isRollerMotorInPosition;

    double railMotorCurrent;
    double rollerMotorCurrent;
  }

  private TalonFX railMotor;
  private TalonFX rollerMotor;

  private PositionDutyCycle railoutput;

  private ElavatorInputsAutoLogged inputs;

  /** Creates a new Elavater. */
  public Elavator() {
    railMotor = configMotor(Constants.Elevator.railMotorID, Constants.Elevator.isRailMotorInverted, Constants.Elevator.railMotorPIDF, Constants.Elevator.railMotorConvertionFactor);
    rollerMotor = configMotor(Constants.Elevator.rollerMotorID, Constants.Elevator.isRollerMotorInverted, Constants.Elevator.rollerMotorPIDF, Constants.Elevator.rollerMotorConvertionFactor);

    inputs = new ElavatorInputsAutoLogged();

    railMotor.setPosition(0);

    railoutput = new PositionDutyCycle(0);
    railoutput.EnableFOC = false;
  }

  public void setRailMotor(double height){
    inputs.railMotorTarget = height;
    railMotor.setControl(railoutput.withPosition(inputs.railMotorTarget));
  }

  public double getRailMotorPosition(){
    return inputs.railMotorPosition;
  }

  public double getSlidingMotorPosition(){
    return inputs.rollerMotorPosition;
  }

  public boolean isRailMotorInPosition(){
    return Math.abs(inputs.railMotorPosition - inputs.railMotorTarget) < Constants.Elevator.railMotorTolarance;
  }

  public boolean isRollerMotorInPosition(){
    return Math.abs(inputs.rollerMotorPosition - inputs.rollerMotorTarget) < Constants.Elevator.rollerMotorTolarance;
  }

  public void update(){
    inputs.railMotorPosition = railMotor.getPosition().getValueAsDouble();
    inputs.rollerMotorPosition = rollerMotor.getPosition().getValueAsDouble();

    inputs.isRailMotorInPosition = isRailMotorInPosition();
    inputs.isRollerMotorInPosition = isRollerMotorInPosition();

    inputs.railMotorCurrent = railMotor.getStatorCurrent().getValueAsDouble();
    inputs.rollerMotorCurrent = rollerMotor.getStatorCurrent().getValueAsDouble();

    Logger.processInputs("elavator", inputs);
  }

  private TalonFX configMotor(int motorID, boolean isInverted, PIDFGains pidGains, double convertionFactor){
    TalonFX motor = new TalonFX(motorID);

    TalonFXConfiguration config = new TalonFXConfiguration();

    config.Slot0.kP = pidGains.getP();
    config.Slot0.kI = pidGains.getI();
    config.Slot0.kD = pidGains.getD();
    config.Slot0.kS = pidGains.getF();

    config.Feedback.SensorToMechanismRatio = convertionFactor;

    config.MotorOutput.Inverted = isInverted ? InvertedValue.Clockwise_Positive : InvertedValue.CounterClockwise_Positive;

    motor.getConfigurator().apply(config);

    motor.optimizeBusUtilization();

    motor.getVelocity().setUpdateFrequency(50);
    motor.getStatorCurrent().setUpdateFrequency(50);
    motor.getPosition().setUpdateFrequency(50);

    return motor;
  }
}
