// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.Arm.climb;

import org.littletonrobotics.junction.AutoLog;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.PositionDutyCycle;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Elavator extends SubsystemBase {
  @AutoLog
  public static class ElavatorInputs{
    double railMotorPosition;
    double rollerMotorPosition;

    double railMotorTarget;
    double rollerMotorTarget;
  }

  private TalonFX railMotor;
  private TalonFX rollerMotor;

  private PositionDutyCycle railoutput;

  private ElavatorInputsAutoLogged inputs;

  /** Creates a new Elavater. */
  public Elavator() {
    railMotor = new TalonFX(Constants.ClimbConstants.railMotorID);
    rollerMotor = new TalonFX(Constants.ClimbConstants.rollerMotorID);

    var railMotorConfiguration = new TalonFXConfiguration();
    var rollerMotorConfiguration = new TalonFXConfiguration();

    railMotorConfiguration.Slot0.kP = Constants.ClimbConstants.railMotorPIDF.getP();
    railMotorConfiguration.Slot0.kI = Constants.ClimbConstants.railMotorPIDF.getI();
    railMotorConfiguration.Slot0.kD = Constants.ClimbConstants.railMotorPIDF.getD();
    railMotorConfiguration.Slot0.kS = Constants.ClimbConstants.railMotorPIDF.getF();

    railMotorConfiguration.Feedback.SensorToMechanismRatio = Constants.ClimbConstants.railMotorConvertionFactor;

    railMotorConfiguration.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

    rollerMotorConfiguration.Slot0.kP = Constants.ClimbConstants.rollerMotorPIDF.getP();
    rollerMotorConfiguration.Slot0.kI = Constants.ClimbConstants.rollerMotorPIDF.getI();
    rollerMotorConfiguration.Slot0.kD = Constants.ClimbConstants.rollerMotorPIDF.getD();
    rollerMotorConfiguration.Slot0.kS = Constants.ClimbConstants.rollerMotorPIDF.getF();

    railMotor.getConfigurator().apply(railMotorConfiguration);
    rollerMotor.getConfigurator().apply(rollerMotorConfiguration);

    inputs = new ElavatorInputsAutoLogged();

    railMotor.setPosition(0);

    railoutput = new PositionDutyCycle(0);
    railoutput.EnableFOC = false;
  }

  public void setRailMotor(double height){
    railMotor.setControl(railoutput.withPosition(height));
  }

  @Override
  public void periodic() {
    inputs.railMotorPosition = getRailMotorPosition();
    inputs.rollerMotorPosition = getSlidingMotorPosition();
  }

  public double getRailMotorPosition(){
    return inputs.railMotorPosition;
  }

  public double getSlidingMotorPosition(){
    return inputs.rollerMotorPosition;
  }

  public boolean isRailMotorInPosition(){
    return Math.abs(inputs.railMotorPosition - inputs.railMotorTarget) < Constants.ClimbConstants.railMotorTolarance;
  }

  public boolean isRollerMotorInPosition(){
    return Math.abs(inputs.rollerMotorPosition - inputs.rollerMotorTarget) < Constants.ClimbConstants.rollerMotorTolarance;
  }
}
