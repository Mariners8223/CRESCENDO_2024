// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.Arm;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkPIDController;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ArmConstants;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */

  private CANSparkFlex armMotor;
  private CANSparkFlex rotationMotor;

  public Arm() {
    SparkAbsoluteEncoder armAbsoluteEncoder = armMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    SparkAbsoluteEncoder rotationAbsoluteEncoder = rotationMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    armMotor = new CANSparkFlex(ArmConstants.armRotationMotorID, MotorType.kBrushless);
    rotationMotor = new CANSparkFlex(ArmConstants.rollerRotationMotorID, MotorType.kBrushless);

    armMotor.getEncoder().setPosition(armAbsoluteEncoder.getPosition());
    rotationMotor.getEncoder().setPosition(rotationAbsoluteEncoder.getPosition());

    armMotor.setIdleMode(IdleMode.kBrake);
    rotationMotor.setIdleMode(IdleMode.kBrake);

    armMotor.setInverted(ArmConstants.armRotationInverted);
    rotationMotor.setInverted(ArmConstants.rollerRotationInverted);

    configureMotors();
  }

  private void configureMotors() {
    armMotor.getPIDController().setP(ArmConstants.armRotationPID.getP());
    armMotor.getPIDController().setI(ArmConstants.armRotationPID.getI());
    armMotor.getPIDController().setD(ArmConstants.armRotationPID.getD());

    rotationMotor.getPIDController().setP(ArmConstants.rollerRotationPID.getP());
    rotationMotor.getPIDController().setI(ArmConstants.rollerRotationPID.getI());
    rotationMotor.getPIDController().setD(ArmConstants.rollerRotationPID.getD());
  }

  public void setArmMotorPO(double precent) {
    if(precent > 0 && getArmPosition() >= 90) precent = 0;
    if(precent < 0 && getArmPosition() <= 0) precent = 0;

    precent = MathUtil.clamp(precent, -0.6, 0.6);
    armMotor.set(precent);
  }

  public void setRotationMotor(double precent) {
    if(precent > 0 && getRotationPosition() >= 180) precent = 0;
    if(precent < 0 && getRotationPosition() <= 0) precent = 0;

    precent = MathUtil.clamp(precent, -0.6, 0.6);
    rotationMotor.set(precent);
  }

  public void setArmMotorDegrees(double degrees) {
    degrees = MathUtil.clamp(degrees, 0, 90);
    armMotor.getPIDController().setReference(degrees, ControlType.kPosition);
  }

  public void setRotationMotorDegrees(double degrees) {
    degrees = MathUtil.clamp(degrees, 0, 180);
    rotationMotor.getPIDController().setReference(degrees, ControlType.kPosition);
  }

  public double getArmPosition() {
    return armMotor.getEncoder().getPosition();
  }

  public double getRotationPosition() {
    return rotationMotor.getEncoder().getPosition();
  }

  public void stopArmMotor() {
    armMotor.disable();
  }

  public void stallArmMotor() {
    armMotor.setIdleMode(IdleMode.kBrake);
  }

  public void stopRotationMotor() {
    rotationMotor.disable();
  }

  public void stallRotationMotor() {
    rotationMotor.setIdleMode(IdleMode.kBrake);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
