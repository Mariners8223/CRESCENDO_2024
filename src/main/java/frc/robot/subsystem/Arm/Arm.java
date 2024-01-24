// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.Arm;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.util.PIDFGains;

public class Arm extends SubsystemBase {
  /** Creates a new Arm. */

  private CANSparkFlex mainMotor;
  private CANSparkFlex seconderyMotor;

  private ArmInputsAutoLogged inputs;

  private Pose2d intakePostion;
  private Pose2d shooterPostion;

  @AutoLog
  public static class ArmInputs{
    double mainMotorPostion;
    double seconderyMotorPosition;

    double mainMotorAbsolutePostion;
    double seconderyAbsolutePostion;

    double mainCurrent;
    double seconderyCurrent;
  }

  public Arm() {
    mainMotor = configureMotors(Constants.ArmConstants.MotorConstants.mainMotorID, Constants.ArmConstants.MotorConstants.mainZeroOffset ,Constants.ArmConstants.MotorConstants.mainPID,
    Constants.ArmConstants.MotorConstants.mainInverted, Constants.ArmConstants.MotorConstants.mainConvertionFactor, Constants.ArmConstants.MotorConstants.mainSoftLimits);

    seconderyMotor = configureMotors(Constants.ArmConstants.MotorConstants.seconderyMotorID, Constants.ArmConstants.MotorConstants.seconderyZeroOffset, Constants.ArmConstants.MotorConstants.seconderyPID,
    Constants.ArmConstants.MotorConstants.seconderyInverted, Constants.ArmConstants.MotorConstants.seconderyConvecrtionFactor, Constants.ArmConstants.MotorConstants.seconderySoftLimits);
  }

  public void updateLogger(){
    inputs.mainMotorPostion = mainMotor.getEncoder().getPosition();
    inputs.seconderyMotorPosition = mainMotor.getEncoder().getPosition();

    inputs.mainCurrent = mainMotor.getOutputCurrent();
    inputs.seconderyCurrent = seconderyMotor.getOutputCurrent();

    inputs.mainMotorAbsolutePostion = mainMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition();
    inputs.seconderyAbsolutePostion = seconderyMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition();

    Logger.processInputs(getName(), inputs);
  }

  public void updateArmPostions(){
    shooterPostion = new Pose2d(Constants.ArmConstants.mainPivotDistanceFromCenterMeters -
    Math.cos(Units.rotationsToRadians(inputs.mainMotorPostion)) * Constants.ArmConstants.armLengthMeters -
    Math.cos(Units.rotationsToRadians(inputs.seconderyMotorPosition - inputs.mainMotorPostion)) * Constants.ArmConstants.shooterAndIntakeLengthMeters / 2,
    Math.sin(Units.rotationsToRadians(inputs.mainMotorPostion)) * Constants.ArmConstants.armLengthMeters +
    Math.sin(Units.rotationsToRadians(inputs.seconderyMotorPosition - inputs.mainMotorPostion)) * Constants.ArmConstants.shooterAndIntakeLengthMeters / 2,
    Rotation2d.fromRotations(inputs.mainMotorPostion + 1 - inputs.seconderyMotorPosition));

    intakePostion = new Pose2d(shooterPostion.getX() -
    Math.cos(Units.rotationsToRadians(inputs.seconderyMotorPosition - inputs.mainMotorPostion)) * Constants.ArmConstants.shooterAndIntakeLengthMeters,
    shooterPostion.getY() - Math.sin(Units.rotationsToRadians(inputs.seconderyMotorPosition - inputs.mainMotorPostion)) * Constants.ArmConstants.shooterAndIntakeLengthMeters,
    shooterPostion.getRotation());
  }

  public Pose2d getShooterPostion(){
    return shooterPostion;
  }

  public Pose2d getIntakePostion(){
    return intakePostion;
  }

  public void update(){
    updateLogger();
    updateArmPostions();
  }

  @Override
  public void periodic() {
    update();
    // This method will be called once per scheduler run
  }


  private CANSparkFlex configureMotors(int canID, double zeroOffset, PIDFGains pidfGains, boolean motorInverted, double convertionFactor, double[] softLimit) {
    CANSparkFlex sparkFlex = new CANSparkFlex(canID, MotorType.kBrushless);

    sparkFlex.getPIDController().setP(pidfGains.getP());
    sparkFlex.getPIDController().setI(pidfGains.getI());
    sparkFlex.getPIDController().setD(pidfGains.getD());
    sparkFlex.getPIDController().setIZone(pidfGains.getIZone());

    sparkFlex.setInverted(motorInverted);

    if(softLimit.length > 2) softLimit = new double[]{1, -1};

    sparkFlex.setSoftLimit(SoftLimitDirection.kForward, (float)softLimit[0]);
    sparkFlex.setSoftLimit(SoftLimitDirection.kReverse, (float)softLimit[1]);

    sparkFlex.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).setPositionConversionFactor(1 / (convertionFactor * 1024));
    sparkFlex.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).setZeroOffset(zeroOffset / 360);

    sparkFlex.setIdleMode(IdleMode.kBrake);
    
    sparkFlex.getEncoder().setPosition(sparkFlex.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition());

    sparkFlex.getEncoder().setPositionConversionFactor(1 / convertionFactor);

    return sparkFlex;
  }
}
