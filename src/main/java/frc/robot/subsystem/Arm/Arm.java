// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.Arm;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.util.PIDFGains;

public class Arm extends SubsystemBase {
  public static class ArmPostion{
    public double x;
    public double y;
    public double rotation;

    public ArmPostion(double x, double y, double rotation){
      this.x = x;
      this.y = y;
      this.rotation = rotation;
    }

   public ArmPostion(){
      this.x = 0;
      this.y = 0;
      this.rotation = 0;
    }

    public ArmPostion(Pose2d pose){
      this.x = pose.getX();
      this.y = pose.getY();
      this.rotation = pose.getRotation().getRadians();
    }

    public Pose2d toPose2d(){
      return new Pose2d(x, y, Rotation2d.fromRadians(rotation));
    }
  }

  /** Creates a new Arm. */

  private static Arm instance;

  private CANSparkFlex mainMotor;
  private CANSparkFlex seconderyMotor;

  private ArmInputsAutoLogged inputs;

  private ArmPostion intakePostion;
  private ArmPostion shooterPostion;

  @AutoLog
  public static class ArmInputs{
    double mainMotorPostion;
    double seconderyMotorPosition;

    double mainMotorAbsolutePostion;
    double seconderyAbsolutePostion;

    double mainCurrent;
    double seconderyCurrent;
  }

  public static Arm getInstance(){
    if(instance == null) instance = new Arm();
    return instance;
  }

  private Arm() {
    mainMotor = configureMotors(Constants.ArmConstants.MotorConstants.mainMotorID, Constants.ArmConstants.MotorConstants.mainZeroOffset ,Constants.ArmConstants.MotorConstants.mainPID,
    Constants.ArmConstants.MotorConstants.mainInverted, Constants.ArmConstants.MotorConstants.mainConvertionFactor, Constants.ArmConstants.MotorConstants.mainSoftLimits);

    seconderyMotor = configureMotors(Constants.ArmConstants.MotorConstants.seconderyMotorID, Constants.ArmConstants.MotorConstants.seconderyZeroOffset, Constants.ArmConstants.MotorConstants.seconderyPID,
    Constants.ArmConstants.MotorConstants.seconderyInverted, Constants.ArmConstants.MotorConstants.seconderyConvecrtionFactor, Constants.ArmConstants.MotorConstants.seconderySoftLimits);

    inputs = new ArmInputsAutoLogged();

    intakePostion = new ArmPostion();
    shooterPostion = new ArmPostion();
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
    shooterPostion.x = Math.cos(Units.rotationsToRadians(inputs.mainMotorPostion)) * Constants.ArmConstants.armLengthMeters - ArmConstants.mainPivotDistanceFromCenterMeters;
    shooterPostion.y = Math.sin(Units.rotationsToRadians(inputs.mainMotorPostion)) * Constants.ArmConstants.armLengthMeters + ArmConstants.armHeightFromFrameMeters;
    shooterPostion.rotation = Units.rotationsToRadians(inputs.mainMotorPostion + 1 - inputs.seconderyMotorPosition);

    intakePostion.x = shooterPostion.x +
    Math.sin(Units.rotationsToRadians(inputs.seconderyMotorPosition) - (Math.PI / 2) - Units.rotationsToRadians(inputs.mainMotorPostion)) * ArmConstants.shooterAndIntakeLengthMeters;
    intakePostion.y = shooterPostion.y + 
    Math.cos(Units.rotationsToRadians(inputs.seconderyMotorPosition) - (Math.PI / 2) - Units.rotationsToRadians(inputs.mainMotorPostion)) * ArmConstants.shooterAndIntakeLengthMeters;
    intakePostion.rotation = shooterPostion.rotation;
  }

  public ArmPostion getShooterPostion(){
    return shooterPostion;
  }

  public ArmPostion getIntakePostion(){
    return intakePostion;
  }

  public void moveShooterToPose(ArmPostion position){
    // Assuming that the units are in rotations
    mainMotor.getPIDController().setReference(Units.radiansToRotations(Math.acos(position.x + ArmConstants.mainPivotDistanceFromCenterMeters / ArmConstants.armLengthMeters)),
    ControlType.kPosition);
  }

  public void moveIntakeToPose(ArmPostion postion){
    seconderyMotor.getPIDController().setReference(Units.radiansToRotations(Math.asin((postion.x - shooterPostion.x) / ArmConstants.shooterAndIntakeLengthMeters)),
    ControlType.kPosition);
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
