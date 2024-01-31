// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.Arm;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase;
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
import frc.robot.Constants.ArmConstants;
import frc.robot.subsystem.Arm.Intake.Intake;
import frc.robot.subsystem.Arm.Shooter.Shooter;
import frc.util.PIDFGains;

public class Arm extends SubsystemBase{
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

  public static enum ControlType{
    Xaxis,
    Yaxis,
    Rotation,
  }

  /** Creates a new Arm. */

  private static Arm instance;

  private CANSparkFlex mainMotor;
  private CANSparkFlex seconderyMotor;

  private ArmInputsAutoLogged inputs;

  private ArmPostion intakePostion;
  private ArmPostion shooterPostion;

  private Shooter shooter;
  private Intake intake;

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

  public Shooter getShooter(){
    return shooter;
  }

  public Intake getIntake(){
    return intake;
  }

  private Arm() {
    mainMotor = configureMotors(Constants.ArmConstants.MotorConstants.mainMotorID, Constants.ArmConstants.MotorConstants.mainZeroOffset ,Constants.ArmConstants.MotorConstants.mainPID,
    Constants.ArmConstants.MotorConstants.mainInverted, Constants.ArmConstants.MotorConstants.mainConvertionFactor, Constants.ArmConstants.MotorConstants.mainSoftLimits);

    seconderyMotor = configureMotors(Constants.ArmConstants.MotorConstants.seconderyMotorID, Constants.ArmConstants.MotorConstants.seconderyZeroOffset, Constants.ArmConstants.MotorConstants.seconderyPID,
    Constants.ArmConstants.MotorConstants.seconderyInverted, Constants.ArmConstants.MotorConstants.seconderyConvecrtionFactor, Constants.ArmConstants.MotorConstants.seconderySoftLimits);

    inputs = new ArmInputsAutoLogged();

    intakePostion = new ArmPostion();
    shooterPostion = new ArmPostion();

    shooter = new Shooter();
    intake = new Intake();
  }

  public ArmPostion getShooterPostion(){
    return shooterPostion;
  }

  public ArmPostion getIntakePostion(){
    return intakePostion;
  }

  public void moveShooterToPose(ArmPostion position, ControlType controlType){
    switch (controlType) {
      case Xaxis:
        mainMotor.getPIDController().setReference(Units.radiansToRotations(Math.acos(position.x + ArmConstants.mainPivotDistanceFromCenterMeters / ArmConstants.armLengthMeters)),
        CANSparkBase.ControlType.kPosition);
        break;
      case Yaxis:
        mainMotor.getPIDController().setReference(Units.radiansToRotations(Math.asin(position.y / ArmConstants.armLengthMeters)),
        CANSparkBase.ControlType.kPosition);
        break;
      case Rotation:
        mainMotor.getPIDController().setReference(Units.radiansToRotations(position.rotation), 
        CANSparkBase.ControlType.kPosition);
        break;
      default:
        mainMotor.getPIDController().setReference(Units.radiansToRotations(position.rotation), 
        CANSparkBase.ControlType.kPosition);
        break;
    }
  }

  public void moveIntakeToPose(ArmPostion postion, ControlType controlType){
    switch (controlType) {
      case Xaxis:
        seconderyMotor.getPIDController().setReference(Units.radiansToRotations(Math.asin((postion.x - shooterPostion.x) / ArmConstants.shooterAndIntakeLengthMeters)) + 
        (Math.PI / 2) - Units.degreesToRadians(shooterPostion.rotation),
        CANSparkBase.ControlType.kPosition);
        break;
      case Yaxis:
        seconderyMotor.getPIDController().setReference(Units.radiansToRotations(Math.acos((postion.y - shooterPostion.y) / ArmConstants.shooterAndIntakeLengthMeters) + 
        (Math.PI / 2) - Units.degreesToRadians(shooterPostion.rotation)),
        CANSparkBase.ControlType.kPosition);
        break;
      case Rotation:
        seconderyMotor.getPIDController().setReference(Units.radiansToRotations(postion.rotation + Math.PI),
        CANSparkBase.ControlType.kPosition);
        break;
      default:
        seconderyMotor.getPIDController().setReference(Units.radiansToRotations(postion.rotation + Math.PI),
        CANSparkBase.ControlType.kPosition);
        break;
    }
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
    shooterPostion.rotation = Units.rotationsToRadians(inputs.mainMotorPostion);

    intakePostion.x = shooterPostion.x +
    Math.sin(Units.rotationsToRadians(inputs.seconderyMotorPosition) - (Math.PI / 2) - Units.rotationsToRadians(inputs.mainMotorPostion)) * ArmConstants.shooterAndIntakeLengthMeters;
    intakePostion.y = shooterPostion.y + 
    Math.cos(Units.rotationsToRadians(inputs.seconderyMotorPosition) - (Math.PI / 2) - Units.rotationsToRadians(inputs.mainMotorPostion)) * ArmConstants.shooterAndIntakeLengthMeters;
    intakePostion.rotation = Units.rotationsToRadians(inputs.seconderyMotorPosition) - Math.PI;
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
