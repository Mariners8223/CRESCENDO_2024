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
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.ShooterCommands.AimShooterZone1;
import frc.robot.subsystem.Arm.Intake.Intake;
import frc.robot.subsystem.Arm.Shooter.Shooter;
import frc.robot.subsystem.Arm.climb.Elavator;
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
  
  @AutoLog
  public static class ArmInputs{
    double mainMotorPostion;
    double secondaryMotorPosition;

    double mainMotorTargetPostion;
    double secondaryTargetPostion;

    double mainMotorAbsolutePostion;
    double secondaryAbsolutePostion;

    double mainCurrent;
    double secondaryCurrent;
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

  public static enum ControlType{
    Xaxis,
    Yaxis,
    Rotation,
  }

  /** Creates a new Arm. */

  private static Arm instance;

  private CANSparkFlex mainMotor;
  private CANSparkFlex secondaryMotor;

  private ArmInputsAutoLogged inputs;

  private ArmPostion intakePosition;
  private ArmPostion shooterPosition;
  private ArmPostion climbPosition;

  private Shooter shooter;
  private Intake intake;
  private Elavator climb;


  private Arm() {
    mainMotor = configureMotors(Constants.ArmConstants.MotorConstants.mainMotorID, Constants.ArmConstants.MotorConstants.mainZeroOffset ,Constants.ArmConstants.MotorConstants.mainPID,
    Constants.ArmConstants.MotorConstants.mainInverted, Constants.ArmConstants.MotorConstants.mainConvertionFactor, Constants.ArmConstants.MotorConstants.mainSoftLimits);

    secondaryMotor = configureMotors(Constants.ArmConstants.MotorConstants.seconderyMotorID, Constants.ArmConstants.MotorConstants.seconderyZeroOffset, Constants.ArmConstants.MotorConstants.seconderyPID,
    Constants.ArmConstants.MotorConstants.seconderyInverted, Constants.ArmConstants.MotorConstants.seconderyConvecrtionFactor, Constants.ArmConstants.MotorConstants.seconderySoftLimits);

    inputs = new ArmInputsAutoLogged();

    intakePosition = new ArmPostion();
    shooterPosition = new ArmPostion();
    climbPosition = new ArmPostion();

    shooter = Shooter.getInstance();
    intake = Intake.getInstance();
    climb = Elavator.getInstance();

    createArmTriggers();
  }

  private void createArmTriggers(){
    new Trigger(() -> RobotContainer.getRobotZone() >= 1 && RobotContainer.getRobotZone() <= 3).and(RobotContainer::isAmplified)
    .and(this::isArmInPosition).and(RobotContainer::isRobotSpeakerMode).onTrue(new InstantCommand()); //TODO: add command that shoots, and add robot rotation check

    new Trigger(() -> RobotContainer.getRobotZone() == 1).and(RobotContainer::isRobotSpeakerMode).whileTrue(new RepeatCommand(new AimShooterZone1()));
    new Trigger(() -> RobotContainer.getRobotZone() == 2).and(RobotContainer::isRobotSpeakerMode).whileTrue(new InstantCommand()); //TODO: add command that homes the arm with more advance calculations

    new Trigger(() -> RobotContainer.getRobotZone() == 2 || RobotContainer.getRobotZone() == 3).and(RobotContainer::isRobotAmpMode);

  }

  public ArmPostion getShooterPosition(){
    return shooterPosition;
  }

  public ArmPostion getIntakePosition(){
    return intakePosition;
  }
  public ArmPostion getClimbPosition(){
    return climbPosition;
  }

  public boolean isArmInPosition(){
    return Math.abs(inputs.mainMotorTargetPostion - inputs.mainMotorPostion) < Constants.ArmConstants.MotorConstants.mainMotortolarance &&
    Math.abs(inputs.secondaryMotorPosition - inputs.secondaryTargetPostion) < Constants.ArmConstants.MotorConstants.seconderyMotorTolarance;
  }

  public double getAngleToSpeaker(){
    return 0;
  }

  public void moveShooterToPose(ArmPostion position){
    inputs.mainMotorTargetPostion = Units.radiansToRotations(Math.asin(position.y / ArmConstants.armLengthMeters));
    mainMotor.getPIDController().setReference(inputs.mainMotorTargetPostion, CANSparkBase.ControlType.kPosition);

    inputs.secondaryTargetPostion = inputs.mainMotorTargetPostion + Units.radiansToRotations(position.rotation);
    secondaryMotor.getPIDController().setReference(inputs.secondaryTargetPostion, CANSparkBase.ControlType.kPosition);
  }


  public void moveIntakeToPose(ArmPostion postion, ControlType controlType){
    switch (controlType) {
      case Xaxis:
        secondaryMotor.getPIDController().setReference(Units.radiansToRotations(Math.asin((postion.x - shooterPosition.x) / ArmConstants.shooterAndIntakeLengthMeters)) + 
        (Math.PI / 2) - shooterPosition.rotation,
        CANSparkBase.ControlType.kPosition);
        break;
      case Yaxis:
        secondaryMotor.getPIDController().setReference(Units.radiansToRotations(Math.acos((postion.y - shooterPosition.y) / ArmConstants.shooterAndIntakeLengthMeters)) + 
        (Math.PI / 2) - shooterPosition.rotation,
        CANSparkBase.ControlType.kPosition);
        break;
      case Rotation:
        secondaryMotor.getPIDController().setReference(Units.radiansToRotations(postion.rotation),
        CANSparkBase.ControlType.kPosition);
        break;
      default:
        secondaryMotor.getPIDController().setReference(Units.radiansToRotations(postion.rotation),
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
    inputs.secondaryMotorPosition = mainMotor.getEncoder().getPosition();

    inputs.mainCurrent = mainMotor.getOutputCurrent();
    inputs.secondaryCurrent = secondaryMotor.getOutputCurrent();

    inputs.mainMotorAbsolutePostion = mainMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition();
    inputs.secondaryAbsolutePostion = secondaryMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle).getPosition();

    Logger.processInputs(getName(), inputs);
  }

  public void updateArmPostions(){
    shooterPosition.x = (Math.cos(Units.rotationsToRadians(inputs.mainMotorPostion))) * (Constants.ArmConstants.armLengthMeters) - ArmConstants.mainPivotDistanceFromCenterMeters
    + (Math.sin(Units.rotationsToRadians(inputs.secondaryMotorPosition)) * ArmConstants.SecondaryMotorDistanceFromShooterMeters);
    shooterPosition.y = Math.sin(Units.rotationsToRadians(inputs.mainMotorPostion)) * (Constants.ArmConstants.armLengthMeters) + ArmConstants.armHeightFromFrameMeters
    - (Math.cos(Units.rotationsToRadians(inputs.secondaryMotorPosition))) * ArmConstants.SecondaryMotorDistanceFromShooterMeters;
    shooterPosition.rotation = Units.rotationsToRadians(inputs.mainMotorPostion + inputs.secondaryMotorPosition);

    intakePosition.x = shooterPosition.x +
    Math.sin(Units.rotationsToRadians(inputs.secondaryMotorPosition) - (Math.PI / 2) - Units.rotationsToRadians(inputs.mainMotorPostion)) * ArmConstants.shooterAndIntakeLengthMeters;
    intakePosition.y = shooterPosition.y + 
    Math.cos(Units.rotationsToRadians(inputs.secondaryMotorPosition) - (Math.PI / 2) - Units.rotationsToRadians(inputs.mainMotorPostion)) * ArmConstants.shooterAndIntakeLengthMeters;
    intakePosition.rotation = Units.rotationsToRadians(inputs.secondaryMotorPosition);
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
