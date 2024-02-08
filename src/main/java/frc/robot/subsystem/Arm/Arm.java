// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.Arm;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
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

  /** Creates a new Arm. */

  private static Arm instance;

  private CANSparkFlex mainMotor;
  private CANSparkFlex secondaryMotor;

  private DutyCycleEncoder mainAbsEncoder;
  private DutyCycleEncoder secondaryAbsEncoder;

  private ArmInputsAutoLogged inputs;

  private ArmPostion intakePosition;
  private ArmPostion shooterPosition;

  private Shooter shooter;
  private Intake intake;
  private Elavator climb;


  private Arm() {

    mainAbsEncoder = configureAbsEncoder(Constants.ArmConstants.Motors.mainAbsEncoderID, Constants.ArmConstants.Motors.mainZeroOffset);
    secondaryAbsEncoder = configureAbsEncoder(Constants.ArmConstants.Motors.seconderyAbsEncoderID, Constants.ArmConstants.Motors.seconderyZeroOffset);

    mainMotor = configureMotors(Constants.ArmConstants.Motors.mainMotorID, mainAbsEncoder.get() ,Constants.ArmConstants.Motors.mainPID,
    Constants.ArmConstants.Motors.mainInverted, Constants.ArmConstants.Motors.mainConvertionFactor, Constants.ArmConstants.Motors.mainSoftLimits);

    secondaryMotor = configureMotors(Constants.ArmConstants.Motors.seconderyMotorID, secondaryAbsEncoder.get(), Constants.ArmConstants.Motors.seconderyPID,
    Constants.ArmConstants.Motors.seconderyInverted, Constants.ArmConstants.Motors.seconderyConvecrtionFactor, Constants.ArmConstants.Motors.seconderySoftLimits);

    inputs = new ArmInputsAutoLogged();

    intakePosition = new ArmPostion();
    shooterPosition = new ArmPostion();

    shooter = Shooter.getInstance();
    intake = Intake.getInstance();
    climb = Elavator.getInstance();

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

  public double getMainMotorPositionDegrees(){
    return Units.degreesToRotations(inputs.mainMotorPostion);
  }

  public double getSeconderyMotorPositionDegrees(){
    return Units.degreesToRotations(inputs.secondaryTargetPostion);
  }

  public boolean isArmInPosition(){
    return Math.abs(inputs.mainMotorTargetPostion - inputs.mainMotorPostion) < Constants.ArmConstants.Motors.mainMotortolarance &&
    Math.abs(inputs.secondaryMotorPosition - inputs.secondaryTargetPostion) < Constants.ArmConstants.Motors.seconderyMotorTolarance;
  }

  public double getAngleToSpeaker(){
    return 0;
  }

  public void moveShooterToDegree(double degree){
    MathUtil.clamp(degree, 0, 100);

    inputs.mainMotorTargetPostion = Units.degreesToRotations(degree);
    mainMotor.getPIDController().setReference(inputs.mainMotorTargetPostion, ControlType.kPosition);
  }

  public void moveIntakeToDegree(double degree){
    MathUtil.clamp(degree, 0, 180);

    inputs.secondaryTargetPostion = Units.degreesToRotations(degree);
    secondaryMotor.getPIDController().setReference(inputs.secondaryTargetPostion, ControlType.kPosition);
  }

  public void stallMainMotor(){
    mainMotor.setIdleMode(IdleMode.kBrake);
  }

  public void stallSecondaryMotor(){
    secondaryMotor.setIdleMode(IdleMode.kBrake);
  }

  public void moveShooterToPose(ArmPostion position){
    inputs.mainMotorTargetPostion = Units.radiansToRotations(Math.asin(position.y / ArmConstants.armLengthMeters));
    // mainMotor.getPIDController().setReference(inputs.mainMotorTargetPostion, CANSparkBase.ControlType.kPosition);
    mainMotor.getPIDController().setReference(realToMotorUnits(inputs.mainMotorTargetPostion, Constants.ArmConstants.Motors.mainConvertionFactor), CANSparkBase.ControlType.kPosition);

    inputs.secondaryTargetPostion = inputs.mainMotorTargetPostion + Units.radiansToRotations(position.rotation);
    // secondaryMotor.getPIDController().setReference(inputs.secondaryTargetPostion, CANSparkBase.ControlType.kPosition);
    secondaryMotor.getPIDController().setReference(realToMotorUnits(inputs.secondaryTargetPostion, Constants.ArmConstants.Motors.seconderyConvecrtionFactor), CANSparkBase.ControlType.kPosition);
  }

  public void moveIntakeToPose(ArmPostion postion){
    // switch (controlType) {
    //   case Xaxis:
    //     secondaryMotor.getPIDController().setReference(Units.radiansToRotations(Math.asin((postion.x - shooterPosition.x) / ArmConstants.shooterAndIntakeLengthMeters)) + 
    //     (Math.PI / 2) - shooterPosition.rotation,
    //     CANSparkBase.ControlType.kPosition);
    //     break;
    //   case Yaxis:
    //     secondaryMotor.getPIDController().setReference(Units.radiansToRotations(Math.acos((postion.y - shooterPosition.y) / ArmConstants.shooterAndIntakeLengthMeters)) + 
    //     (Math.PI / 2) - shooterPosition.rotation,
    //     CANSparkBase.ControlType.kPosition);
    //     break;
    //   case Rotation:
    //     secondaryMotor.getPIDController().setReference(Units.radiansToRotations(postion.rotation),
    //     CANSparkBase.ControlType.kPosition);
    //     break;
    //   default:
    //     secondaryMotor.getPIDController().setReference(Units.radiansToRotations(postion.rotation),
    //     CANSparkBase.ControlType.kPosition);
    //     break;
    // }

    inputs.mainMotorTargetPostion = Units.radiansToRotations(
      Math.asin((postion.y + Constants.ArmConstants.shooterAndIntakeLengthMeters * Math.cos(postion.rotation - Math.PI / 2)) / Constants.ArmConstants.armLengthMeters));

    inputs.secondaryTargetPostion = Units.radiansToRotations(postion.rotation) - inputs.mainMotorTargetPostion;

    mainMotor.getPIDController().setReference(realToMotorUnits(inputs.mainMotorTargetPostion, Constants.ArmConstants.Motors.mainConvertionFactor), CANSparkBase.ControlType.kPosition);
    secondaryMotor.getPIDController().setReference(realToMotorUnits(inputs.secondaryTargetPostion, Constants.ArmConstants.Motors.seconderyConvecrtionFactor), CANSparkBase.ControlType.kPosition);
  }

  public void update(){
    updateLogger();
    updateArmPostions();
  }

  @Override
  public void periodic() {
    update();
    SmartDashboard.putNumber("Main absEncoder", mainAbsEncoder.get());
    SmartDashboard.putNumber("secondary absEncoder", secondaryAbsEncoder.get());
    // This method will be called once per scheduler run
  }

  public void updateLogger(){
    inputs.mainMotorPostion = motorToRealUnits(mainMotor.getEncoder().getPosition(), Constants.ArmConstants.Motors.mainConvertionFactor);
    inputs.secondaryMotorPosition = motorToRealUnits(secondaryMotor.getEncoder().getPosition(), Constants.ArmConstants.Motors.seconderyConvecrtionFactor);

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

  private DutyCycleEncoder configureAbsEncoder(int port, double offset){
    DutyCycleEncoder encoder = new DutyCycleEncoder(port);
    encoder.setPositionOffset(offset);
    return encoder;
  }

  private CANSparkFlex configureMotors(int canID, double absPosition, PIDFGains pidfGains, boolean motorInverted, double convertionFactor, double[] softLimit) {
    CANSparkFlex sparkFlex = new CANSparkFlex(canID, MotorType.kBrushless);

    sparkFlex.getPIDController().setP(pidfGains.getP());
    sparkFlex.getPIDController().setI(pidfGains.getI());
    sparkFlex.getPIDController().setD(pidfGains.getD());
    sparkFlex.getPIDController().setIZone(pidfGains.getIZone());

    sparkFlex.setInverted(motorInverted);

    if(softLimit.length != 2) softLimit = new double[]{1, -1};

    sparkFlex.setSoftLimit(SoftLimitDirection.kForward, (float)(softLimit[0] * convertionFactor));
    sparkFlex.setSoftLimit(SoftLimitDirection.kReverse, (float)(softLimit[1] * convertionFactor));

    sparkFlex.getEncoder().setPositionConversionFactor(1);

    sparkFlex.setIdleMode(IdleMode.kBrake);
    
    sparkFlex.getEncoder().setPosition(absPosition * convertionFactor);

    return sparkFlex;
  }

  private static double realToMotorUnits(double realUnits, double convertionFactor){
    return realUnits * convertionFactor;
  }

  private static double motorToRealUnits(double motorUnits, double convertionFactor){
    return motorUnits / convertionFactor;
  }
}
