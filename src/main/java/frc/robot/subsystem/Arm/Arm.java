// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.Arm;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;
import org.opencv.core.Mat;

import com.revrobotics.CANSparkBase;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
// import edu.wpi.first.math.util.Units;
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

    public ArmPostion copyArmPostion(){
      return new ArmPostion(x, y, rotation);
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
    double secondaryMotorTargetPostion;

    double mainMotorAbsolutePostion;
    double secondaryAbsolutePostion;

    double mainCurrent;
    double secondaryCurrent;
  }

  public static Arm getInstance(){
    if(instance == null) instance = new Arm();
    return instance;
  }

  public Elavator getElavatorSub(){
    return elavator;
  }

  public Intake getIntakeSub(){
    return intake;
  }

  public Shooter getShooterSub(){
    return shooter;
  }

  /** Creates a new Arm. */

  private static Arm instance;

  private CANSparkFlex mainMotor;
  private CANSparkFlex secondaryMotor;

  private RelativeEncoder mainEncoder;
  private RelativeEncoder secondaryEncoder;

  private SparkAbsoluteEncoder mainAbsEncoder;
  private SparkAbsoluteEncoder secondaryAbsEncoder;

  private ArmInputsAutoLogged inputs;

  private ArmPostion intakePosition;
  private ArmPostion shooterPosition;

  private Shooter shooter;
  private Intake intake;
  private Elavator elavator;

  private Arm() {
    mainMotor = configureMotors(Constants.ArmConstants.Motors.mainMotorID ,Constants.ArmConstants.Motors.mainPID,
    Constants.ArmConstants.Motors.mainInverted, Constants.ArmConstants.Motors.mainConversionFactor, Constants.ArmConstants.Motors.mainSoftLimits);

    secondaryMotor = configureMotors(Constants.ArmConstants.Motors.secondaryMotorID, Constants.ArmConstants.Motors.secondaryPID,
    Constants.ArmConstants.Motors.secondaryInverted, Constants.ArmConstants.Motors.secondaryConversionFactor, Constants.ArmConstants.Motors.secondarySoftLimits);

    mainAbsEncoder = mainMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);
    secondaryAbsEncoder = secondaryMotor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    mainAbsEncoder.setZeroOffset(Constants.ArmConstants.Motors.mainZeroOffset);
    secondaryAbsEncoder.setZeroOffset(Constants.ArmConstants.Motors.secondaryZeroOffset);

    mainEncoder = mainMotor.getExternalEncoder(8192);
    secondaryEncoder = secondaryMotor.getExternalEncoder(8192);

    mainEncoder.setPosition(mainAbsEncoder.getPosition());
    secondaryEncoder.setPosition(secondaryAbsEncoder.getPosition());

    mainMotor.getPIDController().setFeedbackDevice(mainEncoder);
    secondaryMotor.getPIDController().setFeedbackDevice(secondaryEncoder);

    inputs = new ArmInputsAutoLogged();

    intakePosition = new ArmPostion();
    shooterPosition = new ArmPostion();

    shooter = new Shooter();
    intake = new Intake();
    elavator = new Elavator();

    // new Trigger(RobotState::isEnabled).onTrue(new InstantCommand(() -> {
    //   mainPIDController.setSetpoint(inputs.mainMotorAbsolutePostion);
    //   seconderyPIDController.setSetpoint(inputs.secondaryAbsolutePostion);
    //   SmartDashboard.putNumber("mainT", inputs.mainMotorAbsolutePostion);
    //   SmartDashboard.putNumber("secoT", inputs.secondaryAbsolutePostion);
    // }).ignoringDisable(true));
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

  public double getMainMotorRotation(){
    return inputs.mainMotorPostion;
  }

  public double getSecoMotorRotation(){
    return inputs.secondaryMotorPosition;
  }

  public boolean isArmInPosition(){
    return Math.abs(inputs.mainMotorTargetPostion - inputs.mainMotorPostion) < Constants.ArmConstants.Motors.mainMotorTolerance &&
    Math.abs(inputs.secondaryMotorPosition - inputs.secondaryMotorTargetPostion) < Constants.ArmConstants.Motors.secondaryMotorTolerance;
  }

  public double getAngleToSpeaker(){
    return 0;
  }

  public void moveShooterToPose(ArmPostion position){
    inputs.mainMotorTargetPostion = Math.asin(position.y / ArmConstants.armLengthMeters) / (Math.PI * 2);
    mainMotor.getPIDController().setReference(inputs.mainMotorTargetPostion, ControlType.kSmartMotion);


    inputs.secondaryMotorTargetPostion =  (position.rotation / (Math.PI * 2)) - inputs.mainMotorTargetPostion;
    secondaryMotor.getPIDController().setReference(inputs.secondaryMotorTargetPostion, ControlType.kSmartMotion);
  }

  public void moveIntakeToPose(double alpha, double beta){
    inputs.mainMotorTargetPostion = alpha;
    inputs.secondaryMotorTargetPostion = beta;

    mainMotor.getPIDController().setReference(inputs.mainMotorTargetPostion, ControlType.kSmartMotion);
    secondaryMotor.getPIDController().setReference(inputs.secondaryMotorTargetPostion, ControlType.kSmartMotion);
  }

  public void update(){
    updateLogger();
    updateArmPostions();
  }

  @Override
  public void periodic() {
    update();
    SmartDashboard.putNumber("Main absEncoder", inputs.mainMotorAbsolutePostion);
    SmartDashboard.putNumber("secondary absEncoder", inputs.secondaryAbsolutePostion);

    SmartDashboard.putNumber("shooter angle", shooterPosition.rotation / (Math.PI) * 180);

    SmartDashboard.putNumber("prox", intake.getProximity());

    // SmartDashboard.getData("main");
    // SmartDashboard.getData("seco");

    // mainPIDController.setSetpoint(SmartDashboard.getNumber("mainT", 0.25));
    // seconderyPIDController.setSetpoint(SmartDashboard.getNumber("secoT", 0.1));
    // mainPIDController.setSetpoint(MathUtil.clamp(SmartDashboard.getNumber("mainT", 0.25), Constants.ArmConstants.Motors.mainSoftLimits[1], Constants.ArmConstants.Motors.mainSoftLimits[0]));
    // seconderyPIDController.setSetpoint(MathUtil.clamp(SmartDashboard.getNumber("secoT", 0.1), Constants.ArmConstants.Motors.secondarySoftLimits[1], Constants.ArmConstants.Motors.secondarySoftLimits[0]));


    // mainMotor.set(mainPIDController.calculate(inputs.mainMotorAbsolutePostion));
    // secondaryMotor.set(seconderyPIDController.calculate(inputs.secondaryAbsolutePostion));
    // mainMotor.getPIDController().setReference(mainPIDController.calculate(inputs.mainMotorAbsolutePostion), ControlType.kVoltage);
    // secondaryMotor.getPIDController().setReference(seconderyPIDController.calculate(inputs.secondaryAbsolutePostion), ControlType.kVoltage);



    // This method will be called once per scheduler run
  }

  public void updateLogger(){
    inputs.mainMotorPostion = mainEncoder.getPosition();
    inputs.secondaryMotorPosition = secondaryEncoder.getPosition();

    inputs.mainCurrent = mainMotor.getOutputCurrent();
    inputs.secondaryCurrent = secondaryMotor.getOutputCurrent();

    inputs.mainMotorAbsolutePostion = mainAbsEncoder.getPosition();
    inputs.secondaryAbsolutePostion = secondaryEncoder.getPosition();

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

  // private CANSparkFlex configureMotors(int canID, double absPosition, PIDFGains pidfGains, boolean motorInverted, double convertionFactor, double[] softLimit, double VoltageCompensation) {
    private CANSparkFlex configureMotors(int canID, PIDFGains pidfGains, boolean motorInverted, double convertionFactor, double[] softLimit) {
    CANSparkFlex sparkFlex = new CANSparkFlex(canID, MotorType.kBrushless);

    sparkFlex.restoreFactoryDefaults();
    sparkFlex.getPIDController().setP(pidfGains.getP());
    sparkFlex.getPIDController().setI(pidfGains.getI());
    sparkFlex.getPIDController().setD(pidfGains.getD());
    sparkFlex.getPIDController().setIZone(pidfGains.getIZone());

    sparkFlex.setInverted(motorInverted);

    if(softLimit.length != 2) softLimit = new double[]{1, -1};

    sparkFlex.setSoftLimit(SoftLimitDirection.kForward, (float)(softLimit[0]));
    sparkFlex.setSoftLimit(SoftLimitDirection.kReverse, (float)(softLimit[1]));

    sparkFlex.getEncoder().setPositionConversionFactor(1);

    sparkFlex.setIdleMode(IdleMode.kBrake);
    
    sparkFlex.enableVoltageCompensation(12);
    //Todo: What is Voltage Compensation? 

    return sparkFlex;
  }

  // public class SysIDArm{
  //   MutableMeasure<Velocity<Angle>> mainAngularVelocity = MutableMeasure.mutable(edu.wpi.first.units.Units.RotationsPerSecond.zero());
  //   MutableMeasure<Angle> mainAngularPosition = MutableMeasure.mutable(edu.wpi.first.units.Units.Rotations.zero());
  //   MutableMeasure<Voltage> mainVoltage = MutableMeasure.mutable(edu.wpi.first.units.Units.Volts.zero());

  //   public SysIDArm(){}
    
  //   SysIdRoutine mainMotorRoutine = new SysIdRoutine(
  //     new SysIdRoutine.Config(),
  //     new SysIdRoutine.Mechanism(
  //       (volts) -> {
  //         mainMotor.getPIDController().setReference(volts.in(edu.wpi.first.units.Units.Volts), ControlType.kVoltage);
  //       }, 
  //       (log) -> {
  //         // How to get Acceleration?
  //         log.motor("Main Motor")
  //         .angularVelocity(mainAngularVelocity.mut_replace(mainMotor.getEncoder().getVelocity() / Constants.ArmConstants.Motors.mainConversionFactor, edu.wpi.first.units.Units.RotationsPerSecond))
  //         .angularPosition(mainAngularPosition.mut_replace(mainMotor.getEncoder().getPosition() / Constants.ArmConstants.Motors.mainConversionFactor, edu.wpi.first.units.Units.Rotations))
  //         .voltage(mainVoltage.mut_replace(mainMotor.getAppliedOutput() * mainMotor.getBusVoltage(), edu.wpi.first.units.Units.Volts));
  //       }, 
  //       Arm.getInstance()));
    
  //   MutableMeasure<Velocity<Angle>> secondaryAngularVelocity = MutableMeasure.mutable(edu.wpi.first.units.Units.RotationsPerSecond.zero());
  //   MutableMeasure<Angle> secondaryAngularPosition = MutableMeasure.mutable(edu.wpi.first.units.Units.Rotations.zero());
  //   MutableMeasure<Voltage> secondaryVoltage = MutableMeasure.mutable(edu.wpi.first.units.Units.Volts.zero());

  //   SysIdRoutine secondaryMotorRoutine = new SysIdRoutine(
  //     new SysIdRoutine.Config(
  //       // null, null, null,
  //       // (state) -> Logger.recordOutput("SysIdTestState", state.toString())
  //     ),
  //     new SysIdRoutine.Mechanism(
  //       (volts) -> {
  //         secondaryMotor.getPIDController().setReference(volts.in(edu.wpi.first.units.Units.Volts), ControlType.kVoltage);
  //       }, 
  //       (log) -> {
  //         // How to get Acceleration?
  //         log.motor("Seco Motor")
  //         .angularVelocity(secondaryAngularVelocity.mut_replace(secondaryMotor.getEncoder().getVelocity() / Constants.ArmConstants.Motors.secondaryConversionFactor, edu.wpi.first.units.Units.RPM))
  //         .angularPosition(secondaryAngularPosition.mut_replace(secondaryMotor.getEncoder().getPosition() / Constants.ArmConstants.Motors.secondaryConversionFactor, edu.wpi.first.units.Units.Rotations))
  //         .voltage(secondaryVoltage.mut_replace(secondaryMotor.getAppliedOutput() * secondaryMotor.getBusVoltage(), edu.wpi.first.units.Units.Volts));
  //       }, 
  //       Arm.getInstance()));

  //   public Command quasistaticMain(SysIdRoutine.Direction direction){
  //     return mainMotorRoutine.quasistatic(direction);
  //   }
  //   public Command quasistaticSecondary(SysIdRoutine.Direction direction){
  //     return secondaryMotorRoutine.quasistatic(direction);
  //   }

  //   public Command dynamicMain(SysIdRoutine.Direction direction){
  //     return mainMotorRoutine.dynamic(direction);
  //   }
  //   public Command dynamicSecondary(SysIdRoutine.Direction direction){
  //     return secondaryMotorRoutine.dynamic(direction);
  //   }

  // }
}
