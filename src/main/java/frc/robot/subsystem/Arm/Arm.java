// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.Arm;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import com.revrobotics.CANSparkFlex;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkBase.SoftLimitDirection;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystem.Arm.Intake.Intake;
import frc.robot.subsystem.Arm.Shooter.Shooter;
import frc.robot.subsystem.Arm.climb.Elavator;
import frc.util.PIDFGains;

public class Arm extends SubsystemBase{

  public static enum knownArmPosition{
    Intake, Shooter, Home, Stow, Free ,Amp, Climb, Unknown, AlphaAim_close
  }
  
  public static class ArmPosition{
    public double x;
    public double y;
    public double rotation;

    public ArmPosition(double x, double y, double rotation){
      this.x = x;
      this.y = y;
      this.rotation = rotation;
    }

   public ArmPosition(){
      this.x = 0;
      this.y = 0;
      this.rotation = 0;
    }

    public ArmPosition(Pose2d pose){
      this.x = pose.getX();
      this.y = pose.getY();
      this.rotation = pose.getRotation().getRadians();
    }

    public ArmPosition copyArmPostion(){
      return new ArmPosition(x, y, rotation);
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

    // Mechanism2d visualArm;
    // MechanismRoot2d visualArm_Root;
    // MechanismLigament2d visualArm_MainPivot;
    // MechanismLigament2d visualArm_SeconderyPivot;
    // MechanismLigament2d visualArm_Elavator;
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

  private ArmPosition intakePosition;
  private ArmPosition shooterPosition;

  public knownArmPosition lastknownPosition;

  private Shooter shooter;
  private Intake intake;
  private Elavator elavator;

  private Arm() {
    mainMotor = configureMotors(Constants.Arm.Motors.mainMotorID ,Constants.Arm.Motors.mainPID,
    Constants.Arm.Motors.mainInverted, Constants.Arm.Motors.mainSoftLimits, Constants.Arm.Motors.mainMaxOutputs);

    secondaryMotor = configureMotors(Constants.Arm.Motors.secondaryMotorID, Constants.Arm.Motors.secondaryPID,
    Constants.Arm.Motors.secondaryInverted, Constants.Arm.Motors.secondarySoftLimits, Constants.Arm.Motors.secondaryMaxOutputs);

    mainAbsEncoder = configAbsoluteEncoder(mainMotor, true, Constants.Arm.Motors.mainZeroOffset);
    secondaryAbsEncoder = configAbsoluteEncoder(secondaryMotor, true, Constants.Arm.Motors.secondaryZeroOffset);

    mainEncoder = configEncoder(mainMotor, 8192, Constants.Arm.Motors.mainEncoderInverted, getRollOverPosition(mainAbsEncoder.getPosition()));
    secondaryEncoder = configEncoder(secondaryMotor, 8192, Constants.Arm.Motors.secondaryEncoderInverted, getRollOverPosition(secondaryAbsEncoder.getPosition()));

    inputs = new ArmInputsAutoLogged();

    intakePosition = new ArmPosition();
    shooterPosition = new ArmPosition();

    shooter = new Shooter();
    intake = new Intake();  
    elavator = new Elavator();

    lastknownPosition = knownArmPosition.Home;

    // inputs.visualArm = new Mechanism2d(getMainMotorRotation(), getAngleToSpeaker()); //TODO add length\
    // inputs.visualArm_Root = inputs.visualArm.getRoot("arm root", -Constants.Arm.mainPivotDistanceFromCenterMeters, 0);
    // inputs.visualArm_MainPivot = new MechanismLigament2d("main pivot", Constants.Arm.armLengthMeters, Units.rotationsToDegrees(mainEncoder.getPosition()));
    // inputs.visualArm_SeconderyPivot = new MechanismLigament2d("secondery pivot", 0.389, Units.rotationsToDegrees(secondaryEncoder.getPosition()));
    // inputs.visualArm_Elavator = new MechanismLigament2d("elavator", 0.5, 0);

    // inputs.visualArm_Root.append(inputs.visualArm_MainPivot);
    // inputs.visualArm_MainPivot.append(inputs.visualArm_SeconderyPivot);
    // inputs.visualArm_MainPivot.append(inputs.visualArm_Elavator);
  }

  /**
   * creats the triggers for the arm (for all the zones and automatic functions)
   */
  private void createArmTriggers(){
    new Trigger(() -> RobotContainer.getRobotZone() >= 1 && RobotContainer.getRobotZone() <= 3).and(RobotContainer::isAmplified)
    .and(this::isArmInPosition).and(RobotContainer::isRobotSpeakerMode).onTrue(new InstantCommand()); //TODO: add command that shoots, and add robot rotation check

    // new Trigger(() -> RobotContainer.getRobotZone() == 1).and(RobotContainer::isRobotSpeakerMode).whileTrue(new RepeatCommand(new AimShooterZone1()));
    // new Trigger(() -> RobotContainer.getRobotZone() == 2).and(RobotContainer::isRobotSpeakerMode).whileTrue(new InstantCommand()); //TODO: add command that homes the arm with more advance calculations
    //THE SHOOTING ZONES ARE IN ARMUTIL
    new Trigger(() -> RobotContainer.getRobotZone() == 2 || RobotContainer.getRobotZone() == 3).and(RobotContainer::isRobotAmpMode);
  
  }

  /**
   * gets the current position of the shooter
   * @return the current position of the shooter
   */
  public ArmPosition getShooterPosition(){
    return shooterPosition;
  }

  /**
   * gets the current position of the intake
   * @return the current position of the intake
   */
  public ArmPosition getIntakePosition(){
    return intakePosition;
  }

  /**
   * gets the current position of the main motor
   * @return the current position of the main motor (in rotations)
   */
  public double getMainMotorRotation(){
    return inputs.mainMotorPostion;
  }

  /**
   * gets the current position of the secondary motor
   * @return the current position of the secondary motor (in rotations)
   */
  public double getSecondaryMotorRotation(){
    return inputs.secondaryMotorPosition;
  }

  /**
   * returns if the arm is within tolerance of the target position
   * @return if the arm is within tolerance of the target position
   */
  public boolean isArmInPosition(){
    return Math.abs(inputs.mainMotorTargetPostion - inputs.mainMotorPostion) < Constants.Arm.Motors.mainPID.getTolerance() &&
    Math.abs(inputs.secondaryMotorPosition - inputs.secondaryMotorTargetPostion) < Constants.Arm.Motors.secondaryPID.getTolerance();
  }

  public double getAngleToSpeaker(){
    // return SmartDashboard.getNumber("angle To Speaker", 90);
    return Units.radiansToDegrees(Math.atan((Constants.Speaker.SpeakerTranslation.getZ() - (shooterPosition.y + Constants.Arm.armHeightFromFrameMeters + Constants.DriveTrain.Global.RobotHeightFromGround))
    / SmartDashboard.getNumber("distance To Speaker", 0.1)));
  }

  /**
   * moves the shooter to the given pose
   * @param position the target pose of the shooter (using the y for the main motor and the rotation for the secondary motor)
   */
  public void moveShooterToPose(ArmPosition position){
    inputs.mainMotorTargetPostion = Math.asin(position.y / Constants.Arm.armLengthMeters) / (Math.PI * 2);
    inputs.mainMotorTargetPostion = MathUtil.clamp(inputs.mainMotorTargetPostion, Constants.Arm.Motors.mainSoftLimits[1], Constants.Arm.Motors.mainSoftLimits[0]);

    mainMotor.getPIDController().setReference(inputs.mainMotorTargetPostion, ControlType.kPosition);


    inputs.secondaryMotorTargetPostion =  Units.radiansToRotations(position.rotation) - inputs.mainMotorTargetPostion;
    inputs.secondaryMotorTargetPostion = MathUtil.clamp(inputs.secondaryMotorTargetPostion, Constants.Arm.Motors.secondarySoftLimits[1], Constants.Arm.Motors.secondarySoftLimits[0]);

    secondaryMotor.getPIDController().setReference(inputs.secondaryMotorTargetPostion, ControlType.kPosition);
  }

  /**
   * moves the motors to the given rotation 
   * @param alpha the target of the main motor (IN ROTATIONS)
   * @param beta the target of the secondary motor (IN ROTATIONS)
   */
  public void moveMotorsToRotation(double alpha, double beta){
    inputs.mainMotorTargetPostion = alpha;
    inputs.secondaryMotorTargetPostion = beta;


    mainMotor.getPIDController().setReference(inputs.mainMotorTargetPostion, ControlType.kPosition);
    secondaryMotor.getPIDController().setReference(inputs.secondaryMotorTargetPostion, ControlType.kPosition);
  }

  @Override
  public void periodic() {
    updateLogger();
    updateArmPostions();
    shooter.update();
    intake.update();
    elavator.update();
  }

  /**
   * recored the inputs of the arm and sends them to the logger
   */
  public void updateLogger(){
    inputs.mainMotorPostion = mainEncoder.getPosition();
    inputs.secondaryMotorPosition = secondaryEncoder.getPosition();

    inputs.mainCurrent = mainMotor.getOutputCurrent();
    inputs.secondaryCurrent = secondaryMotor.getOutputCurrent();

    inputs.mainMotorAbsolutePostion = mainAbsEncoder.getPosition();
    inputs.secondaryAbsolutePostion = secondaryAbsEncoder.getPosition();

    // inputs.visualArm_MainPivot.setAngle(Units.rotationsToDegrees(mainEncoder.getPosition()));
    // inputs.visualArm_SeconderyPivot.setAngle(Units.rotationsToDegrees(secondaryEncoder.getPosition()));
    // inputs.visualArm_Elavator.setLength(elavator.getRailMotorPosition());

    Logger.processInputs(getName(), inputs);
  }

  /**
   * takes the recoreded motor positions and calculates the arm positions
   */
  public void updateArmPostions(){
    shooterPosition.x = (Math.cos(Units.rotationsToRadians(inputs.mainMotorPostion))) * (Constants.Arm.armLengthMeters) - Constants.Arm.mainPivotDistanceFromCenterMeters
    + (Math.sin(Units.rotationsToRadians(inputs.secondaryMotorPosition)) * Constants.Arm.SecondaryMotorDistanceFromShooterMeters);
    shooterPosition.y = Math.sin(Units.rotationsToRadians(inputs.mainMotorPostion)) * (Constants.Arm.armLengthMeters) + Constants.Arm.armHeightFromFrameMeters + Constants.DriveTrain.Global.RobotHeightFromGround
    - (Math.cos(Units.rotationsToRadians(inputs.secondaryMotorPosition))) * Constants.Arm.SecondaryMotorDistanceFromShooterMeters;
    shooterPosition.rotation = Units.rotationsToRadians(inputs.mainMotorPostion + inputs.secondaryMotorPosition);

    intakePosition.x = shooterPosition.x +
    Math.sin(Units.rotationsToRadians(inputs.secondaryMotorPosition) - (Math.PI / 2) - Units.rotationsToRadians(inputs.mainMotorPostion)) * Constants.Arm.shooterAndIntakeLengthMeters;
    intakePosition.y = shooterPosition.y + 
    Math.cos(Units.rotationsToRadians(inputs.secondaryMotorPosition) - (Math.PI / 2) - Units.rotationsToRadians(inputs.mainMotorPostion)) * Constants.Arm.shooterAndIntakeLengthMeters;
    intakePosition.rotation = Units.rotationsToRadians(inputs.secondaryMotorPosition);
  }

  /**
   * returns the roll over position of the given value (used for abslute encoders that have a roll over position at 0 and 1)
   * @param value the value of the abs encoder
   * @return the roll over position of the given value (MAY BE NEGETIVE)
   */
  private double getRollOverPosition(double value){
    if(value < 0.75) return value;
    return value - 1;
  }

  public static class IsLastPosition extends Command{
    private knownArmPosition position;

    public IsLastPosition(knownArmPosition position){
      this.position = position;
    }

    @Override
    public void initialize() {
      if(position == Arm.getInstance().lastknownPosition) cancel();
    }
  }

  /**
   * creates a new spark flex motor and configures it with the given parameters
   * @param canID the Can ID of the motor
   * @param pidfGains the PIDF gains of the motor
   * @param motorInverted if the motor is inverted
   * @param softLimit the soft limits of the motor
   * @param maxOutputs the max outputs of the motor
   * @return
   */
  private CANSparkFlex configureMotors(int canID, PIDFGains pidfGains, boolean motorInverted , double[] softLimit, double[] maxOutputs) {
    CANSparkFlex sparkFlex = new CANSparkFlex(canID, MotorType.kBrushless);

    sparkFlex.restoreFactoryDefaults();

    Timer.delay(0.1);

    sparkFlex.getPIDController().setP(pidfGains.getP());
    sparkFlex.getPIDController().setI(pidfGains.getI());
    sparkFlex.getPIDController().setD(pidfGains.getD());
    sparkFlex.getPIDController().setIZone(pidfGains.getIZone());

    if(maxOutputs.length != 2) maxOutputs = new double[]{0.25, -0.25};

    sparkFlex.getPIDController().setOutputRange(maxOutputs[1], maxOutputs[0]);

    sparkFlex.setInverted(motorInverted);

    if(softLimit.length != 2) softLimit = new double[]{1, -1};

    sparkFlex.setSoftLimit(SoftLimitDirection.kForward, (float)(softLimit[0]));
    sparkFlex.setSoftLimit(SoftLimitDirection.kReverse, (float)(softLimit[1]));

    sparkFlex.enableSoftLimit(SoftLimitDirection.kForward, true);
    sparkFlex.enableSoftLimit(SoftLimitDirection.kReverse, true);

    sparkFlex.getEncoder().setPositionConversionFactor(1);

    sparkFlex.setIdleMode(IdleMode.kBrake);
    
    sparkFlex.enableVoltageCompensation(12);
    //Todo: What is Voltage Compensation? 

    return sparkFlex;
  }

  /**
   * configures the absolute encoder of the given motor with the given parameters
   * @param motor the motor that the abs encoder is connected to
   * @param inverted if the encoder is inverted
   * @param ZeroOffset the zero offset of the encoder
   * @return
   */
  private SparkAbsoluteEncoder configAbsoluteEncoder(CANSparkFlex motor, boolean inverted, double ZeroOffset){
    SparkAbsoluteEncoder encoder = motor.getAbsoluteEncoder(SparkAbsoluteEncoder.Type.kDutyCycle);

    encoder.setInverted(inverted);

    Timer.delay(0.2);

    encoder.setZeroOffset(ZeroOffset);

    Timer.delay(0.2);

    return encoder;
  }

  /**
   * configures the relative encoder of the given motor with the given parameters
   * @param motor the motor that the relative encoder is connected to
   * @param ccountsPerRev the counts per revolution of the encoder
   * @param inverted if the encoder is inverted
   * @param absPosition the absolute position of the abs encoder
   * @return
   */
  private RelativeEncoder configEncoder(CANSparkFlex motor, int ccountsPerRev, boolean inverted, double absPosition){
    RelativeEncoder encoder = motor.getExternalEncoder(ccountsPerRev);

    encoder.setInverted(inverted);

    Timer.delay(0.2);

    encoder.setPosition(absPosition);

    Timer.delay(0.2);

    motor.getPIDController().setFeedbackDevice(encoder);

    return encoder;
  }
}
