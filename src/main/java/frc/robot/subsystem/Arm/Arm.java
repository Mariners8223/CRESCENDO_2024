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
    SmartDashboard.putNumber("sliding motor", Elavator.getInstance().getRollerPosition());
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
  public static class ArmAimUtil{
    private static ArmPostion ZaxisTarget;
    private static double StartSpeed;
    private static double Dy;//y axis of targeted point (to calc the distance from speaker)
    private static double Dx;//airial distance to speaker
    private static double distanceToSpeaker;
    private static double ArmAngle = 45;
    public static boolean IsDeadZone;
    public static boolean IsQuikShot;
    //chassis parameters
    private static double YaxisWantedAngle;
    private static double YaxisOffset;
    private static double ChasisAngle;
    private static double VelocityY;
    private static double VelocityX;
  
    private static void Zone1_Equasion(){//simple lazer for zone 1
      try {
        ArmAngle = Math.atan((Constants.SpeakerTranslation.getZ() - ZaxisTarget.y - Constants.ArmConstants.RobotHightFromGround)//hieght
        /distanceToSpeaker);//if there is a problem, return last angle
      } catch (Exception e) {
      }
    }
    private static void Zone2_Equasion(){//complicated for zone 2
      try {
        ArmAngle = Math.atan((Math.pow(StartSpeed, 2)
       - Math.sqrt(Math.pow(StartSpeed, 4)
        - 2*(Constants.SpeakerTranslation.getZ() - ZaxisTarget.y - Constants.ArmConstants.RobotHightFromGround)//hieght
        *Constants.gGravity_phisics*Math.pow(StartSpeed, 2) - Math.pow(Constants.gGravity_phisics*Dx, 2)))
        /(Dx*Constants.gGravity_phisics)
      );
      } catch (Exception e) {
      }
    }
  
    private static void RobotSpeedRelative_angle(){
      try {
        ArmAngle = Math.atan((StartSpeed*Math.sin(ArmAngle))/(VelocityX*Math.cos(YaxisWantedAngle)*(-1)
         + VelocityY*Math.cos(YaxisWantedAngle - Units.degreesToRadians(90))));
      } catch (Exception e) {
      }
    }
  
    private static void CalcAngleZaxis(){
      if (distanceToSpeaker <= Constants.ArmConstants.EndOfZone1) {
        if(IsQuikShot){
          ZaxisTarget = Constants.ArmConstants.QuikShotPosition;
        }
        else ZaxisTarget = Constants.ArmConstants.Zone1_ArmPosition;
        Zone1_Equasion();
      }
      else{
        if(IsQuikShot){
          ZaxisTarget = Constants.ArmConstants.QuikShotPosition;
        }
        else ZaxisTarget = Constants.ArmConstants.Zone2_ArmPosition;
        Zone2_Equasion();
      }
      RobotSpeedRelative_angle();
    }
  
    private static void getDy(){
      if (RobotContainer.driveBase.getPose().getTranslation().getY() <= Constants.ArmConstants.SpeakerIsCenterRatioBottomLocation) {
        IsDeadZone = true;
            Dy = Constants.ArmConstants.SpeakerBottomLocationY + Constants.ArmConstants.SpeakerLength;//aime to the most right corner (robot prespective)
      }
      else{
        IsDeadZone = false;
            Dy = Constants.ArmConstants.SpeakerBottomLocationY
       + Constants.ArmConstants.SpeakerLength - Constants.ArmConstants.SpeakerIsCenterRatio * (RobotContainer.driveBase.getPose().getTranslation().getY()
        - Constants.ArmConstants.SpeakerIsCenterRatioBottomLocation);//aim to a point prespective to the robot location in the chosen shooting zone
      }
    }
    private static void getDx(){
      Dx = RobotContainer.driveBase.getPose().getTranslation().getX() - Constants.SpeakerTranslation.getX();
    }
    private static void CalcDistance_withDxDy(){
      getDx();
      getDy();
      distanceToSpeaker = Math.hypot(Dx, Dy);
    }
  
    private static void getWantedDegree(){
      YaxisWantedAngle = Units.degreesToRadians(180) - Math.atan(Dy/Dx);
    }
    private static void getChassisOffset(){
      YaxisOffset = Math.atan(VelocityY/VelocityX);
    }
    private static void CalcYaxisAngle(){
      getChassisOffset();
      ChasisAngle =  YaxisWantedAngle*2 - YaxisOffset;
    }
    private static void CalcVelocityXy_field(){
      VelocityY = StartSpeed*Math.cos(ArmAngle)*Math.sin(YaxisWantedAngle)
      + RobotContainer.driveBase.getAbsoluteChassisSpeeds().vyMetersPerSecond + RobotContainer.driveBase.getAbsoluteChassisSpeeds().omegaRadiansPerSecond
      * RobotContainer.arm.getShooterPosition().x * Math.sin(Units.degreesToRadians(-90) + YaxisWantedAngle);
      VelocityX = StartSpeed*Math.cos(ArmAngle)*(-1)*Math.cos(YaxisWantedAngle)
      + RobotContainer.driveBase.getAbsoluteChassisSpeeds().vxMetersPerSecond + RobotContainer.driveBase.getAbsoluteChassisSpeeds().omegaRadiansPerSecond
      * RobotContainer.arm.getShooterPosition().x * Math.cos(Units.degreesToRadians(-90) + YaxisWantedAngle);
    }

    public static void UpdateParameters(){
    StartSpeed = RobotContainer.arm.getShooter().getShooterVelocity();
    CalcDistance_withDxDy();
    getWantedDegree();
    CalcAngleZaxis();
    CalcVelocityXy_field();
    ZaxisTarget.rotation = ArmAngle;
    CalcYaxisAngle();
    }
    public static double getArmAngle(){
      return ArmAngle;
    }
    public static double getChassisAngle(){
      return ChasisAngle;
    }
    public static ArmPostion getArmNeededPosition(){
      return ZaxisTarget;
    }
  }
}
