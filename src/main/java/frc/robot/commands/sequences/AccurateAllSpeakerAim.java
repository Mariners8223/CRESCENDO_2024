// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystem.Arm.Arm.ArmPostion;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AccurateAllSpeakerAim extends SequentialCommandGroup {
  //shooter parameters
  private static ArmPostion ZaxisTarget;
  private static double StartSpeed;
  private static double Dy;//y axis of targeted point (to calc the distance from speaker)
  private static double Dx;//airial distance to speaker
  private static double distanceToSpeaker;
  private static double angleZaxis = 45;
  public static boolean IsDeadZone;
  //chassis parameters
  private static double YaxisWantedAngle;
  private static double YaxisOffset;
  private static double VelocityY;
  private static double VelocityX;

  public void Zone1_Equasion(){//simple lazer for zone 1
    try {
      angleZaxis = Math.atan((Constants.SpeakerTranslation.getZ() - ZaxisTarget.y - Constants.ArmConstants.RobotHightFromGround)//hieght
      /distanceToSpeaker);//if there is a problem, return last angle
    } catch (Exception e) {
    }
  }
  public void Zone2_Equasion(){//complicated for zone 2
    try {
      angleZaxis = Math.atan((Math.pow(StartSpeed, 2)
     - Math.sqrt(Math.pow(StartSpeed, 4)
      - 2*(Constants.SpeakerTranslation.getZ() - ZaxisTarget.y - Constants.ArmConstants.RobotHightFromGround)//hieght
      *Constants.gGravity_phisics*Math.pow(StartSpeed, 2) - Math.pow(Constants.gGravity_phisics*Dx, 2)))
      /(Dx*Constants.gGravity_phisics)
    );
    } catch (Exception e) {
    }
  }

  public void RobotSpeedRelative_angle(){
    try {
      angleZaxis = Math.atan((StartSpeed*Math.sin(angleZaxis))/(VelocityX*Math.cos(YaxisWantedAngle)*(-1)
       + VelocityY*Math.cos(YaxisWantedAngle - Units.degreesToRadians(90))));
    } catch (Exception e) {
    }
  }

  public void CalcAngleZaxis(){
    if (distanceToSpeaker <= Constants.ArmConstants.EndOfZone1) {
      ZaxisTarget = Constants.ArmConstants.Zone1_ArmPosition;
      Zone1_Equasion();
    }
    else{
      ZaxisTarget = Constants.ArmConstants.Zone2_ArmPosition;
      Zone2_Equasion();
    }
    RobotSpeedRelative_angle();
  }

  public void getDy(){
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
  public void getDx(){
    Dx = RobotContainer.driveBase.getPose().getTranslation().getX() - Constants.SpeakerTranslation.getX();
  }
  public void CalcDistance_withDxDy(){
    getDx();
    getDy();
    distanceToSpeaker = Math.hypot(Dx, Dy);
  }

  public void getWantedDegree(){
    YaxisWantedAngle = Units.degreesToRadians(180) - Math.atan(Dy/Dx);
  }
  public void getChassisOffset(){
    YaxisOffset = Math.atan(VelocityY/VelocityX);
  }
  public double CalcYaxisAngle(){
    getChassisOffset();
    return YaxisWantedAngle*2 - YaxisOffset;
  }
  private void CalcVelocityXy_field(){
    VelocityY = StartSpeed*Math.cos(angleZaxis)*Math.sin(YaxisWantedAngle)
    + RobotContainer.driveBase.getAbsoluteChassisSpeeds().vyMetersPerSecond + RobotContainer.driveBase.getAbsoluteChassisSpeeds().omegaRadiansPerSecond
    * RobotContainer.arm.getShooterPosition().x * Math.sin(Units.degreesToRadians(-90) + YaxisWantedAngle);
    VelocityX = StartSpeed*Math.cos(angleZaxis)*(-1)*Math.cos(YaxisWantedAngle)
    + RobotContainer.driveBase.getAbsoluteChassisSpeeds().vxMetersPerSecond + RobotContainer.driveBase.getAbsoluteChassisSpeeds().omegaRadiansPerSecond
    * RobotContainer.arm.getShooterPosition().x * Math.cos(Units.degreesToRadians(-90) + YaxisWantedAngle);
  }
  /** Creates a new AccurateAllSpeakerAim. */
  public AccurateAllSpeakerAim() {
    StartSpeed = RobotContainer.arm.getShooter().getShooterVelocity();
    CalcDistance_withDxDy();
    getWantedDegree();
    CalcAngleZaxis();
    CalcVelocityXy_field();
    ZaxisTarget.rotation = angleZaxis;
  
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new InstantCommand(() -> RobotContainer.arm.getShooter().setShooterPower(0.8)),
    new ParallelCommandGroup(new InstantCommand(() -> RobotContainer.arm.moveShooterToPose(ZaxisTarget)),
    new InstantCommand(() -> RobotContainer.driveBase.setTarGetRotation(Rotation2d.fromDegrees(CalcYaxisAngle()))))
    );
  }
}
