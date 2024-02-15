// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.Arm;

import frc.robot.subsystem.Arm.Arm.ArmPosition;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class ArmUtil{
    private static ArmPosition ZaxisTarget;
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
        ArmAngle = Math.atan((Constants.SpeakerTranslation.getZ() - ZaxisTarget.y - Constants.Arm.RobotHightFromGround)//hieght
        /distanceToSpeaker);//if there is a problem, return last angle
      } catch (Exception e) {
      }
    }
    private static void Zone2_Equasion(){//complicated for zone 2
      try {
        ArmAngle = Math.atan((Math.pow(StartSpeed, 2)
       - Math.sqrt(Math.pow(StartSpeed, 4)
        - 2*(Constants.SpeakerTranslation.getZ() - ZaxisTarget.y - Constants.Arm.RobotHightFromGround)//hieght
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
      if (distanceToSpeaker <= Constants.Arm.EndOfZone1) {
        if(IsQuikShot){
          ZaxisTarget = Constants.Arm.QuikShotPosition;
        }
        else ZaxisTarget = Constants.Arm.Zone1_ArmPosition;
        Zone1_Equasion();
      }
      else{
        if(IsQuikShot){
          ZaxisTarget = Constants.Arm.QuikShotPosition;
        }
        else ZaxisTarget = Constants.Arm.Zone2_ArmPosition;
        Zone2_Equasion();
      }
      RobotSpeedRelative_angle();
    }
  
    private static void getDy(){
      if (RobotContainer.driveBase.getPose().getTranslation().getY() <= Constants.Arm.SpeakerIsCenterRatioBottomLocation) {
        IsDeadZone = true;
            Dy = Constants.Arm.SpeakerBottomLocationY + Constants.Arm.SpeakerLength;//aime to the most right corner (robot prespective)
      }
      else{
        IsDeadZone = false;
            Dy = Constants.Arm.SpeakerBottomLocationY
       + Constants.Arm.SpeakerLength - Constants.Arm.SpeakerIsCenterRatio * (RobotContainer.driveBase.getPose().getTranslation().getY()
        - Constants.Arm.SpeakerIsCenterRatioBottomLocation);//aim to a point prespective to the robot location in the chosen shooting zone
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
    StartSpeed = RobotContainer.arm.getShooterSub().getShooterVelocity();
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
    public static ArmPosition getArmNeededPosition(){
      return ZaxisTarget;
    }
  }
