// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.Arm;

import frc.robot.subsystem.Arm.Arm.ArmPosition;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class ArmUtil{
    private static ArmPosition ZaxisTarget;//arm angle arm position
    private static double StartSpeed;//the speed in which the gp is leaving the shooter
    private static double Dy;//y axis of targeted point (to calc the distance from speaker)
    private static double Dx;//airial distance to speaker
    private static double distanceToSpeaker;//self explanatory
    private static double ArmAngle = 45;//the angle in which the arm shell be
    public static boolean IsDeadZone;//is this a dead zone???
    public static boolean IsQuikShot = true;//are we using quik shot????
    //chassis parameters
    private static double YaxisWantedAngle;//the angle in which we want the gp to fly in
    private static double YaxisOffset;//oh no! speed affects our vector, lets calc the offset!
    private static double ChasisAngle;//the final angle that the robot will face
    private static double VelocityY;//field relative
    private static double VelocityX;//field relative
  
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
  
    private static void RobotSpeedRelative_angle(){//calcs the arm angle with speed relativity
      try {
        ArmAngle = Math.atan(-(StartSpeed*Math.sin(ArmAngle))/(StartSpeed * Math.cos(ArmAngle)
        + RobotContainer.driveBase.getAbsoluteChassisSpeeds().vxMetersPerSecond * Math.cos(YaxisWantedAngle)
        + RobotContainer.driveBase.getAbsoluteChassisSpeeds().vyMetersPerSecond * Math.sin(YaxisWantedAngle)));
      } catch (Exception e) {
      }
    }
  
    private static void CalcAngleZaxis(){//chooses zone 1 or 2 or quik shot modes
      // System.out.println("shit");
      if (distanceToSpeaker <= Constants.Arm.EndOfZone1) {
        // System.out.println("1");
        Zone1_Equasion();
        RobotSpeedRelative_angle();
        if(IsQuikShot){
          ZaxisTarget = Constants.Arm.QuikShotPosition;
          ArmAngle = Units.degreesToRadians(180) - ArmAngle;
        }
        else{
          ZaxisTarget = Constants.Arm.Zone1_ArmPosition;
          ZaxisTarget.y = MathUtil.clamp(Constants.Arm.armLengthMeters * Math.sin(ArmAngle), 0, Constants.Arm.armLengthMeters);
          ZaxisTarget.x = MathUtil.clamp(Constants.Arm.armLengthMeters * Math.cos(ArmAngle), Constants.Arm.armLengthMeters, 0);
      }
      }
      else{
        // System.out.println("2");
        Zone2_Equasion();
        RobotSpeedRelative_angle();
        if(IsQuikShot){
          ZaxisTarget = Constants.Arm.QuikShotPosition;
          ArmAngle = Units.degreesToRadians(180) - ArmAngle;
        }
        else{
          ZaxisTarget = Constants.Arm.Zone2_ArmPosition;
          ZaxisTarget.y = Constants.Arm.armLengthMeters * Math.sin(ArmAngle);
          ZaxisTarget.x = Constants.Arm.armLengthMeters * Math.cos(ArmAngle);
        }
      }
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
    private static void CalcDistance_withDxDy(){//i mean, its in the name, calcs the distance to the speaker
      getDx();
      getDy();
      distanceToSpeaker = Math.hypot(Dx, Dy);
    }
  
    private static void getWantedDegree(){//calcs the direction in which we want the gp to fly on
      YaxisWantedAngle = Units.degreesToRadians(180) - Math.atan(Dy/Dx);
    }
    private static void getChassisOffset(){//calcs offset do to speed
      YaxisOffset = Math.atan(VelocityY/VelocityX);
    }
    private static void CalcYaxisAngle(){//calcs the Chassis angle to the speaker relative to the speed
      getChassisOffset();
      ChasisAngle =  YaxisWantedAngle*2 - YaxisOffset;
      if (IsQuikShot) {
        ChasisAngle = Units.degreesToRadians(180) - ChasisAngle;
      }
    }
    private static void CalcVelocityXy_field(){//calcs the Velocity in Y and X axises
      VelocityY = StartSpeed*Math.cos(ArmAngle)*Math.sin(YaxisWantedAngle)
      + RobotContainer.driveBase.getAbsoluteChassisSpeeds().vyMetersPerSecond + RobotContainer.driveBase.getAbsoluteChassisSpeeds().omegaRadiansPerSecond
      * RobotContainer.arm.getShooterPosition().x * Math.sin(Units.degreesToRadians(-90) + YaxisWantedAngle);
      VelocityX = StartSpeed*Math.cos(ArmAngle)*(-1)*Math.cos(YaxisWantedAngle)
      + RobotContainer.driveBase.getAbsoluteChassisSpeeds().vxMetersPerSecond + RobotContainer.driveBase.getAbsoluteChassisSpeeds().omegaRadiansPerSecond
      * RobotContainer.arm.getShooterPosition().x * Math.cos(Units.degreesToRadians(-90) + YaxisWantedAngle);
    }
    public static void ResetParameters(){//resets the parameters for a new mode
    ZaxisTarget = new ArmPosition();//arm angle arm position
    StartSpeed = RobotContainer.arm.getShooterSub().getShooterVelocity();//the speed in which the gp is leaving the shooter
    Dy = 1;//y axis of targeted point (to calc the distance from speaker)
    Dx = 1;//airial distance to speaker
    distanceToSpeaker = 1;//self explanatory
    ArmAngle = 45;//the angle in which the arm shell be
    IsDeadZone = true;//is this a dead zone???
    IsQuikShot = false;//are we using quik shot????
    //chassis parameters
    YaxisWantedAngle = 0;//the angle in which we want the gp to fly in
    YaxisOffset = 1;//oh no! speed affects our vector, lets calc the offset!
    ChasisAngle = 0;//the final angle that the robot will face
    VelocityY = 1;//field relative
    VelocityX = 1;//field relative
    }

    public static void UpdateParameters(){//updates all the parameters so we can have our desired angles
      StartSpeed = RobotContainer.arm.getShooterSub().getAvrageShooterVelocity();
      CalcDistance_withDxDy();
      getWantedDegree();
      CalcAngleZaxis();
      CalcVelocityXy_field();
      if (IsQuikShot) {
      ZaxisTarget.rotation = ArmAngle;        
      }
      else{
        ZaxisTarget.rotation = 0;
      }
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
