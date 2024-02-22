// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystem.Arm;

import frc.robot.subsystem.Arm.Arm.ArmPosition;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;
import frc.robot.RobotContainer;

/** Add your docs here. */
public class ArmUtil{

  @AutoLog
  public static class ArmUtilInputs{
    ArmPosition ZaxisTarget;//arm angle arm position
    double StartSpeed;//the speed in which the gp is leaving the shooter
    double Dy;//y axis of targeted point (to calc the distance from speaker)
    double Dx;//airial distance to speaker
    double Dz;//The hight of the targeted point on the speaker relative to the shooter hieght from ground
    double distanceToSpeaker;//self explanatory
    double ArmAngle = 45;//the angle in which the arm shell be
    boolean IsDeadZone;//is this a dead zone???
    boolean IsQuikShot = true;//are we using quik shot????
    //chassis parameters
    double YaxisWantedAngle;//the angle in which we want the gp to fly in
    double YaxisOffset;//oh no! speed affects our vector, lets calc the offset!
    double ChassisAngle;//the final angle that the robot will face
    double VelocityY;//field relative
    double VelocityX;//field relative
    boolean isResetNeeded = true;
    boolean isZone1;
    double WantedVelocity;

    int IndexOfClimbingRope;//Updated in update parameters

    double d;//the distance to the amp
    double alpha;//the angle the robot should face to aim to the amp
    double v;//the velocity the gp should fly in in-order to get to the wanted position near the amp
  }
  

    private static ArmUtilInputsAutoLogged inputs;

    private static void calcDy(){
      if (RobotContainer.driveBase.getPose().getTranslation().getY() <= Constants.Arm.SpeakerIsCenterRatioBottomLocation) {
        inputs.IsDeadZone = true;
        inputs.Dy = -(Constants.Arm.SpeakerBottomLocationY + Constants.Arm.SpeakerLength - Constants.Arm.SpeakerIsCenterRatioBottomLocation);//aime to the most right corner (robot prespective)
      }
      else{//may be changed to a single point
        inputs.IsDeadZone = false;
        inputs.Dy = -(Constants.Arm.SpeakerBottomLocationY + Constants.Arm.SpeakerLength - Constants.Arm.SpeakerIsCenterRatio
         * (RobotContainer.driveBase.getPose().getTranslation().getY()// + Arm.getInstance().getShooterPosition().x*Math.sin(ChasisAngle)
         - Constants.Arm.SpeakerIsCenterRatioBottomLocation)
         -(//Arm.getInstance().getShooterPosition().x*Math.sin(ChasisAngle)
         + RobotContainer.driveBase.getPose().getTranslation().getY()));//aim to a point prespective to the robot location in the chosen shooting zone
      }
    }
    private static void calcDx(){
      inputs.Dx = RobotContainer.driveBase.getPose().getTranslation().getX()
       - Constants.Speaker.SpeakerTranslation.getX();
    }
    private static void CalcDz(){
      inputs.Dz = (Constants.Speaker.SpeakerTranslation.getZ() - Arm.getInstance().getShooterPosition().y);
    }
    private static void CalcDistance_withDxDy(){//i mean, its in the name, calcs the distance to the speaker
      calcDx();
      calcDy();
      CalcDz();
      inputs.distanceToSpeaker = Math.hypot(inputs.Dx, inputs.Dy);

      if (inputs.IsQuikShot) {//adds or subtracks the distance from the center of the robot to the shotter from the distance to the speaker
        inputs.distanceToSpeaker += Arm.getInstance().getShooterPosition().x;
      }
      else inputs.distanceToSpeaker -= Arm.getInstance().getShooterPosition().x;
      // System.out.println("Dx " + Dx);
      // System.out.println("Dy " + Dy);
    }

    private static void ResetParameters(){//resets the parameters for a new mode
      if (inputs.isResetNeeded) {
        inputs.ZaxisTarget = Constants.Arm.QuikShotPosition;//arm angle arm position
        // StartSpeed = RobotContainer.arm.getShooterSub().getShooterVelocity();//the speed in which the gp is leaving the shooter
        inputs.WantedVelocity = 0;
        inputs.Dy = 1;//y axis of targeted point (to calc the distance from speaker)
        inputs.Dx = 1;//airial distance to speaker
        inputs.Dz = 1;//The hight of the targeted point on the speaker relative to the shooter hieght from ground
        inputs.distanceToSpeaker = 1;//self explanatory
        inputs.ArmAngle = 45;//the angle in which the arm shell be
        inputs.IsDeadZone = true;//is this a dead zone???
        //chassis parameters
        inputs.YaxisWantedAngle = 0;//the angle in which we want the gp to fly in
        inputs.YaxisOffset = 1;//oh no! speed affects our vector, lets calc the offset!
        inputs.ChassisAngle = 0;//the final angle that the robot will face
        inputs.VelocityY = 1;//field relative
        inputs.VelocityX = 1;//field relative
        inputs.isResetNeeded = false;
        inputs.isZone1 = inputs.distanceToSpeaker <= Constants.Arm.EndOfZone1;
      }
    }

    private static void CalcWantedSpeed(){
      if (inputs.isZone1) {
        inputs.WantedVelocity = inputs.distanceToSpeaker/Constants.Shooter.GPAirTimeZone1;
      }
      else{
        inputs.WantedVelocity = inputs.distanceToSpeaker/Constants.Shooter.GPAirTimeZone2;
      }
      inputs.WantedVelocity = MathUtil.clamp(inputs.WantedVelocity, Units.rotationsPerMinuteToRadiansPerSecond(3500) *  Constants.Shooter.wheelRadius, Units.rotationsPerMinuteToRadiansPerSecond(6000) * Constants.Shooter.wheelRadius);//shooting speed clamp
      inputs.StartSpeed = inputs.WantedVelocity;
    }

    private static void getWantedDegree(){//calcs the direction in which we want the gp to fly on
      if (RobotContainer.isRedAllince.getAsBoolean()) {
        inputs.YaxisWantedAngle = Math.atan(inputs.Dy/inputs.Dx);
      }
      else inputs.YaxisWantedAngle = Units.degreesToRadians(180) - Math.atan(inputs.Dy/inputs.Dx);
      if (inputs.IsQuikShot) inputs.YaxisWantedAngle = Units.degreesToRadians(180) - inputs.YaxisWantedAngle;
    }
    private static void getChassisOffset(){//calcs offset do to speed
      inputs.YaxisOffset = Math.atan(inputs.VelocityY/inputs.VelocityX);
    }
    private static void CalcYaxisAngle(){//calcs the Chassis angle to the speaker relative to the speed
      getChassisOffset();
      inputs.ChassisAngle = inputs.YaxisWantedAngle//WORKS
      //  - inputs.YaxisOffset
       ;//once there was a *2 in here
    }
  
    private static void Zone1_Equasion(){//simple lazer for zone 1
      try {
        inputs.ArmAngle = Math.atan(inputs.Dz/inputs.distanceToSpeaker);//if there is a problem, return last angle
      } catch (Exception e) {
      }
    }
    private static void Zone2_Equasion(){//complicated for zone 2
      try {
        inputs.ArmAngle = Math.atan((Math.pow(inputs.StartSpeed, 2)
       - Math.sqrt(Math.pow(inputs.StartSpeed, 4)
        - 2*(inputs.Dz)//hieght
        *Constants.gGravity_phisics*Math.pow(inputs.StartSpeed, 2) - Math.pow(Constants.gGravity_phisics*inputs.distanceToSpeaker, 2)))
        /(inputs.distanceToSpeaker*Constants.gGravity_phisics)
      );
      } catch (Exception e) {
      }
    }
  
    private static void RobotSpeedRelative_angle(){//calcs the arm angle with speed relativity
      try {
        inputs.ArmAngle = Math.abs(Math.atan(-(inputs.StartSpeed*Math.sin(inputs.ArmAngle))/(inputs.StartSpeed * Math.cos(inputs.ArmAngle)
        + RobotContainer.driveBase.getAbsoluteChassisSpeeds().vxMetersPerSecond * Math.cos(inputs.YaxisWantedAngle)
        + RobotContainer.driveBase.getAbsoluteChassisSpeeds().vyMetersPerSecond * Math.sin(inputs.YaxisWantedAngle))));
      } catch (Exception e) {
      }
    }
  
    private static void CalcAngleZaxis(){//chooses zone 1 or 2 or quik shot modes
      // System.out.println("shit");
      if (inputs.distanceToSpeaker <= Constants.Arm.EndOfZone1) {
        // System.out.println("1");
        Zone1_Equasion();
        // RobotSpeedRelative_angle();
        if(inputs.IsQuikShot){
          inputs.ZaxisTarget = Constants.Arm.QuikShotPosition;
          inputs.ArmAngle = Units.degreesToRadians(180) - inputs.ArmAngle;
        }
        else{
          inputs.ZaxisTarget = Constants.Arm.Zone1_ArmPosition;
          inputs.ZaxisTarget.y = MathUtil.clamp(Constants.Arm.armLengthMeters * Math.sin(inputs.ArmAngle), 0, Constants.Arm.armLengthMeters);
          inputs.ZaxisTarget.x = MathUtil.clamp(Constants.Arm.armLengthMeters * Math.cos(inputs.ArmAngle), Constants.Arm.armLengthMeters, 0);
      }
      }
      else{
        // System.out.println("2");
        Zone2_Equasion();
        // RobotSpeedRelative_angle();
        if(inputs.IsQuikShot){
          inputs.ZaxisTarget = Constants.Arm.QuikShotPosition;
          inputs.ArmAngle = Units.degreesToRadians(180) - inputs.ArmAngle;
        }
        else{
          inputs.ZaxisTarget = Constants.Arm.Zone2_ArmPosition;
          inputs.ZaxisTarget.y = Constants.Arm.armLengthMeters * Math.sin(inputs.ArmAngle);
          inputs.ZaxisTarget.x = Constants.Arm.armLengthMeters * Math.cos(inputs.ArmAngle);
        }
      }
    }
  
    
    private static void CalcVelocityXy_field(){//calcs the Velocity in Y and X axises
      inputs.VelocityY = inputs.StartSpeed*Math.cos(inputs.ArmAngle)*Math.sin(inputs.YaxisWantedAngle)
      + RobotContainer.driveBase.getAbsoluteChassisSpeeds().vyMetersPerSecond + RobotContainer.driveBase.getAbsoluteChassisSpeeds().omegaRadiansPerSecond
      * RobotContainer.arm.getShooterPosition().x * Math.sin(Units.degreesToRadians(-90) + inputs.YaxisWantedAngle);
      inputs.VelocityX = inputs.StartSpeed*Math.cos(inputs.ArmAngle)*(-1)*Math.cos(inputs.YaxisWantedAngle)
      + RobotContainer.driveBase.getAbsoluteChassisSpeeds().vxMetersPerSecond + RobotContainer.driveBase.getAbsoluteChassisSpeeds().omegaRadiansPerSecond
      * RobotContainer.arm.getShooterPosition().x * Math.cos(Units.degreesToRadians(-90) + inputs.YaxisWantedAngle);
    }

    public static void UpdateParameters(){//updates all the parameters so we can have our desired angles
      ResetParameters();
      CalcDistance_withDxDy();
      if (inputs.isZone1 != (inputs.distanceToSpeaker <= Constants.Arm.EndOfZone1)) {
        inputs.isResetNeeded = true;
      }
      inputs.isZone1 = inputs.distanceToSpeaker <= Constants.Arm.EndOfZone1;
      CalcWantedSpeed();
      getWantedDegree();
      CalcAngleZaxis();
      CalcVelocityXy_field();
      if (inputs.IsQuikShot) {
        inputs.ZaxisTarget.rotation = inputs.ArmAngle;        
      }
      else{
        inputs.ZaxisTarget.rotation = 0;
      }
      CalcYaxisAngle();

      //Climb
      inputs.IndexOfClimbingRope = Constants.Elevator.SlidingPositions.SlidingPositions_MiddleRope.indexOf(//gets the index of the clossest rope
        RobotContainer.driveBase.getPose().getTranslation().nearest(Constants.Elevator.SlidingPositions.SlidingPositions_MiddleRope)
      );
      Logger.processInputs("ArmUtl", inputs);
    }
    public static void SetQuikShotMode(boolean QuikShot){
      if (inputs.IsQuikShot != QuikShot) {
        inputs.isResetNeeded = true;
      }
      inputs.IsQuikShot = QuikShot;
    }
    public static double getArmAngle(){
      return inputs.ArmAngle;
    }
    public static double getChassisAngle(){
      return inputs.ChassisAngle;
    }
    public static ArmPosition getArmNeededPosition(){
      return inputs.ZaxisTarget;
    }

    public static double getDx(){
      return inputs.Dx;
    }
    public static double getDy(){
      return inputs.Dy;
    }
    public static double getDz(){
      return inputs.Dz;
    }
    public static boolean getIsDeadZone(){
      return inputs.IsDeadZone;
    }
    public static double getWantedSpeed(){
      return inputs.WantedVelocity;
    }
    public static boolean isZone1(){
      return inputs.isZone1;
    }
    //ZONESSSS
    public static enum ArmZones{
      ShootingZone,
      PickUpZone,
      SourceZone
    }
    public static ArmZones getZones(){
      switch (Constants.robotZones.indexOf(RobotContainer.driveBase.getPose().getTranslation().nearest(Constants.robotZones))) {
        case 0:
          return ArmZones.ShootingZone;
        case 1:
          return ArmZones.PickUpZone;
        case 2:
          return ArmZones.SourceZone;
        default:
          return ArmZones.PickUpZone;
      }
    }
    //Climb Shit

    public static int getIndexOfClimbingRope(){
      return inputs.IndexOfClimbingRope;
    }

    //Aim to amp area
    private static void CalcAimToAmp(){
      inputs.alpha = Math.atan((Constants.AmpPose.getY() - RobotContainer.driveBase.getPose().getY())
        /(Constants.AmpPose.getX() - RobotContainer.driveBase.getPose().getX()));

      inputs.d = Math.hypot(Constants.AmpPose.getX() - RobotContainer.driveBase.getPose().getX(), 
        Constants.AmpPose.getY() - RobotContainer.driveBase.getPose().getY());
      
      inputs.v = Math.sqrt((Constants.gGravity_phisics*inputs.d)
        /(2*Math.tan(Arm.getInstance().getShooterPosition().rotation)
        *Math.pow(Math.cos(Arm.getInstance().getShooterPosition().rotation), 2)));
    }
    public static double getChassisAngle_ToAmp(){
      CalcAimToAmp();
      return inputs.alpha;
    }
    public static double getWantedVelocity_ToAmp(){
      CalcAimToAmp();
      return inputs.v;
    }
  }
