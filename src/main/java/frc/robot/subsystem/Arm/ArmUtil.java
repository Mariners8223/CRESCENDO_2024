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
  
    private static ArmPosition ZaxisTarget;//arm angle arm position
    private static ArmUtilInputsAutoLogged inputs;

    /**
   * calculates the distance from the robot to the speaker on the y axis
   * @return returns dy - the distance between the robot and the speaker on y axis
   */
    private static double calcDy(){
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
      return inputs.Dy;
    }

    /**
   * calculates the distance from the robot to the speaker on the x axis
   * @return returns dx - the distance between the robot and the speaker on x axis
   */
    private static double calcDx(){
      inputs.Dx = RobotContainer.driveBase.getPose().getTranslation().getX()
       - Constants.Speaker.SpeakerTranslation.getX();
      return inputs.Dx;
    }

    /**
   * calculates the distance from the robot to the speaker on the z axis
   * @return returns dz - the distance between the robot and the speaker on z axis
   */
    private static double CalcDz(){
      inputs.Dz = (Constants.Speaker.SpeakerTranslation.getZ() - Arm.getInstance().getShooterPosition().y);
      return inputs.Dz;
    }
    /**
     * calculates the arial distance between the robot and the speaker (xy axis)
     * @param Dy the distance between the robot and the speaker on y axis
     * @param Dx the distance between the robot and the speaker on x axis
     * @return returns the arial distance between the robot and the speaker
     */
    private static double CalcDistance_withDxDy(double Dy, double Dx){
      inputs.distanceToSpeaker = Math.hypot(Dx, Dy);

      if (inputs.IsQuikShot) {//adds or subtracks the distance from the center of the robot to the shotter from the distance to the speaker
        inputs.distanceToSpeaker += Arm.getInstance().getShooterPosition().x;
      }
      else inputs.distanceToSpeaker -= Arm.getInstance().getShooterPosition().x;
      return inputs.distanceToSpeaker;
    }

    /**
     * resets the parameters when needed
     */
    private static void ResetParameters(){//resets the parameters for a new mode
      if (inputs.isResetNeeded) {
        ZaxisTarget = Constants.Arm.QuikShotPosition;//arm angle arm position
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

    /**
     * Calculates the wanted game piece velocity in order to reach the goal the fastest
     * @param distanceToSpeaker the arial distance between the robot and the speaker
     * @return the wanted velocity
     */
    private static double CalcWantedSpeed(double distanceToSpeaker){
      if (inputs.isZone1) {
        inputs.WantedVelocity = distanceToSpeaker/Constants.Shooter.GPAirTimeZone1;
      }
      else{
        inputs.WantedVelocity = distanceToSpeaker/Constants.Shooter.GPAirTimeZone2;
      }
      inputs.WantedVelocity = MathUtil.clamp(inputs.WantedVelocity, Units.rotationsPerMinuteToRadiansPerSecond(3500) *  Constants.Shooter.wheelRadius, Units.rotationsPerMinuteToRadiansPerSecond(6000) * Constants.Shooter.wheelRadius);//shooting speed clamp
      inputs.StartSpeed = inputs.WantedVelocity;
      return inputs.WantedVelocity;
    }

    /**
     * calculate the wanted arial angle to the speaker (calcs the direction in which we want the gp to fly on)
     * @param Dy the distance between the robot and the speaker on y axis
     * @param Dx the distance between the robot and the speaker on x axis
     * @return the wanted arial angle to the speaker
     */
    private static double getWantedDegree(double Dy, double Dx){
      if (RobotContainer.isRedAllince.getAsBoolean()) {
        inputs.YaxisWantedAngle = Math.atan(Dy/Dx);
      }
      else inputs.YaxisWantedAngle = Units.degreesToRadians(180) - Math.atan(Dy/Dx);
      if (inputs.IsQuikShot) inputs.YaxisWantedAngle = Units.degreesToRadians(180) - inputs.YaxisWantedAngle;
      return inputs.YaxisWantedAngle;
    }

    /**
     * calcs the Velocity in X axises
     * @param ArmAngle the angle that the robot shall shoot from - xz axis
     * @param YaxisWantedAngle the angle in which the game piece will fly - xy axis
     * @return the velocity in x axis
     */
    private static double CalcVelocityX_field(double ArmAngle, double YaxisWantedAngle){
      inputs.VelocityX = inputs.StartSpeed*Math.cos(ArmAngle)*(-1)*Math.cos(YaxisWantedAngle)
        + RobotContainer.driveBase.getAbsoluteChassisSpeeds().vxMetersPerSecond + RobotContainer.driveBase.getAbsoluteChassisSpeeds().omegaRadiansPerSecond
        * RobotContainer.arm.getShooterPosition().x * Math.cos(Units.degreesToRadians(-90) + YaxisWantedAngle);
      return inputs.VelocityX;
    }

    /**
     * calcs the Velocity in Y axises
     * @param ArmAngle the angle that the robot shall shoot from (the arm angle) - xz axis
     * @param YaxisWantedAngle the angle in which the game piece will fly - xy axis
     * @return the velocity in y axis
     */
    private static double CalcVelocityY_field(double ArmAngle, double YaxisWantedAngle){
      inputs.VelocityY = inputs.StartSpeed*Math.cos(ArmAngle)*Math.sin(YaxisWantedAngle)
        + RobotContainer.driveBase.getAbsoluteChassisSpeeds().vyMetersPerSecond + RobotContainer.driveBase.getAbsoluteChassisSpeeds().omegaRadiansPerSecond
        * RobotContainer.arm.getShooterPosition().x * Math.sin(Units.degreesToRadians(-90) + YaxisWantedAngle);
      return inputs.VelocityY;
    }

    /**
     * calculates the offset duo to the robot velocity
     * @param VelocityY all the velocity in y axis
     * @param VelocityX all the velocity in x axis
     * @return the offset
     */
    private static double getChassisOffset(double VelocityY, double VelocityX){
      inputs.YaxisOffset = Math.atan(VelocityY/VelocityX);
      return inputs.YaxisOffset;
    }

    /**
     * calcs the Chassis angle to the speaker relative to the speed
     * @param YaxisWantedAngle the wanted arial angle to the speaker (the direction in which we want the gp to fly on)
     * @param YaxisOffset the offset duo to the robot velocity
     * @return the angle that the chassis should face
     */
    private static double CalcYaxisAngle(double YaxisWantedAngle, double YaxisOffset){
      inputs.ChassisAngle = YaxisWantedAngle//WORKS
      //  - inputs.YaxisOffset
       ;//once there was a *2 in here
      return inputs.ChassisAngle;
    }
  
    /**
     * calcs the angle that the arm should give in order for the game piece to reach its goal
     * @param Dz the distance between the robot and the speaker on z axis
     * @param distanceToSpeaker the arial distance between the robot and the speaker
     * @return the angle using simple trigo (simple lazer)
     */
    private static double Zone1_Equasion(double Dz, double distanceToSpeaker){
      try {
        inputs.ArmAngle = Math.atan(Dz/distanceToSpeaker);//if there is a problem, return last angle
      } catch (Exception e) {
      }
      return inputs.ArmAngle;
    }

    /**
     * calcs the angle that the arm should give in order for the game piece to reach its goal
     * @param StartSpeed the speed in which the gp leaves the shooter
     * @param Dz the distance between the robot and the speaker on z axis
     * @param distanceToSpeaker the arial distance between the robot and the speaker
     * @return the angle using a more complex equasion - phizics
     */
    private static double Zone2_Equasion(double StartSpeed, double Dz, double distanceToSpeaker){
      try {
        inputs.ArmAngle = Math.atan((Math.pow(StartSpeed, 2)
       - Math.sqrt(Math.pow(StartSpeed, 4)
        - 2*(Dz)//hieght
        *Constants.gGravity_phisics*Math.pow(StartSpeed, 2) - Math.pow(Constants.gGravity_phisics*distanceToSpeaker, 2)))
        /(distanceToSpeaker*Constants.gGravity_phisics)
      );
      } catch (Exception e) {
      }
      return inputs.ArmAngle;
    }
  
    /**
     * calculates the angle that the arm should have with consideration of the robots momeentom (velocity)
     * @param StartSpeed the speed in which the gp leaves the shooter
     * @param YaxisWantedAngle the wanted arial angle to the speaker (the direction in which we want the gp to fly on)
     * @param ArmAngle the angle that the robot shall shoot from (the arm angle) - xz axis
     * @return the angle using a more complex equasion - phizics
     */
    private static double RobotSpeedRelative_angle(double StartSpeed, double YaxisWantedAngle, double ArmAngle){
      try {
        inputs.ArmAngle = Math.abs(Math.atan(-(StartSpeed*Math.sin(ArmAngle))/(StartSpeed * Math.cos(ArmAngle)
        + RobotContainer.driveBase.getAbsoluteChassisSpeeds().vxMetersPerSecond * Math.cos(YaxisWantedAngle)
        + RobotContainer.driveBase.getAbsoluteChassisSpeeds().vyMetersPerSecond * Math.sin(YaxisWantedAngle))));
      } catch (Exception e) {
      }
      return inputs.ArmAngle;
    }
  
    /**
     * calculates the angle in xz axis that the game piece should fly in (the arm angle), also chosses
     * if to aim as quick shoot or regular shoot and if its zone 1 or 2
     * @param StartSpeed the speed in which the gp leaves the shooter
     * @param Dz the distance between the robot and the speaker on z axis
     * @param distanceToSpeaker the arial distance between the robot and the speaker
     * @param YaxisWantedAngle the wanted arial angle to the speaker (the direction in which we want the gp to fly on)
     * @return the angle using a more complex equasion - phizics
     */
    private static double CalcAngleZaxis(double StartSpeed, double Dz, double distanceToSpeaker, double YaxisWantedAngle){
      if (inputs.distanceToSpeaker <= Constants.Arm.EndOfZone1) {
        inputs.ArmAngle = Zone1_Equasion(Dz, distanceToSpeaker);
        // inputs.ArmAngle = RobotSpeedRelative_angle(StartSpeed, YaxisWantedAngle, inputs.ArmAngle);
        if(inputs.IsQuikShot){
          ZaxisTarget = Constants.Arm.QuikShotPosition;
          inputs.ArmAngle = Units.degreesToRadians(180) - inputs.ArmAngle;
        }
        else{
          ZaxisTarget = Constants.Arm.Zone1_ArmPosition;
          ZaxisTarget.y = MathUtil.clamp(Constants.Arm.armLengthMeters * Math.sin(inputs.ArmAngle), 0, Constants.Arm.armLengthMeters);
          ZaxisTarget.x = MathUtil.clamp(Constants.Arm.armLengthMeters * Math.cos(inputs.ArmAngle), Constants.Arm.armLengthMeters, 0);
        }
      }
      else{
        inputs.ArmAngle = Zone2_Equasion(StartSpeed, Dz, distanceToSpeaker);
        // inputs.ArmAngle = RobotSpeedRelative_angle(StartSpeed, YaxisWantedAngle, inputs.ArmAngle);
        if(inputs.IsQuikShot){
          ZaxisTarget = Constants.Arm.QuikShotPosition;
          inputs.ArmAngle = Units.degreesToRadians(180) - inputs.ArmAngle;
        }
        else{
          ZaxisTarget = Constants.Arm.Zone2_ArmPosition;
          ZaxisTarget.y = Constants.Arm.armLengthMeters * Math.sin(inputs.ArmAngle);
          ZaxisTarget.x = Constants.Arm.armLengthMeters * Math.cos(inputs.ArmAngle);
        }
      }
      return inputs.ArmAngle;
    }

    /**
     * updates all the parameters so we can have our desired angles
     */
    public static void UpdateParameters(){
      ResetParameters();

      inputs.Dx = calcDx();
      inputs.Dy = calcDy();
      inputs.Dz = CalcDz();

      inputs.distanceToSpeaker = CalcDistance_withDxDy(inputs.Dy, inputs.Dy);

      if (inputs.isZone1 != (inputs.distanceToSpeaker <= Constants.Arm.EndOfZone1)) {
        inputs.isResetNeeded = true;
      }
      inputs.isZone1 = inputs.distanceToSpeaker <= Constants.Arm.EndOfZone1;

      inputs.WantedVelocity = CalcWantedSpeed(inputs.distanceToSpeaker);
      inputs.YaxisWantedAngle = getWantedDegree(inputs.Dy, inputs.Dx);

      inputs.ArmAngle = CalcAngleZaxis(inputs.StartSpeed, inputs.Dz, inputs.distanceToSpeaker, inputs.YaxisWantedAngle);

      inputs.VelocityX = CalcVelocityX_field(inputs.ArmAngle, inputs.YaxisWantedAngle);
      inputs.VelocityY = CalcVelocityY_field(inputs.ArmAngle, inputs.YaxisWantedAngle);

      if (inputs.IsQuikShot) {
        ZaxisTarget.rotation = inputs.ArmAngle;        
      }
      else{
        ZaxisTarget.rotation = 0;
      }

      inputs.YaxisOffset = getChassisOffset(inputs.VelocityY, inputs.VelocityX);
      inputs.ChassisAngle = CalcYaxisAngle(inputs.YaxisWantedAngle, inputs.YaxisOffset);

      //Climb
      inputs.IndexOfClimbingRope = Constants.Elevator.SlidingPositions.SlidingPositions_MiddleRope.indexOf(//gets the index of the clossest rope
        RobotContainer.driveBase.getPose().getTranslation().nearest(Constants.Elevator.SlidingPositions.SlidingPositions_MiddleRope)
      );

      Logger.processInputs("ArmUtl", inputs);
    }

    /**
     * sets the mode to or from quick shoot
     * @param QuikShot should quick shoot be used
     */
    public static void SetQuikShotMode(boolean QuikShot){
      if (inputs.IsQuikShot != QuikShot) {
        inputs.isResetNeeded = true;
      }
      inputs.IsQuikShot = QuikShot;
    }

    /**
     * get the wanted angle in which the robot should fire the game piece
     * @return the arm angle
     */
    public static double getArmAngle(){
      return inputs.ArmAngle;
    }

    /**
     * get the angle in which the robot should face while firing the game piece
     * @return the chassis angle
     */
    public static double getChassisAngle(){
      return inputs.ChassisAngle;
    }

    /**
     * get the armposition for the wanted arm angle
     * @return the ArmPosition
     */
    public static ArmPosition getArmNeededPosition(){
      return ZaxisTarget;
    }

    /**
     * get the distance between the robot and the speaker on x axis
     * @return Dx
     */
    public static double getDx(){
      return inputs.Dx;
    }

    /**
     * get the distance between the robot and the speaker on y axis
     * @return Dy
     */
    public static double getDy(){
      return inputs.Dy;
    }

    /**
     * get the distance between the robot and the speaker on z axis
     * @return Dz
     */
    public static double getDz(){
      return inputs.Dz;
    }

    /**
     * get the status of the robot
     * @return if the robot is in a bad aiming zone
     */
    public static boolean getIsDeadZone(){
      return inputs.IsDeadZone;
    }

    /**
     * get the speed that the game piece should fly in
     * @return the speed in m/s
     */
    public static double getWantedSpeed(){
      return inputs.WantedVelocity;
    }

    /**
     * get the robot status
     * @return if the robot is in zone 1
     */
    public static boolean isZone1(){
      return inputs.isZone1;
    }
    //ZONESSSS
    public static enum ArmZones{
      ShootingZone,
      PickUpZone,
      SourceZone
    }
    /**
     * get the zone that the robot is in
     * @return
     */
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

    /**
     * get the index of the clossest rope (the one  you want to climb)
     * @return the index of the rope in the array in constants
     */
    public static int getIndexOfClimbingRope(){
      return inputs.IndexOfClimbingRope;
    }

    //Aim to amp area
    /**
     * calcs the velocity, the distance and the angle that the robot should have in order to shoot to the amp area
     */
    private static void CalcAimToAmp(){
      inputs.alpha = Math.atan((Constants.AmpPose.getY() - RobotContainer.driveBase.getPose().getY())
        /(Constants.AmpPose.getX() - RobotContainer.driveBase.getPose().getX()));

      inputs.d = Math.hypot(Constants.AmpPose.getX() - RobotContainer.driveBase.getPose().getX(), 
        Constants.AmpPose.getY() - RobotContainer.driveBase.getPose().getY());
      
      inputs.v = Math.sqrt((Constants.gGravity_phisics*inputs.d)
        /(2*Math.tan(Arm.getInstance().getShooterPosition().rotation)
        *Math.pow(Math.cos(Arm.getInstance().getShooterPosition().rotation), 2)));
    }

    /**
     * get the angle that the robot should face in order to shoot to the amp area
     * @return chassis angle
     */
    public static double getChassisAngle_ToAmp(){
      CalcAimToAmp();
      return inputs.alpha;
    }

    /**
     * get the wanted velocity that the gp should have in order to get to the AMP area
     * @return the speed in m/s
     */
    public static double getWantedVelocity_ToAmp(){
      CalcAimToAmp();
      return inputs.v;
    }
  }
