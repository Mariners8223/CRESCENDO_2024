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

/** Add your docs here.
 * ArmUtil is the aiming mechanizem of the robot
 * it aims and calculates every thing that has to do with the arm of our 2024 robot
 */
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
    boolean IsQuikShot;//are we using quik shot????
    //chassis parameters
    double YaxisWantedAngle;//the angle in which we want the gp to fly in
    double YaxisOffset;//oh no! speed affects our vector, lets calc the offset!
    double ChassisAngle;//the final angle that the robot will face
    double VelocityY;//field relative
    double VelocityX;//field relative
    boolean isResetNeeded;
    boolean isZone1;
    double WantedVelocity;

    double GamePieceVelocityX;
    double GamePieceVelocityZ;

    int IndexOfClimbingRope;//Updated in update parameters

    //AMP SHIT

    // double d;//the distance to the amp
    // double alpha;//the angle the robot should face to aim to the amp
    // double v;//the velocity the gp should fly in in-order to get to the wanted position near the amp

    double AmpVelocity;//the needed velocity to amp
    double ArmAngle_AMP;//the arm angle to amp
    double DistanceToAMP;//the distance to amp
    double ChassisAngle_AMP;//the chassis angle to the amp
    boolean isBetaShoot_AMP;//whether we aim with alpha or beta
    boolean isAmpShot;
  }
  
    private static ArmPosition ZaxisTarget;//arm angle arm position
    private static ArmUtilInputsAutoLogged inputs = new ArmUtilInputsAutoLogged();
    private static boolean isFirstReset = true;

    /**
   * calculates the distance from the robot to the speaker on the y axis
   * @return returns dy - the distance between the robot and the speaker on y axis
   */
    private static double calcDy(){
      if (RobotContainer.driveBase.getPose().getTranslation().getY() <= Constants.Arm.SpeakerIsCenterRatioBottomLocation) {
        inputs.IsDeadZone = true;
        // inputs.Dy = -(Constants.Arm.SpeakerBottomLocationY + Constants.Arm.SpeakerLength - Constants.Arm.SpeakerIsCenterRatioBottomLocation);//aime to the most right corner (robot prespective)
      }
      else{//may be changed to a single point
        inputs.IsDeadZone = false;
        // inputs.Dy = -(Constants.Arm.SpeakerBottomLocationY + Constants.Arm.SpeakerLength - Constants.Arm.SpeakerIsCenterRatio
        //  * (RobotContainer.driveBase.getPose().getTranslation().getY()// + Arm.getInstance().getShooterPosition().x*Math.sin(ChasisAngle)
        //  - Constants.Arm.SpeakerIsCenterRatioBottomLocation)
        //  -(//Arm.getInstance().getShooterPosition().x*Math.sin(ChasisAngle)
        //  + RobotContainer.driveBase.getPose().getTranslation().getY()));//aim to a point prespective to the robot location in the chosen shooting zone
      }
      inputs.Dy = Constants.Arm.SpeakerMidlleLocationY - RobotContainer.driveBase.getPose().getY();
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
      if(inputs.IsQuikShot){
        inputs.Dz = (Constants.Speaker.SpeakerTranslation.getZ() - 0.4)
        + 0.25;//Arm.getInstance().getShooterPosition().y);
        // TODO: check if its better to do the calc with just minus 0.4 meters - the arm hieght
      }
      else{
        inputs.Dz = (Constants.Speaker.SpeakerTranslation.getZ() - Constants.Arm.armHeightFromFrameMeters
         - Constants.DriveTrain.Global.RobotHeightFromGround)
        + Constants.Arm.DistanceFromMainArmToShooterOutput
        + 0.12;
        // + CalcAlphaOffset(inputs.ArmAngle);//adds the distance between the main arm and where the gp is flying out of
        // inputs.Dz = Constants.Speaker.SpeakerTranslation.getZ() - Arm.getInstance().getShooterPosition().y;
      }
      return inputs.Dz;
    }

    /**
     * calculates the offset duo to the the angle of the arm
     * @param MainAngle the angle that the main arm should face
     * @return the offset of the the angle of the arm adds to the hieght of the speaker
     */
    public static double CalcAlphaOffset(double MainAngle){
      return (Constants.Arm.armLengthMeters*Math.sin(Constants.Arm.Motors.secondarySoftLimits[1]))/(Math.cos(MainAngle));
    }
    
    /**
     * calculates the arial distance between the robot and the speaker (xy axis)
     * @param Dy the distance between the robot and the speaker on y axis
     * @param Dx the distance between the robot and the speaker on x axis
     * @return returns the arial distance between the robot and the speaker
     */
    private static double CalcDistance_withDxDy(double Dy, double Dx){
      inputs.distanceToSpeaker = Math.hypot(Dy, Dx);

      if (inputs.IsQuikShot) {//adds or subtracks the distance from the center of the robot to the shotter from the distance to the speaker
        inputs.distanceToSpeaker += Math.abs(Arm.getInstance().getShooterPosition().x);
      }
      // else inputs.distanceToSpeaker += Constants.Arm.mainPivotDistanceFromCenterMeters;// Constants.Speaker.AlphaShootOffset_distance;
      return inputs.distanceToSpeaker;// TODO: check if the comment works
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
        // inputs.ArmAngle = 45;//the angle in which the arm shell be
        inputs.IsDeadZone = true;//is this a dead zone???
        //chassis parameters
        inputs.YaxisWantedAngle = 0;//the angle in which we want the gp to fly in
        inputs.YaxisOffset = 1;//oh no! speed affects our vector, lets calc the offset!
        inputs.ChassisAngle = 0;//the final angle that the robot will face
        inputs.VelocityY = 1;//field relative
        inputs.VelocityX = 1;//field relative
        inputs.isResetNeeded = false;
        inputs.isZone1 = inputs.distanceToSpeaker <= Constants.Arm.EndOfZone1;
        inputs.GamePieceVelocityX = 1;
        inputs.GamePieceVelocityZ = 1;

        if (isFirstReset) {
        inputs.AmpVelocity = 1;
        inputs.ArmAngle_AMP = 30;
        inputs.DistanceToAMP = 1;
        inputs.isAmpShot = false;
        }
      }
    }

    /**
     * resets all parametes so we wont have problems
     */
    public static void StartArmUtil(){
      if (isFirstReset) {
        ResetParameters();
        isFirstReset = false;
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
      // inputs.GamePieceVelocityX = CalcVelocityX_GP(inputs.Dz, inputs.Dz, distanceToSpeaker);
      // inputs.GamePieceVelocityZ = ClacVelocityZ_GP(distanceToSpeaker);
      // inputs.WantedVelocity = Math.hypot(inputs.GamePieceVelocityX, inputs.GamePieceVelocityZ );

      inputs.WantedVelocity = MathUtil.clamp(inputs.WantedVelocity, Units.rotationsPerMinuteToRadiansPerSecond(3500) *  Constants.Shooter.wheelRadius, Units.rotationsPerMinuteToRadiansPerSecond(5500) * Constants.Shooter.wheelRadius);//shooting speed clamp
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
      if (RobotContainer.isBlueAllince.getAsBoolean()) {
        inputs.YaxisWantedAngle = Math.atan(Dy/Dx);
      }
      else inputs.YaxisWantedAngle = Units.degreesToRadians(180) - Math.atan(Dy/Dx);
      if (inputs.IsQuikShot) inputs.YaxisWantedAngle = Units.degreesToRadians(180) + inputs.YaxisWantedAngle;
      else inputs.YaxisWantedAngle = inputs.YaxisWantedAngle%(2 * Math.PI);
      return inputs.YaxisWantedAngle;
    }

    /**
     * calcs the Velocity in X axises
     * @param ArmAngle the angle that the robot shall shoot from - xz axis
     * @param YaxisWantedAngle the angle in which the game piece will fly - xy axis
     * @return the velocity in x axis
     */
    private static double CalcRobotVelocityX_field(double ArmAngle, double YaxisWantedAngle){
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
    private static double CalcRobotVelocityY_field(double ArmAngle, double YaxisWantedAngle){
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
     * calculates the needed Arm Angle to reach the targetted place
     * @param MaxHieght the max hieght the game piece is allowed to get *IN METERS*
     * @param TargetHieght the hieght in which you wish the game piece to finigh at *IN METERS*
     * @param distanceToSpeaker the starting distance between the game piece and the speaker *IN METERS*
     * @return the Arm angle
     */
    public static double Zone2_Equasion_NEW(double MaxHieght, double TargetHieght, double distanceToSpeaker){
      inputs.GamePieceVelocityX = CalcVelocityX_GP(inputs.Dz, inputs.Dz, distanceToSpeaker);
      inputs.GamePieceVelocityZ = ClacVelocityZ_GP(distanceToSpeaker);
      return Math.atan(inputs.GamePieceVelocityZ/inputs.GamePieceVelocityX);
    }

    public static double Zone2_Equasion_overShoot(double MaxHieght, double distanceToSpeaker){
      return Math.atan(2*(MaxHieght/distanceToSpeaker));
    }

    /**
     * calculates the needed velocity on the z axis in order to get to the targeted hieght 
     * @param MaxHieght the max hieght the game piece is allowed to get *IN METERS*
     * @return the velocity in the z axis
     */
    public static double ClacVelocityZ_GP(double MaxHieght){
      if (MaxHieght >= 0) return Math.sqrt(2*MaxHieght* Constants.gGravity_phisics);
      return inputs.GamePieceVelocityZ;
    }

    /**
     * calculates the needed velocity on the x axis in order to get to the targeted distance
     * @param MaxHieght the max hieght the game piece is allowed to get *IN METERS*
     * @param TargetHieght the hieght in which you wish the game piece to finigh at *IN METERS*
     * @param distanceToSpeaker the starting distance between the game piece and the speaker *IN METERS*
     * @return the velocity in the x axis
     */
    public static double CalcVelocityX_GP(double MaxHieght, double TargetHieght, double distanceToSpeaker){
      if (MaxHieght >= TargetHieght && TargetHieght >= 0)
      return (Constants.gGravity_phisics * distanceToSpeaker)
        /(Math.sqrt(2* Constants.gGravity_phisics*MaxHieght) + Math.sqrt(2*Constants.gGravity_phisics*(MaxHieght - TargetHieght)));
      return inputs.GamePieceVelocityX;
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
    private static double CalcAngleZaxis(double StartSpeed, double Dz, double distanceToSpeaker, double YaxisWantedAngle) {
      if (inputs.isZone1) {
        inputs.ArmAngle = Zone1_Equasion(Dz, distanceToSpeaker);
        // inputs.ArmAngle = RobotSpeedRelative_angle(StartSpeed, YaxisWantedAngle, inputs.ArmAngle);
        if(inputs.IsQuikShot){
          ZaxisTarget = Constants.Arm.QuikShotPosition;
          inputs.ArmAngle = Units.degreesToRadians(180) - inputs.ArmAngle;
        }
        else {
          ZaxisTarget = Constants.Arm.Zone1_ArmPosition;
          ZaxisTarget.y = MathUtil.clamp(Constants.Arm.armLengthMeters * Math.sin(inputs.ArmAngle), 0, Constants.Arm.armLengthMeters);
          ZaxisTarget.x = MathUtil.clamp(Constants.Arm.armLengthMeters * Math.cos(inputs.ArmAngle), Constants.Arm.armLengthMeters, 0);
        }
      }
      else {
        // inputs.ArmAngle = Zone2_Equasion(StartSpeed, Dz, distanceToSpeaker);
        inputs.ArmAngle = Zone2_Equasion_overShoot(Dz - 0.8, distanceToSpeaker);
        // inputs.ArmAngle = Zone2_Equasion_NEW(Dz, Dz, distanceToSpeaker);
        // inputs.ArmAngle = RobotSpeedRelative_angle(StartSpeed, YaxisWantedAngle, inputs.ArmAngle);
        // inputs.ArmAngle = Zone1_Equasion(Dz + 0.1, distanceToSpeaker);

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
    public static void UpdateParameters_SpeakerAim(){
      ResetParameters();

      inputs.Dx = calcDx();
      inputs.Dy = calcDy();
      inputs.Dz = CalcDz();

      inputs.distanceToSpeaker = CalcDistance_withDxDy(inputs.Dy, inputs.Dx);

      // if (inputs.isZone1 != (inputs.distanceToSpeaker <= Constants.Arm.EndOfZone1)) {
      //   inputs.isResetNeeded = true;
      // }
      inputs.isZone1 = inputs.distanceToSpeaker <= Constants.Arm.EndOfZone1;
      // inputs.isZone1 = CalcWantedSpeed(inputs.distanceToSpeaker) < 18.6;

      inputs.WantedVelocity = CalcWantedSpeed(inputs.distanceToSpeaker);
      inputs.YaxisWantedAngle = getWantedDegree(inputs.Dy, inputs.Dx);

      inputs.ArmAngle = CalcAngleZaxis(inputs.StartSpeed, inputs.Dz, inputs.distanceToSpeaker, inputs.YaxisWantedAngle);

      inputs.VelocityX = CalcRobotVelocityX_field(inputs.ArmAngle, inputs.YaxisWantedAngle);
      inputs.VelocityY = CalcRobotVelocityY_field(inputs.ArmAngle, inputs.YaxisWantedAngle);

      if (inputs.IsQuikShot) {
        ZaxisTarget.rotation = inputs.ArmAngle;        
      }
      else{
        ZaxisTarget.rotation = 0;
      }

      inputs.YaxisOffset = getChassisOffset(inputs.VelocityY, inputs.VelocityX);
      inputs.ChassisAngle = CalcYaxisAngle(inputs.YaxisWantedAngle, inputs.YaxisOffset);

      //Climb
      // inputs.IndexOfClimbingRope = Constants.Elevator.SlidingPositions.SlidingPositions_MiddleRope.indexOf(//gets the index of the clossest rope
      //   RobotContainer.driveBase.getPose().getTranslation().nearest(Constants.Elevator.SlidingPositions.SlidingPositions_MiddleRope)
      // );

      Logger.processInputs("ArmUtil", inputs);
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

    /**
     * return if the robot is in quick shot mode
     */
    public static boolean isQuickShot(){
      return inputs.IsQuikShot;
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

    // //Aim to amp area
    // /**
    //  * calcs the velocity, the distance and the angle that the robot should have in order to shoot to the amp area
    //  * THIS IS BETA AIM WITH VELOCITY, PROBABLY WONT WORK
    //  */
    // private static void CalcAimToAmp(){
    //   inputs.alpha = Math.atan((Constants.AmpPose.getY() - RobotContainer.driveBase.getPose().getY())
    //     /(Constants.AmpPose.getX() - RobotContainer.driveBase.getPose().getX()));

    //   inputs.d = Math.hypot(Constants.AmpPose.getX() - RobotContainer.driveBase.getPose().getX(), 
    //     Constants.AmpPose.getY() - RobotContainer.driveBase.getPose().getY());
      
    //   inputs.v = Math.sqrt((Constants.gGravity_phisics*inputs.d)
    //     /(2*Math.tan(Arm.getInstance().getShooterPosition().rotation)
    //     *Math.pow(Math.cos(Arm.getInstance().getShooterPosition().rotation), 2)));
    // }

    /**
     * returns the distance between the robot and the point it was given
     * @param x the x axis value of the target IN METERS
     * @param y the y axis value of the target IN METERS
     * @return the distance
     */
    private static double CalcDistanceToSetPoint(double x, double y){
      return Math.hypot(x - RobotContainer.driveBase.getPose().getX(), y - RobotContainer.driveBase.getPose().getY());
    }

    /**
     * calculates the velocity on the x axis in order to get to the same hieght as we started
     * @param MaxHieght the maximum hieght the gp is allowed to get IN METER
     * @param DistnaceToAMP the distance to the targeted place IN METERS
     * @return the velocity in M/S
     */
    private static double CalcVelocityAMPX_GP(double MaxHieght, double DistanceToAMP){
      if (MaxHieght >= 0) {
        return (2*Math.sqrt(2*MaxHieght* Constants.gGravity_phisics) * DistanceToAMP)/ Constants.gGravity_phisics;
      }
      return 1;
    }

    /**
     * calcs the arm angle for the amp area
     * @param MaxHieght the maximum hieght the gp is allowed to get IN METER
     * @param Velocity the velocity that the gp shell travel IN M/S
     * @return the arm angle in RAD
     */
    private static double CalcArmAngleToAMP(double MaxHieght, double Velocity){
      if (Velocity != 0 && MaxHieght >= 0) {
        return Math.asin((Math.sqrt(MaxHieght*2* Constants.gGravity_phisics))/Velocity);
      }
      return inputs.ArmAngle_AMP;
    }

    /**
     * calculates the chassis angle to a set point
     * @param x the x axis value of the target
     * @param y the y axis value of the target
     * @return the angle in RAD
     */
    private static double CalcChassisAngleToSetPoint(double x, double y){
      if (x != RobotContainer.driveBase.getPose().getX()) {
        return Math.atan((y - RobotContainer.driveBase.getPose().getY())/(x - RobotContainer.driveBase.getPose().getX()));
      }
      return Units.degreesToRadians(90);
    }

    /**
     * updates the parameters for the amp area shooting calculations
     */
    public static void UpdateParameters_AMPAim(){
      inputs.DistanceToAMP = CalcDistanceToSetPoint(Constants.Speaker.ampTranslation.getX(), Constants.Speaker.ampTranslation.getY());

      inputs.AmpVelocity = Math.hypot(CalcVelocityAMPX_GP(3.5, inputs.DistanceToAMP), ClacVelocityZ_GP(3.5));
      
      inputs.AmpVelocity = MathUtil.clamp(inputs.AmpVelocity, Units.rotationsPerMinuteToRadiansPerSecond(3500) *  Constants.Shooter.wheelRadius, Units.rotationsPerMinuteToRadiansPerSecond(5500) * Constants.Shooter.wheelRadius);//shooting speed clamp

      inputs.WantedVelocity = inputs.AmpVelocity;
      
      inputs.ArmAngle_AMP = CalcArmAngleToAMP(3.5, inputs.AmpVelocity);

      inputs.ChassisAngle_AMP = CalcChassisAngleToSetPoint(Constants.Speaker.ampTranslation.getX(), Constants.Speaker.ampTranslation.getY());

      if (RobotContainer.isBlueAllince.getAsBoolean()) {
        inputs.ChassisAngle_AMP = Units.degreesToRadians(180) - inputs.ChassisAngle_AMP;
      }

      if (inputs.isBetaShoot_AMP) {
        inputs.ArmAngle_AMP = Units.degreesToRadians(180) - inputs.ArmAngle_AMP;
        inputs.ChassisAngle_AMP += 180;
      }
    }

    /**
     * changes the mode, wether to update the amp shot or the speaker shoot
     * @param isAmpShot is it amp shoot
     */
    public static void setIsAmpShot(boolean isAmpShot){
      inputs.isAmpShot = isAmpShot;
    }

    /**
     * return the needed velocity for the amp area
     * @return the velocity in m/s
     */
    public static double getWantedVelocity_ToAmp(){
      return inputs.AmpVelocity;
    }

    /**
     * return the needed armangle for the amp area
     * @return the arm angle in RAD
     */
    public static double getArmAngle_ToAMP(){
      return inputs.ArmAngle_AMP;
    }
    
    /**
     * returns the chassis angle for aimming to amp area
     * @return the angle in RAD
     */
    public static double getChassisAngle_ToAmp(){
      return inputs.ChassisAngle_AMP;
    }

    /**
     * set wether you use the alpha or beta shot
     * @param isBeta is it beta?
     */
    public static void setIsBetaShoot_amp(boolean isBeta){
      inputs.isBetaShoot_AMP = isBeta;
    }

    // /**
    //  * get the angle that the robot should face in order to shoot to the amp area
    //  * @return chassis angle
    //  */
    // public static double getChassisAngle_ToAmp(){
    //   CalcAimToAmp();
    //   return inputs.alpha;
    // }

    // /**
    //  * get the wanted velocity that the gp should have in order to get to the AMP area
    //  * @return the speed in m/s
    //  */
    // public static double getWantedVelocity_ToAmp(){
    //   CalcAimToAmp();
    //   return inputs.v;
    // }

    /**
     * returns the arm angle if it in was zone 1
     */
    public static double getZone1(){
      return Zone1_Equasion(inputs.Dz, inputs.distanceToSpeaker);
    }

    /**
     * returns the arm angle if it in was zone 2
     */
    public static double getZone2(){
      return Zone2_Equasion(inputs.StartSpeed, inputs.Dz, inputs.distanceToSpeaker);
    }

    public static double getNEWZone2(){
      return Zone2_Equasion_NEW(inputs.Dz, inputs.Dz, inputs.distanceToSpeaker);
    }

    /**updates parameters*/
    public static void UpdateParameters(){
      if (inputs.isAmpShot) {
        UpdateParameters_AMPAim();
      }
      else{
        UpdateParameters_SpeakerAim();
      }
    }
  }
