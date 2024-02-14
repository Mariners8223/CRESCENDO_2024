// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Arm.ArmPostion;
import frc.robot.subsystem.Arm.Shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimShooter extends InstantCommand {
  private static Arm arm;
  private static ArmPostion target = new ArmPostion(0.4, 0.4, 0);
  private static double StartSpeed;
  private static double Dy;//y axis of targeted point (to calc the distance from speaker)
  private static double Dx;//airial distance to speaker
  private static double distanceToSpeaker;
  private static double angle = 45;
  public static boolean IsDeadZone;

  public AimShooter() {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = Arm.getInstance();
    addRequirements(arm);
  }

  public double getAngle(){//for the turret aim
    return angle;
  }
  public double getDy(){
    return Dy;
  }
  public double getDx(){
    return Dx;
  }

  public double Zone1_Equasion(){//simple lazer for zone 1
    try {
      return Math.atan((Constants.SpeakerTranslation.getZ() - target.y - Constants.ArmConstants.RobotHightFromGround)//hieght
      /distanceToSpeaker);//if there is a problem, return last angle
    } catch (Exception e) {
      return angle;
    }
  }
  public double Zone2_Equasion(){//complicated for zone 2
    try {
      return Math.atan((Math.pow(StartSpeed, 2)
     - Math.sqrt(Math.pow(StartSpeed, 4)
      - 2*(Constants.SpeakerTranslation.getZ() - target.y - Constants.ArmConstants.RobotHightFromGround)//hieght
      *Constants.gGravity_phisics*Math.pow(StartSpeed, 2) - Math.pow(Constants.gGravity_phisics*Dx, 2)))
      /(Dx*Constants.gGravity_phisics)
    );
    } catch (Exception e) {
      return angle;
    }
  }
  public double RobotSpeedRelative_angle(){
    try {
      return (StartSpeed*Math.sin(angle))/(StartSpeed*Math.cos(angle) + RobotContainer.driveBase.getChassisSpeeds().vxMetersPerSecond);
    } catch (Exception e) {
      return angle;
    }
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    StartSpeed = Shooter.getInstance().getShooterVelocity();
    //Get Aiming Point
    //at this moment Dy represents the actual Y axis on pose2d for the point we are aiming at
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
    Dx = RobotContainer.driveBase.getPose().getTranslation().getX() - Constants.SpeakerTranslation.getX();
    //Get distance
    distanceToSpeaker = Math.hypot(Dx, Dy);
    //Choose zone 1 or 2
    if (distanceToSpeaker <= Constants.ArmConstants.EndOfZone1) {
      target = Constants.ArmConstants.Zone1_ArmPosition;
      angle = Zone1_Equasion();
    }
    else{
      target = Constants.ArmConstants.Zone2_ArmPosition;
      angle = Zone2_Equasion();
    }
    //get true angle with robot speed
    angle = RobotSpeedRelative_angle();
    //aim
    target.rotation = Math.atan(angle);
    arm.moveShooterToPose(target);
  }
}
