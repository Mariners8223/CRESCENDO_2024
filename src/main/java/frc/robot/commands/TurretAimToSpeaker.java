// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TurretAimToSpeaker extends InstantCommand {
  private static double SpeedOffset;
  private static double WantedDegree;
  private static double YaxisOfTargetInSpeaker;
  public TurretAimToSpeaker() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    YaxisOfTargetInSpeaker = Constants.ArmConstants.SpeakerBottomLocationY
    + Constants.ArmConstants.SpeakerLength - Constants.ArmConstants.SpeakerIsCenterRatio * (RobotContainer.driveBase.getPose().getTranslation().getY()
     - Constants.ArmConstants.SpeakerIsCenterRatioBottomLocation);//this is the y axis of the wanted point
    YaxisOfTargetInSpeaker = YaxisOfTargetInSpeaker - RobotContainer.driveBase.getPose().getY();//distance between robot and speaker on y axis
    
    WantedDegree = Units.degreesToRadians(180) - Math.atan(YaxisOfTargetInSpeaker/(RobotContainer.driveBase.getPose().getTranslation().getX() - Constants.SpeakerTranslation.getX()));
    SpeedOffset = Math.atan((RobotContainer.driveBase.getChassisSpeeds().vyMetersPerSecond +
    RobotContainer.driveBase.getChassisSpeeds().omegaRadiansPerSecond * RobotContainer.arm.getIntakePosition().x)
    /(RobotContainer.driveBase.getChassisSpeeds().vxMetersPerSecond
    + RobotContainer.arm.getShooter().getShooterVelocity()*Math.cos(RobotContainer.arm.getShooter().getTrueGamePieceVelocityAngle_RobotRelative_ArialView())));//not very accurate
    RobotContainer.driveBase.setTargetRotation(Rotation2d.fromDegrees(WantedDegree + SpeedOffset));
  }
}
