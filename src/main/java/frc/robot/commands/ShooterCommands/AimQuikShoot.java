// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Arm.ArmPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimQuikShoot extends InstantCommand {
  private static Arm arm;
  private static ArmPosition target = new ArmPosition(0,0,0);
  private static double angle;
  private static double distanceToSpeaker;
  public AimQuikShoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = Arm.getInstance();
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distanceToSpeaker = Math.hypot(Constants.Arm.SpeakerBottomLocationY
     + Constants.Arm.SpeakerLength - Constants.Arm.SpeakerIsCenterRatio * (RobotContainer.driveBase.getPose().getTranslation().getY()
      - Constants.Arm.SpeakerIsCenterRatioBottomLocation), RobotContainer.driveBase.getPose().getX() + arm.getShooterPosition().x - Constants.SpeakerTranslation.getX());
    
    angle = Math.atan((Constants.SpeakerTranslation.getZ() - arm.getShooterPosition().y)/distanceToSpeaker);
    try {
      target.rotation = Math.atan((RobotContainer.arm.getShooterSub().getShooterVelocity()*Math.sin(angle))
    /(RobotContainer.arm.getShooterSub().getShooterVelocity()*Math.cos(angle) + RobotContainer.driveBase.getChassisSpeeds().vxMetersPerSecond));
    } catch (Exception e) {
      target.rotation = angle;
    }
    target.rotation = Math.atan((RobotContainer.arm.getShooterSub().getShooterVelocity()*Math.sin(angle))
    /(RobotContainer.arm.getShooterSub().getShooterVelocity()*Math.cos(angle) + RobotContainer.driveBase.getChassisSpeeds().vxMetersPerSecond));
    //TODO: check if the angle needs to be reverted bcz the shooter needs to shoot backwards - from the collection position
    Arm.getInstance().moveShooterToPose(target);
  }
}
