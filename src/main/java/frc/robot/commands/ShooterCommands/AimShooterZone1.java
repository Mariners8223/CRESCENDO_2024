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
public class AimShooterZone1 extends InstantCommand {
  private static ArmPosition target = new ArmPosition(0.4, 0.4, 0);
  private static double distanceToSpeaker;
  private static double Y;
  public static boolean IsDeadZone;

  public AimShooterZone1() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Arm.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotContainer.driveBase.getPose().getTranslation().getY() < Constants.Speaker.SpeakerIsCenterRatioBottomLocation) {
      IsDeadZone = true;
          Y = Constants.Speaker.SpeakerBottomLocationY
     + Constants.Speaker.SpeakerIsCenterRatioReverse * Constants.Speaker.SpeakerIsCenterRatioBottomLocation;
    }
    else{
      IsDeadZone = false;
          Y = Constants.Speaker.SpeakerBottomLocationY
     + Constants.Speaker.SpeakerIsCenterRatioReverse * RobotContainer.driveBase.getPose().getTranslation().getY();
    }    
    
    distanceToSpeaker = Math.sqrt(Math.pow(RobotContainer.driveBase.getPose().getTranslation().getX() - Constants.Speaker.SpeakerTranslation.getX(), 2) +
     Math.pow(Y, 2));

    target.rotation = Math.atan((Constants.Speaker.SpeakerTranslation.getZ() - target.y - Constants.DriveTrain.Global.RobotHeightFromGround) / distanceToSpeaker);

    Arm.getInstance().moveShooterToPose(target);
  }
}
