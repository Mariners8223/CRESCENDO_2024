// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Arm.ArmPostion;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimShooterZone1 extends InstantCommand {
  private static ArmPostion target = new ArmPostion(0.4, 0.4, 0);
  private static double distanceToSpeaker;
  private static double angleToSpeaker;

  public AimShooterZone1() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Arm.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    distanceToSpeaker = RobotContainer.driveBase.getPose().getTranslation().getDistance(Constants.SpeakerTranslation.toTranslation2d()) - target.x;

    angleToSpeaker = Math.atan((Constants.SpeakerTranslation.getZ() - target.y) / distanceToSpeaker);

    target.rotation = angleToSpeaker;

    Arm.getInstance().moveShooterToPose(target);
  }
}
