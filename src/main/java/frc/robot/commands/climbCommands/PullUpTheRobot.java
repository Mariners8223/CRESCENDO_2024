// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Arm.climb.Elavator;

public class PullUpTheRobot extends Command {
  /** Creates a new PullUpTheRobot. */
  private static Elavator climb;
  public PullUpTheRobot() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Elavator.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climb = Elavator.getInstance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    climb.SetClimbingHight(frc.robot.Constants.ClimbConstants.PULL_ELAVATER_ARM_POSTION);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
