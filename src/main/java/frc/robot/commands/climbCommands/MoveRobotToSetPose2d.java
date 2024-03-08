// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climbCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;

public class MoveRobotToSetPose2d extends Command {
  /** Creates a new MoveRobotToSetPose2d. */
  Pose2d target;
  int timer;
  public MoveRobotToSetPose2d(Pose2d target) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.target = target;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    RobotContainer.driveBase.findPath(this.target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("got to position");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return RobotContainer.driveBase.getPose().equals(target) || timer > 15;
  }
}
