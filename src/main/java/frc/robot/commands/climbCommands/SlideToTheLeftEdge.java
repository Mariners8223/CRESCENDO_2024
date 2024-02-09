// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climbCommands;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.climb.Elavator;

public class SlideToTheLeftEdge extends Command {
  /** Creates a new SlideToClosestRopeEdge. */
  private static Elavator climb;
  private static Translation2d target;
  public static int RopeIndex;

  public SlideToTheLeftEdge() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Arm.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    climb = Elavator.getInstance();
    target = RobotContainer.driveBase.getPose().getTranslation().nearest(Constants.ClimbConstants.SlidingPositions.SlidingPositions_MiddleRope);
    RopeIndex = Constants.ClimbConstants.SlidingPositions.SlidingPositions_MiddleRope.indexOf(target);
    target = Constants.ClimbConstants.SlidingPositions.SlidingPositions_RightEdgeRope.get(RopeIndex);
    climb.moveRobotOnRope(RobotContainer.driveBase.getPose().getTranslation().getDistance(target) * Constants.ClimbConstants.MotorRotationsToAirialMeters);
    //check if left is + or - for the motor
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //climb.moveRobotOnRope(RobotContainer.driveBase.getPose().getTranslation().getDistance(target) * Constants.ClimbConstants.MotorRotationsToAirialMeters);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    climb.HoldInPlace();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
