// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.climb.Elavator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbToNearestRope extends SequentialCommandGroup {
  /** Creates a new ClimbToNearestRope. */
  public ClimbToNearestRope() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    int index = Constants.Elevator.SlidingPositions.SlidingPositions_MiddleRope.indexOf(
      RobotContainer.driveBase.getPose().getTranslation().nearest(
        Constants.Elevator.SlidingPositions.SlidingPositions_MiddleRope));
    
    addCommands(
    new InstantCommand(() -> //go in stage and face the rope closest to you
     RobotContainer.driveBase.findPath(Constants.Elevator.SlidingPositions.InStageMiddleLocations_POSE2D.get(index))),
    new InstantCommand(() -> // move the arm to the climbing position
     Arm.getInstance().moveShooterToPose(frc.robot.Constants.Elevator.CLIMBING_POSTION)),
    new InstantCommand(() -> // move the elavator to the climbing position
     Arm.getInstance().getElavatorSub().SetClimbingHight(frc.robot.Constants.Elevator.PUSH_ELAVATER_ARM_POSTION)),
    new InstantCommand(() -> // move the robot to the rope
     RobotContainer.driveBase.findPath(Constants.Elevator.SlidingPositions.UnderRopeMiddleLocations_POSE2D.get(index))),
    new InstantCommand(() -> //pull yourself up
    Arm.getInstance().getElavatorSub().SetClimbingHight(frc.robot.Constants.Elevator.PULL_ELAVATER_ARM_POSTION))
    );
  }
}
