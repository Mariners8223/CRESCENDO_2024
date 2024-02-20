// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.climbCommands.ClimbElevator;
import frc.robot.commands.climbCommands.MoveArmToClimbPosition;
import frc.robot.commands.climbCommands.MoveRobotToSetPose2d;

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
    
    addCommands(//TODO
    new ParallelCommandGroup(
      new MoveRobotToSetPose2d(Constants.Elevator.SlidingPositions.InStageMiddleLocations_POSE2D.get(index)),
      new MoveArmToClimbPosition()
    ),
    new ClimbElevator(true),
    new MoveRobotToSetPose2d(Constants.Elevator.SlidingPositions.UnderRopeMiddleLocations_POSE2D.get(index)),
    new ClimbElevator(false)
    );
  }
}
