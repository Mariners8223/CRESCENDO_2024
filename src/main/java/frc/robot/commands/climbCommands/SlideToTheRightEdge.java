// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climbCommands;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.climb.Elavator;

public class SlideToTheRightEdge extends InstantCommand {
  /** Creates a new SlideToTheRightEdge. */
  private static Elavator climb;
  private static Translation2d target;
  private static int RopeIndex;
  public SlideToTheRightEdge() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Arm.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {//TODO
    climb = Arm.getInstance().getElavatorSub();
    target = RobotContainer.driveBase.getPose().getTranslation().nearest(Constants.Elevator.SlidingPositions.SlidingPositions_MiddleRope);
    RopeIndex = Constants.Elevator.SlidingPositions.SlidingPositions_MiddleRope.indexOf(target);
    target = Constants.Elevator.SlidingPositions.SlidingPositions_RightEdgeRope.get(RopeIndex);
    climb.setRollerMotor(-(RobotContainer.driveBase.getPose().getTranslation().getDistance(target) * Constants.Elevator.AirialMetersToRopeLength));
    //check if right is + or - for the motor
  }
}
