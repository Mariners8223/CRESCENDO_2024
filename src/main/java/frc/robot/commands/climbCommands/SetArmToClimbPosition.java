// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climbCommands;

import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest.RobotCentric;
import com.google.flatbuffers.Constants;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.climb.Elavator;

public class SetArmToClimbPosition extends Command {
  /** Creates a new SetArmToClimbPosition. */
  public SetArmToClimbPosition() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Arm.getInstance());
    addRequirements(Elavator.getInstance());
  }
  private static Arm arm;
  private static Elavator climb;


  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm = Arm.getInstance();
    climb = Elavator.getInstance();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    arm.moveShooterToPose(frc.robot.Constants.ClimbConstants.CLIMBING_POSTION);
    climb.SetClimbingHight(frc.robot.Constants.ClimbConstants.PUSH_ELAVATER_ARM_POSTION);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    //climb.moveClimbToPose(frc.robot.Constants.ClimbConstants.PULL_ELAVATER_ARM_POSTION); - in different command
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
