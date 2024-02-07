// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Arm.Arm;

public class moveSecondaryMotorToDegree extends Command {
  /** Creates a new moveSecondaryMotorToDegree. */
  private Arm arm;
  private double degrees;

  public moveSecondaryMotorToDegree(double degrees) {
    arm = Arm.getInstance();
    this.degrees = degrees;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.moveIntakeToDegree(degrees);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isArmInPosition();
  }
}
