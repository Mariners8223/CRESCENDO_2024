// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands.Collect;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Intake.Intake;

public class Collect1 extends Command {
  /** Creates a new Collect1. */
  private static Intake intake;
  public Collect1() {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = Arm.getInstance().getIntakeSub();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.setMotor(0.8);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("Collect 1 finish, game piece is in");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.isGamePieceDetected() || intake.getLaserReading();
  }
}
