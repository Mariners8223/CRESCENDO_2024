// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands.Collect;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Intake.Intake;

public class Collect2 extends Command {
  /** Creates a new Collect2. */
  private static Intake intake;
  public Collect2() {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = Arm.getInstance().getIntakeSub();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("collect2 finish - note is in");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !intake.getLaserReading();
  }
}
