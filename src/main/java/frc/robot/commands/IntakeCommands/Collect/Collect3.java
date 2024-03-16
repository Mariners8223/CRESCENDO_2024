// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands.Collect;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Intake.Intake;

public class Collect3 extends Command {
  /** Creates a new Collect3. */
  private static Intake intake;
  private static int timer;
  public Collect3() {
    // Use addRequirements() here to declare subsystem dependencies.
    intake = Arm.getInstance().getIntakeSub();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.stopMotor();
    if (timer >= 20) {//if the note is stuck, get it out
      intake.setPosition(intake.getMotorPosition() - 2.5);
    }
    intake.setIsGamePieceDetected(true);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return intake.getLaserReading() || timer >= 20;
  }
}
