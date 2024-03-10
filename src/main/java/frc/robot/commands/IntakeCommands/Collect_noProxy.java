// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Intake.Intake;

public class Collect_noProxy extends SequentialCommandGroup {

  public Collect_noProxy() {
    addCommands(
      new CollectNoProxy1(),
      new WaitCommand(0.05),
      new InstantCommand(() -> Arm.getInstance().getIntakeSub().setMotor(-1)),
      new WaitCommand(0.05),
      new InstantCommand(() -> Arm.getInstance().getIntakeSub().stopMotor()));
  }


  private static class CollectNoProxy1 extends Command{
  /** Creates a new Collect_noProxy. */
  private static Intake intake;
  public CollectNoProxy1() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake = Arm.getInstance().getIntakeSub();
    intake.setMotor(0.8);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false; 
  }
}
}
