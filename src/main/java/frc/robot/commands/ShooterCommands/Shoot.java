// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Arm.Arm;

public class Shoot extends Command {
  /** Creates a new Shoot. */

  private Arm arm;
  private int timer;

  public Shoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = Arm.getInstance();

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.getShooterSub().setShooterPower(0.8);
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
    arm.getIntakeSub().setMotor(1);
    Timer.delay(0.1);

    arm.getIntakeSub().stopMotor();
    arm.getShooterSub().stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer >= 50;
  }
}
