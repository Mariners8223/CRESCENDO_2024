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
    timer = 0;

    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.getShooter().setShooterPower(0.8);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(timer == 30)
      arm.getIntake().setMotor(0.8);

    timer++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.getShooter().stopMotors();
    arm.getIntake().stopMotor();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return !arm.getIntake().isGamePieceDetected() || timer >= 150;
  }
}
