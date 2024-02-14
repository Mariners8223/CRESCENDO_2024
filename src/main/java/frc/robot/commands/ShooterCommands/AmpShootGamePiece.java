// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;


import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Shooter.Shooter;

public class AmpShootGamePiece extends Command {
  /** Creates a new AmpShootGamePiece. */
  private static Arm arm;
  private static Shooter shooter;

  private double startTime;
  
  public AmpShootGamePiece() {
    arm = Arm.getInstance();
    shooter = arm.getShooterSub();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooterPower(Constants.Shooter.ShootToAmpPower);
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (Timer.getFPGATimestamp() - startTime >= Constants.Shooter.ShootToAmpTime);
  }
}
