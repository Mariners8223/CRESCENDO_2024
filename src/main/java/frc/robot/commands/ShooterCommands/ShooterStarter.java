// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.ArmUtil;
import frc.robot.subsystem.Arm.Shooter.Shooter;

public class ShooterStarter extends Command {
  /** THIS COMMAND WILL STOP ONLY IF YOU CANCEL IT */
  private static Shooter shooter;
  double startTime;
  public ShooterStarter() {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = Arm.getInstance().getShooterSub();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // ArmUtil.UpdateParameters_SpeakerAim();
    shooter.setShooterVelocity(ArmUtil.getWantedSpeed());
    startTime = Timer.getFPGATimestamp();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if (interrupted) {
      shooter.stopMotors();
    }
    shooter.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Timer.getFPGATimestamp() - startTime > 10 && !Arm.getInstance().getIntakeSub().isGamePieceDetected();
    return false;
  }
}
