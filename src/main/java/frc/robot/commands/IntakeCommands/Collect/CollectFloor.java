// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands.Collect;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Intake.Intake;

public class CollectFloor extends Command {
  Intake intake;
  double startTime;
  boolean WasGamePieceDetected;
  /** Creates a new CollectFloor. */
  public CollectFloor() {
    intake = Arm.getInstance().getIntakeSub();
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    WasGamePieceDetected = intake.isGamePieceDetected();
    if(!WasGamePieceDetected) intake.setMotor(0.8);
    else intake.setMotor(-0.6);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    if(interrupted){
      intake.stopMotor();
      if(WasGamePieceDetected) WasGamePieceDetected = false;
    }
    else if(!WasGamePieceDetected){
      intake.setPosition(intake.getMotorPosition());
      intake.setIsGamePieceDetected(true);
    }
    else{
      intake.stopMotor();
      intake.setIsGamePieceDetected(false);
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (intake.getLaserReading() && !WasGamePieceDetected) || (WasGamePieceDetected && Timer.getFPGATimestamp() - startTime >= 0.5);
  }
}
