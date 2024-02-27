// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Arm.ArmPosition;
import frc.robot.subsystem.Arm.Arm.knownArmPosition;

public class AlphaAim_close extends Command {
  /** Creates a new AlphaShoot_close. */
  private static Arm arm;
  private static ArmPosition target;

  public AlphaAim_close() {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = Arm.getInstance();
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    target = Constants.Arm.AlphaFromClose.copyArmPostion();
    arm.moveShooterToPose(target);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    arm.lastknownPosition = knownArmPosition.AlphaAim_close;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isArmInPosition();
  }
}
