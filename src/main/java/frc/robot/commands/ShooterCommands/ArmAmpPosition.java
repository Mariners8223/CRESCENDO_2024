// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.subsystem.Arm.Arm;

public class ArmAmpPosition extends InstantCommand {
  /** Creates a new ArmAmpPosition. */
  private static Arm arm;
  public ArmAmpPosition() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm = Arm.getInstance();
    arm.moveShooterToPose(Constants.ArmConstants.AmpArmPosition);
  }
}
