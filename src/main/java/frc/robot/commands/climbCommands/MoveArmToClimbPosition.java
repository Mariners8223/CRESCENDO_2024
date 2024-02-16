// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Arm.Arm;

public class MoveArmToClimbPosition extends Command {
  /** Creates a new MoveArmToClimbPosition. */
  Arm arm;
  public MoveArmToClimbPosition() {
      arm = Arm.getInstance();
      // addRequirements();
      addRequirements(arm);  }

  @Override
  public void initialize(){
    arm.moveMotorsToRotation(0.25, 0.5);
  }

  @Override
  public boolean isFinished(){
    return arm.isArmInPosition();
  }
}
