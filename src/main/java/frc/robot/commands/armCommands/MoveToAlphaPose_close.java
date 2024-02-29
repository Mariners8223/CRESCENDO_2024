// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Arm.ArmPosition;

public class MoveToAlphaPose_close extends Command {
  /** Creates a new AlphaShoot_close. */
  private static Arm arm;
  private static ArmPosition target;

  public MoveToAlphaPose_close() {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = Arm.getInstance();
    
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // target = Constants.Arm.AlphaFromClose.copyArmPostion();
    // arm.moveShooterToPose(target);

    arm.moveMotorsToRotation(Units.degreesToRotations(57.7), 0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("command ended");
      if(!interrupted) arm.lastknownPosition = Arm.knownArmPosition.AlphaAim_close;
      else arm.lastknownPosition = Arm.knownArmPosition.Unknown;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isArmInPosition();
  }
}
