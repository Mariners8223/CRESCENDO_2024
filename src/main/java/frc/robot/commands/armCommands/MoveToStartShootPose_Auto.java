// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Arm.knownArmPosition;

public class MoveToStartShootPose_Auto extends Command {
  /** Creates a new MoveToStartShootPose_Auto. */
  private static Arm arm;
  public MoveToStartShootPose_Auto() {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = Arm.getInstance();
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.moveMotorsToRotation(Units.degreesToRotations(139.3-0.05), 0.05);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    System.out.println("command ended");
    if (interrupted) {
      arm.lastknownPosition = knownArmPosition.Unknown;
    }
    else arm.lastknownPosition = knownArmPosition.Start_AutoShoot;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.isArmInPosition();
  }
}
