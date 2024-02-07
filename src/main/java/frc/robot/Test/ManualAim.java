// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Test;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Arm.ArmPostion;

public class ManualAim extends Command {
  CommandPS5Controller controller;
  Arm arm;

  ArmPostion target = new ArmPostion();
  /** Creates a new ManualAim. */
  public ManualAim() {
    controller = RobotContainer.driveController;
    arm = Arm.getInstance();
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(arm);

    target.y = 0.4;
    target.rotation = arm.getShooterPosition().rotation;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    arm.moveShooterToPose(target);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    target.rotation = target.rotation + Units.degreesToRadians(controller.getR2Axis());
    target.rotation = target.rotation - Units.degreesToRadians(controller.getL2Axis());

    arm.moveShooterToPose(target);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
