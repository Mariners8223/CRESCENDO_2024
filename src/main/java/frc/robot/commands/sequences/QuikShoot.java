// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeCommands.Collect;
import frc.robot.commands.ShooterCommands.Shoot;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class QuikShoot extends SequentialCommandGroup {
  /** Creates a new QuikShoot. */
  public QuikShoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> RobotContainer.arm.moveIntakeToPose(
        Constants.ArmConstants.FloorPosition, frc.robot.subsystem.Arm.Arm.ControlType.Rotation)),//move intake to collect
      new InstantCommand(() -> RobotContainer.arm.getShooter().setShooterPower(0.8)),//start the shooting prosses
      new ParallelCommandGroup(
        new Collect(),
        new InstantCommand(() -> RobotContainer.driveBase.findPath(Constants.AutoConstants.MiddleNote))),//get game piece
      new InstantCommand(() -> RobotContainer.arm.moveShooterToPose(Constants.AutoConstants.FastShootPose)),//aim
      new Shoot()//shoot
    );
  }
}
