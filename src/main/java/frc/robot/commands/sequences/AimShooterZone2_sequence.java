// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.TurretAimToSpeaker;
import frc.robot.commands.ShooterCommands.AimShooterZone2;
import frc.robot.subsystem.Arm.Shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimShooterZone2_sequence extends SequentialCommandGroup {
  /** Creates a new AimShooterZone2_sequence. */
  public AimShooterZone2_sequence() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new InstantCommand(() -> Shooter.getInstance().setShooterPower(0.8)),
      new ParallelCommandGroup(new AimShooterZone2(),
      new TurretAimToSpeaker())
    );
  }
}
