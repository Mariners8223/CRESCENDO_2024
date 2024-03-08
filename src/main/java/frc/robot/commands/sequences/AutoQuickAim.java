// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCommands.IntakeToFloor;
import frc.robot.commands.ShooterCommands.QuickAim.QuickAim1;
import frc.robot.subsystem.Arm.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AutoQuickAim extends SequentialCommandGroup {
  /** Creates a new AutoQuikAim. */
  public AutoQuickAim() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeToFloor().onlyIf(() -> Arm.getInstance().lastknownPosition != Arm.knownArmPosition.Intake),
      new QuickAim1()
    );
  }
}
