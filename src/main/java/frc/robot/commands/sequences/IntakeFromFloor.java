// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCommands.Collect;
import frc.robot.commands.IntakeCommands.IntakeToFloor;
import frc.robot.commands.IntakeCommands.IntakeToFloor.MoveIntakeNumber;
import frc.robot.commands.armCommands.MoveToFree;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeFromFloor extends SequentialCommandGroup {
  /** Creates a new IntakeFromFloor. */
  public IntakeFromFloor() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // new MoveToFree(),
      // new MoveIntakeNumber(false),
      // new MoveIntakeNumber(true),
      new Collect()
    );
  }
}