// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.commands.ShooterCommands.QuickAim;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Arm.knownArmPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FastAim extends ParallelCommandGroup {
  /** Creates a new FastAim. */
  public FastAim() {//this command chosses the best shooting position for the current arm position
    addCommands(
      new QuickAim().onlyIf(() -> Arm.getInstance().lastknownPosition == knownArmPosition.Intake
      && Arm.getInstance().lastknownPosition != knownArmPosition.Amp
      && Arm.getInstance().lastknownPosition != knownArmPosition.Shooter),
      new AimRegularToSpeaker().onlyIf(() -> Arm.getInstance().lastknownPosition != knownArmPosition.Intake)
    );
  }
}
