// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands.Collect;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystem.Arm.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Collect extends SequentialCommandGroup {
  /** Creates a new Collect. */
  public Collect() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // addCommands(new Collect_1stDraft(),
    //  new WaitCommand(0.6),
    //   new InstantCommand(() -> {Arm.getInstance().getShooterSub().stopMotors(); Arm.getInstance().getIntakeSub().stopMotor();}));
    addCommands(new Collect1().onlyIf(() -> !Arm.getInstance().getIntakeSub().isGamePieceDetected()),// maybe it shouldbe !Arm.getInstance().getIntakeSub().getLaserReading();
    new Collect2().onlyIf(() -> !Arm.getInstance().getIntakeSub().isGamePieceDetected()),
    new Collect3().onlyIf(() -> !Arm.getInstance().getIntakeSub().isGamePieceDetected()));
  }
}
