// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.ShooterCommands.AimShooter;
import frc.robot.commands.ShooterCommands.RegularShootingPosition;
import frc.robot.commands.armCommands.MoveToHome;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Arm.knownArmPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimRegularToSpeaker extends SequentialCommandGroup {
  /** Creates a new AimToSpeaker. */
  public AimRegularToSpeaker() {
    AimShooter Aim = new AimShooter();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    
    addCommands(
      new MoveToHome().onlyIf(() -> Arm.getInstance().lastknownPosition != knownArmPosition.Home),//moves to free within
      new RepeatCommand(Aim)
    );
  }
}
