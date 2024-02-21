// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.armCommands.MoveToFree;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Arm.knownArmPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RegularShootingPosition extends SequentialCommandGroup {
  /** Creates a new RegularShootingPosition. */
  public RegularShootingPosition() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new MoveToFree().onlyIf(() -> Arm.getInstance().lastknownPosition != knownArmPosition.Free),
    new MoveIntakeIn());
  }

  public static class MoveIntakeIn extends Command{
    private Arm arm;

    public MoveIntakeIn() {
      arm = Arm.getInstance();
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(arm);
    }

    @Override
    public void initialize() {
      arm.moveMotorsToRotation(Arm.getInstance().getMainMotorRotation(), 0);
    }

    @Override
    public boolean isFinished() {
      return arm.isArmInPosition();
    }
  }
}
