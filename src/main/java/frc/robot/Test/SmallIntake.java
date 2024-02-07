// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Test;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommands.IntakeToFloor;
import frc.robot.subsystem.Arm.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SmallIntake extends SequentialCommandGroup {
  /** Creates a new SmallIntake. */
  public SmallIntake() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveToFree(),
      new IntakeToFloor()
    );
  }

  public static class MoveToFree extends Command{
    private Arm arm;


    public MoveToFree() {
      arm = Arm.getInstance();
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(arm);
    }

    @Override
    public void initialize() {
      arm.moveIntakeToPose(Constants.ArmConstants.freeMovementPosition);
    }

    @Override
    public boolean isFinished() {
      return arm.isArmInPosition();
    }
  }
}
