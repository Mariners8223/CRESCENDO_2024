// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Arm.knownArmPosition;

public class MoveToSourceCollection extends SequentialCommandGroup {
  public MoveToSourceCollection(){
    addCommands(new MoveToSourceCollection_main(), new MoveToSourceCollection_secondary());
  }

  private class MoveToSourceCollection_main extends Command {
    /** Creates a new MoveToSourceCollection. */
    private static Arm arm;
    public MoveToSourceCollection_main() {
      // Use addRequirements() here to declare subsystem dependencies.
      arm = Arm.getInstance();
      addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      arm.moveMotorsToRotation(0.1260986328125, arm.getSecondaryMotorRotation());
    }
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return arm.isArmInPosition();
    }
  }
  private class MoveToSourceCollection_secondary extends Command {
    /** Creates a new MoveToSourceCollection. */
    private static Arm arm;
    public MoveToSourceCollection_secondary() {
      // Use addRequirements() here to declare subsystem dependencies.
      arm = Arm.getInstance();
      addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      arm.moveMotorsToRotation(arm.getMainMotorRotation(), 0.0401611328125);
    }
    
    @Override
    public void end(boolean interapted){
      arm.lastknownPosition = knownArmPosition.Source_Intake;
    }
    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return arm.isArmInPosition();
    }
  }
}

