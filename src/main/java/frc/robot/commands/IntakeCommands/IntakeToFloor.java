// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.armCommands.MoveToFree;
import frc.robot.subsystem.Arm.*;
import frc.robot.subsystem.Arm.Arm.IsLastPosition;
// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeToFloor extends ParallelRaceGroup {  
  public IntakeToFloor() {
    addCommands(
      new SequentialCommandGroup(
        new MoveToFree(),
        new MoveIntakeNumber(false),
        new MoveIntakeNumber(true)
      ),
      new IsLastPosition(Arm.knownArmPosition.Intake)//TODO: kick eyal for doing an error
    );
  }

  public static class MoveIntakeNumber extends Command{
    private Arm arm;
    boolean mainMotor;

    public MoveIntakeNumber(boolean mainMotor) {
      arm = Arm.getInstance();
      this.mainMotor = mainMotor;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(arm);
    }

    @Override
    public void initialize() {
      if(mainMotor) arm.moveMotorsToRotation(Constants.Intake.mainIntakeAngle, Arm.getInstance().getSecondaryMotorRotation());
      else arm.moveMotorsToRotation(Arm.getInstance().getMainMotorRotation(), Constants.Intake.secondaryIntakeAngle);
    }

    @Override
    public void end(boolean interrupted){
      System.out.println("Intake ended");
      if(interrupted && Arm.getInstance().lastknownPosition != Arm.knownArmPosition.Intake) Arm.getInstance().lastknownPosition = Arm.knownArmPosition.Unknown;
      else if(mainMotor) Arm.getInstance().lastknownPosition = Arm.knownArmPosition.Intake;
    }

    @Override
    public boolean isFinished() {
      return arm.isArmInPosition();
    }
  }
}
