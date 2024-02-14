// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystem.Arm.*;
import frc.robot.subsystem.Arm.Arm.ArmPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IntakeToFloor extends SequentialCommandGroup {  
  public IntakeToFloor() {
    addCommands(
      new MoveToFree(),
      new MoveIntakeNumber(false),
      new MoveIntakeNumber(true)
    );
  }

  public static class MoveToFree extends Command {
    private Arm arm;
    private ArmPosition target;


    public MoveToFree() {
      arm = Arm.getInstance();
      // Use addRequirements() here to declare subsystem dependencies.
      target = Constants.Arm.freeMovementPosition.copyArmPostion();
      addRequirements(arm);
    }

    @Override
    public void initialize() {
      target.rotation = arm.getShooterPosition().rotation - Units.rotationsToRadians(arm.getMainMotorRotation()) + Units.rotationsToRadians(0.14);
      arm.moveShooterToPose(target);
    }

    @Override
    public void end(boolean interrupted){
      System.out.println("command ended");
    }

    @Override
    public boolean isFinished() {
      return arm.isArmInPosition();
    }
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
    public boolean isFinished() {
      return arm.isArmInPosition();
    }
  }
}
