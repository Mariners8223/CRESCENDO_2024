// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Test;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.IntakeCommands.IntakeToFloor;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Arm.ArmPostion;

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
      new MoveIntakeNumber(Arm.getInstance().getMainMotorRotation(), 0.36)
      // new MoveIntakeNumber(-0.01, Arm.getInstance().getSecoMotorRotation())
    );
  }

  public static class MoveToFree extends Command {
    private Arm arm;
    private ArmPostion target;


    public MoveToFree() {
      arm = Arm.getInstance();
      // Use addRequirements() here to declare subsystem dependencies.
      target = Constants.ArmConstants.freeMovementPosition.copyArmPostion();
      addRequirements(arm);
    }

    @Override
    public void initialize() {
      target.rotation = arm.getShooterPosition().rotation - Units.rotationsToRadians(arm.getMainMotorRotation()) + Units.rotationsToRadians(0.14);
      arm.moveShooterToPose(target);
    }

    @Override
    public void execute() {
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
    private double alpha;
    private double beta;

    public MoveIntakeNumber(double alpha, double beta) {
      arm = Arm.getInstance();
      this.alpha = alpha;
      this.beta = beta;
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(arm);
    }

    @Override
    public void initialize() {
    }

    @Override
    public void execute() {
      arm.moveIntakeToPose(alpha, beta);
    }

    @Override
    public boolean isFinished() {
      return arm.isArmInPosition();
    }
  }
}


