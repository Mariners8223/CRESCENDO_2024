// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Arm.ArmPosition;

public class MoveToHome extends SequentialCommandGroup {

  public MoveToHome(){
    addCommands(
      new MoveToHome1(),
      new MoveToHome2()
    );
  }

  private static class MoveToHome1 extends Command{
    Arm arm;
    private static ArmPosition target = Constants.Arm.freeMovementPosition.copyArmPostion();
    /** Creates a new MoveToHome. */
    public MoveToHome1() {
      arm = Arm.getInstance();
      // Use addRequirements() here to declare subsystem dependencies.
      addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      target.rotation = arm.getShooterPosition().rotation - Units.rotationsToRadians(arm.getMainMotorRotation()) + Units.rotationsToRadians(0.14);
      arm.moveShooterToPose(target);
    }

    @Override
    public void end(boolean interrupted){
      System.out.println("finished");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return arm.isArmInPosition();
    }
  }

  private static class MoveToHome2 extends Command{
    Arm arm;
    public MoveToHome2(){
      arm = Arm.getInstance();

      addRequirements(arm);
    }

    @Override
    public void initialize(){
      arm.moveMotorsToRotation(arm.getMainMotorRotation(), 0.09);
    }

    @Override
    public boolean isFinished(){
      return arm.isArmInPosition();
    }
  }
}
