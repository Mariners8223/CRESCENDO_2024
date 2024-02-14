// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armCommands;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.Constants;
import frc.robot.subsystem.Arm.Arm.ArmPosition;

public class MoveToFree extends Command {
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
