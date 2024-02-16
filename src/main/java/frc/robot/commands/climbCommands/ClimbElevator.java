// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.climbCommands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.climb.Elavator;

public class ClimbElevator extends Command {
  /** Creates a new ClimbElevator. */
  Elavator elavator;
  boolean goingUp;

  public ClimbElevator(boolean goingup) {
    elavator = Arm.getInstance().getElavatorSub();
    this.goingUp = goingup;

    addRequirements(Arm.getInstance());
  }

  @Override
  public void initialize(){
    if(goingUp) elavator.setRailMotor(Constants.Elevator.chainHeight - (Constants.Arm.armHeightFromFrameMeters + Constants.DriveTrain.Global.RobotHeightFromGround) * 100 + 8);
    else elavator.setRailMotor(Constants.Elevator.chainHeight - (Constants.Arm.armHeightFromFrameMeters + Constants.DriveTrain.Global.RobotHeightFromGround) * 100 - 15);
  }

  @Override 
  public void end(boolean interrupted){
    System.out.println("Climb 2 ended");
  }

  @Override
  public boolean isFinished(){
    return elavator.isRailMotorInPosition();
  }
}
