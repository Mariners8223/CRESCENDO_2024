// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.ArmUtil;

public class Shoot extends Command {
  /** Creates a new Shoot. */

  private Arm arm;

  public Shoot() {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = Arm.getInstance();

    // addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // if(!Arm.getInstance().getIntakeSub().isGamePieceDetected()){
    //   cancel();
    //   return;
    // }

    // arm.getShooterSub().setShooterPower(0.75);
    // if(ArmUtil.getDx() <= Constants.Arm.EndOfZone1)

    //   arm.getShooterSub().setShooterVelocity(Constants.Shooter.RPMforShooterZone1);
    // else
    //   arm.getShooterSub().setShooterVelocity(Constants.Shooter.RPMforShooterZone2);

    arm.getShooterSub().setShooterVelocity(ArmUtil.getWantedSpeed());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // if(interrupted) return;
    Timer.delay(0.1);

    arm.getIntakeSub().setMotor(1);
    Timer.delay(0.3);

    arm.getIntakeSub().stopMotor();
    arm.getShooterSub().stopMotors();

    //just in case, ends the control of aim
    RobotContainer.driveBase.isControlled = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return arm.getShooterSub().isAtSelctedVelocity();
  }
}
