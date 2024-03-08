// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.ArmUtil;

public class Shoot extends SequentialCommandGroup {
  public Shoot(){
    addCommands(
    new Shoot1(),
    new WaitCommand(0.6),
    new InstantCommand(() -> {
      Arm.getInstance().getIntakeSub().stopMotor();
      Arm.getInstance().getShooterSub().stopMotors();

      //just in case, ends the control of aim
      // RobotContainer.driveBase.isControlled = false;
      RobotContainer.driveBase.setIsControlled(false);

      RobotContainer.AlphaAimCommand.cancel();
      RobotContainer.BetaAimCommand.cancel();
    }));
  }
  /** Creates a new Shoot. */

  private static class Shoot1 extends Command{
  private Arm arm;
  private int timer;

  public Shoot1() {
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

    // arm.getShooterSub().setShooterRPM(4000);;
    if (DriverStation.isAutonomous() && ArmUtil.isZone1()) {
      arm.getShooterSub().setShooterRPM(4000);
    }
    else arm.getShooterSub().setShooterVelocity(ArmUtil.getWantedSpeed());
    timer = 0;
    // if(ArmUtil.getDx() <= Constants.Arm.EndOfZone1)

    //   arm.getShooterSub().setShooterVelocity(Constants.Shooter.RPMforShooterZone1);
    // else
    //   arm.getShooterSub().setShooterVelocity(Constants.Shooter.RPMforShooterZone2);

    // arm.getShooterSub().setShooterVelocity(ArmUtil.getWantedSpeed());
  }

  @Override
  public void execute() {
    timer++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // Timer.delay(0.1);
    // if(interrupted) return;
    // Timer.delay(0.1);

    arm.getIntakeSub().setMotor(1);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (arm.getShooterSub().isAtSelctedVelocity() && timer > 60) || timer > 120;
    // return timer >= 20;
  }
}
}
