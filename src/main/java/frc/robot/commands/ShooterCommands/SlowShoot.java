// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Shooter.Shooter;

public class SlowShoot extends SequentialCommandGroup {
  /** Creates a new SlowShoot. */
  public SlowShoot(){
    addCommands(
    new SlowShoot1(),
    new WaitCommand(0.6),
    new InstantCommand(() -> {
      Arm.getInstance().getIntakeSub().stopMotor();
      Arm.getInstance().getShooterSub().stopMotors();

      //just in case, ends the control of aim
      // RobotContainer.driveBase.isControlled = false;
      RobotContainer.AlphaAimCommand.cancel();
      
      RobotContainer.driveBase.setIsControlled(false);
      Arm.getInstance().getIntakeSub().setIsGamePieceDetected(false);
      // RobotContainer.BetaAimCommand.cancel();
    }));
  }
  
  public class SlowShoot1 extends Command{
  
  private static Shooter shooter;
  private static int timer;

  public SlowShoot1() {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = Arm.getInstance().getShooterSub();
    timer = 0;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooterRPM(4000);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Arm.getInstance().getIntakeSub().setMotor(1);
    Arm.getInstance().getIntakeSub().setIsGamePieceDetected(false);
    if(interrupted){
      Arm.getInstance().getShooterSub().stopMotors();
      Arm.getInstance().getIntakeSub().stopMotor();
    }
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (shooter.isAtSelctedVelocity() && timer > 60) || timer > 120 || (shooter.isMotorsAtSameSpeed() && timer > 90);
  }
  }
}
