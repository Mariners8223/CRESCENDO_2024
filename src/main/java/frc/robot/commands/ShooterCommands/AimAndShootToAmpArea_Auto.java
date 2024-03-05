// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.RobotContainer;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.ArmUtil;
import frc.robot.subsystem.Arm.Shooter.Shooter;

public class AimAndShootToAmpArea_Auto extends Command {
  /** Creates a new AimAndShootToAmpArea_Auto. */
  private static Arm arm;
  private static Shooter shooter;
  private static int timer;
  public AimAndShootToAmpArea_Auto() {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = Arm.getInstance();
    shooter = arm.getShooterSub();
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
    ArmUtil.SetQuikShotMode(true);
      ArmUtil.UpdateParameters();
      RobotContainer.driveBase.setTargetRotation(Rotation2d.fromRadians(ArmUtil.getChassisAngle_ToAmp()), false);
      shooter.setShooterVelocity(ArmUtil.getWantedVelocity_ToAmp());
      RobotContainer.driveBase.isControlled = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Timer.delay(0.1);

    arm.getIntakeSub().setMotor(1);
    Timer.delay(0.3);

    arm.getIntakeSub().stopMotor();
    shooter.stopMotors();

    //just in case, ends the control of aim
    RobotContainer.driveBase.isControlled = false;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer >= 50 || RobotContainer.driveBase.getPose().getRotation().getRadians()%(2*Math.PI) == ArmUtil.getChassisAngle_ToAmp()
     && shooter.isAtSelctedVelocity();
  }
}
