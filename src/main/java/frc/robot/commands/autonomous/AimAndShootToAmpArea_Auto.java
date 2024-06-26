// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.ArmUtil;
import frc.robot.subsystem.Arm.Shooter.Shooter;

public class AimAndShootToAmpArea_Auto extends SequentialCommandGroup {

  public AimAndShootToAmpArea_Auto() {
    addCommands(
      new AimAndShootToAmpArea_Auto1(),
      new WaitCommand(0.1),
      new InstantCommand(() -> Arm.getInstance().getIntakeSub().setMotor(1)),
      new WaitCommand(0.3),
      new InstantCommand(() -> {
        Arm.getInstance().getIntakeSub().stopMotor();
        Arm.getInstance().getShooterSub().stopMotors();
        //just in case, ends the control of aim
        // RobotContainer.driveBase.isControlled = false;
        RobotContainer.driveBase.setIsControlled(false);
      }) 
    );
  }
  private static class AimAndShootToAmpArea_Auto1 extends Command {
  /** Creates a new AimAndShootToAmpArea_Auto. */
  private static Arm arm;
  private static Shooter shooter;
  private static int timer;
  public AimAndShootToAmpArea_Auto1() {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = Arm.getInstance();
    shooter = arm.getShooterSub();
    addRequirements(arm);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    timer = 0;
    ArmUtil.setIsBetaShoot_amp(true);
    ArmUtil.setIsAmpShot(true);
    ArmUtil.UpdateParameters();
    arm.moveMotorsToRotation(0, MathUtil.clamp(ArmUtil.getArmAngle_ToAMP(), Units.degreesToRadians(115), Units.degreesToRadians(180)));
    // RobotContainer.driveBase.setTargetRotation(Rotation2d.fromRadians(ArmUtil.getChassisAngle_ToAmp()), false);
    shooter.setShooterVelocity(ArmUtil.getWantedVelocity_ToAmp());
    // RobotContainer.driveBase.setIsControlled(false);
    // RobotContainer.driveBase.isControlled = false;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    timer++;
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // arm.getShooterSub().stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return timer >= 50 || RobotContainer.driveBase.getPose().getRotation().getRadians()%(2*Math.PI) == ArmUtil.getChassisAngle_ToAmp()
     && shooter.isAtSelctedVelocity();
  }
}
}
