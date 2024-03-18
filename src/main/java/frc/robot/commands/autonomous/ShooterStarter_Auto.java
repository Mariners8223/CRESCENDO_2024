// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.ArmUtil;
import frc.robot.subsystem.Arm.Shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShooterStarter_Auto extends InstantCommand {
  // public static Trigger trigger = new Trigger(RobotState::isTeleop).onTrue(new InstantCommand(() -> Arm.getInstance().getShooterSub().stopMotors()));
  private static Shooter shooter;
  public ShooterStarter_Auto() {
    // Use addRequirements() here to declare subsystem dependencies.
    shooter = Arm.getInstance().getShooterSub();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    shooter.setShooterRPM(4000);
  }
}
