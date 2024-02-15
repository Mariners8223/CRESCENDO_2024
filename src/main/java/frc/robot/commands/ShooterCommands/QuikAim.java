// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.armCommands.MoveToFree;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.ArmUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class QuikAim extends SequentialCommandGroup {
  /** Creates a new QuikAim. */
  public QuikAim() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveToFree(),
      new InstantCommand(() -> RobotContainer.arm.getShooterSub().setShooterPower(0.8)),
      new RepeatCommand(new QuickAim1())
    );
  }

  public static class QuickAim1 extends InstantCommand{
    private static Arm arm;
    public QuickAim1() {
      // Use addRequirements() here to declare subsystem dependencies.
      arm = Arm.getInstance();
      addRequirements(arm);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      ArmUtil.UpdateParameters();
      arm.moveShooterToPose(ArmUtil.getArmNeededPosition());
      RobotContainer.driveBase.setTargetRotation(Rotation2d.fromDegrees(ArmUtil.getChassisAngle()));
  }
  }
}
