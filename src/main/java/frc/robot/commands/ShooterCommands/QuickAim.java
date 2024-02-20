// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeCommands.IntakeToFloor;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.ArmUtil;
import frc.robot.subsystem.Arm.Arm.ArmPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class QuickAim extends SequentialCommandGroup {
  /** Creates a new QuikAim. */
  public QuickAim() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeToFloor().onlyIf(() -> Arm.getInstance().lastknownPosition != Arm.knownArmPosition.Intake),
      // new InstantCommand(() -> RobotContainer.arm.getShooterSub().setShooterPower(0.6)),
      new RepeatCommand(new QuickAim1())
    );
  }

  public static class QuickAim1 extends InstantCommand{
    private static Arm arm;
    private static ArmPosition target;
    public QuickAim1() {
      // Use addRequirements() here to declare subsystem dependencies.
      arm = Arm.getInstance();
      target = new ArmPosition();
      addRequirements(arm);
    }
    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
      ArmUtil.SetQuikShotMode(true);
      ArmUtil.UpdateParameters();

      target = ArmUtil.getArmNeededPosition();
      target.rotation = MathUtil.clamp(target.rotation, Units.rotationsToRadians(0.35), Units.rotationsToRadians(0.5));
      arm.moveShooterToPose(target);
      
      RobotContainer.driveBase.setTargetRotation(Rotation2d.fromRadians(ArmUtil.getChassisAngle()), false);
  }
  }
}
