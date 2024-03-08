// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Test;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.commands.armCommands.MoveToHome;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Arm.ArmPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TestShoot extends SequentialCommandGroup {
  /** Creates a new TestShoot. */
  public TestShoot() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveToHome(),
      new RepeatCommand(new AimShooter())
    );
  }

  public static class AimShooter extends InstantCommand{
    Arm arm;
    static ArmPosition target;

    public AimShooter(){
      arm = Arm.getInstance();
      target = Constants.Arm.freeMovementPosition.copyArmPostion();
      addRequirements(arm);
    }

    @Override
    public void initialize(){
      target.rotation = Units.degreesToRadians(MathUtil.clamp(arm.getAngleToSpeaker(), Constants.Zone1.minAngle, Constants.Zone1.maxAngle));
      arm.moveShooterToPose(target);
    }
  }
}
