// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.Arm;
import frc.robot.commands.ShooterCommands.AmpShootGamePiece;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootToAmp extends SequentialCommandGroup {
  private static Pose2d targetPose = new Pose2d(Constants.Speaker.ampTranslation.getX(), Constants.Speaker.ampTranslation.getY() - 0, Rotation2d.fromDegrees(90)); //TODO add the correct y minus
  /** Creates a new GoToAmp. */
  public ShootToAmp() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new ParallelCommandGroup(
        RobotContainer.driveBase.findPath(targetPose),
        new InstantCommand(() -> RobotContainer.arm.moveShooterToPose(Arm.AmpArmPosition))
      ),
      new AmpShootGamePiece()
    );
  }
}
