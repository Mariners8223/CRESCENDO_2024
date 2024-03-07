// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.ArmUtil;
import frc.robot.subsystem.DriveTrain.DriveBase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BetaAim_AutoInstants extends InstantCommand {
  private static double target;
  private static Arm arm;
   private static DriveBase driveBase;
  public BetaAim_AutoInstants() {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = Arm.getInstance();
    driveBase = RobotContainer.driveBase;

    addRequirements(arm, driveBase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ArmUtil.SetQuikShotMode(true);
      ArmUtil.UpdateParameters();

      // Armtarget = ArmUtil.getArmNeededPosition();
      // Armtarget.rotation = MathUtil.clamp(Armtarget.rotation, Units.rotationsToRadians(0.35), Units.rotationsToRadians(0.5));
      // arm.moveShooterToPose(Armtarget);

      target = MathUtil.clamp(ArmUtil.getArmAngle(), Units.rotationsToRadians(0.35), Units.rotationsToRadians(0.5));
      arm.moveMotorsToRotation(0, Units.radiansToRotations(target));

      // RobotContainer.driveBase.isControlled = true;
      // RobotContainer.driveBase.setIsControlled(true);
      // RobotContainer.driveBase.setTargetRotation(Rotation2d.fromRadians(ArmUtil.getChassisAngle()), false);
      // driveBase.drive(0, 0, 0);

      
      // RobotContainer.driveBase.isControlled = false;
      RobotContainer.driveBase.setIsControlled(false);
  }
}
