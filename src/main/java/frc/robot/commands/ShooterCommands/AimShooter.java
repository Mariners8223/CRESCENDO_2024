// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.ArmUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimShooter extends InstantCommand {
  private static Arm arm;
  private static double target;
  
  public AimShooter() {
    // Use addRequirements() here to declare subsystem dependencies.
    arm = Arm.getInstance();
    addRequirements(arm);
  }
  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    ArmUtil.SetQuikShotMode(false);
    ArmUtil.setIsAmpShot(false);
    // ArmUtil.UpdateParameters();


    // arm.moveShooterToPose(ArmUtil.getArmNeededPosition());
    target = MathUtil.clamp(ArmUtil.getArmAngle(), Units.degreesToRadians(20), Units.degreesToRadians(80));
    arm.moveMotorsToRotation(Units.radiansToRotations(target) - Constants.Arm.Motors.secondarySoftLimits[1], Constants.Arm.Motors.secondarySoftLimits[1]);

    // arm.getShooterSub().setShooterVelocity(ArmUtil.getWantedSpeed());

    // if(RobotContainer.driveController.circle().getAsBoolean()){
    //   RobotContainer.driveBase.setIsControlled(true);
    //   RobotContainer.driveBase.setTargetRotation(Rotation2d.fromRadians(ArmUtil.getChassisAngle() - Math.PI), false);
    // }
    // else RobotContainer.driveBase.setIsControlled(false);
  }
}
