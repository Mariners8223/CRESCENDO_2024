// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.Test;

import com.pathplanner.lib.util.GeometryUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeCommands.IntakeToFloor;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.DriveTrain.DriveBase;
import frc.robot.subsystem.VisionSubSystem.Vision;
import frc.robot.subsystem.VisionSubSystem.Vision.CameraInterface.CameraLocation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimToRing extends SequentialCommandGroup {
  /** Creates a new AimToRing. */
  public AimToRing() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeToFloor().onlyIf(() -> Arm.getInstance().lastknownPosition != Arm.knownArmPosition.Intake),
      new RepeatCommand(new AimToRing1())
    );
  }

  public static class AimToRing1 extends Command {

    DriveBase driveBase;
    CommandPS5Controller controller;
    Vision vision;
    double angleToRing;
    PIDController pidController;

    public AimToRing1() {
      driveBase = RobotContainer.driveBase;
      controller = RobotContainer.driveController;
      vision = RobotContainer.vision;
      pidController = Constants.Vision.aimToRingPID.createPIDController();
    }

    @Override
    public void initialize(){
      driveBase.isControlled = true;
    }

    @Override
    public void execute() {
      angleToRing = vision.getAngleToBestObject(CameraLocation.Front_Right);

      if(angleToRing == -1000) cancel();

      driveBase.setTargetRotation(Rotation2d.fromDegrees(driveBase.getAngle() - angleToRing), true);

      RobotContainer.driveBase.drive(
        -RobotContainer.calculateJoyStickDeadBand(controller.getLeftY()) *
        GeometryUtil.doubleLerp(1, Constants.DriveTrain.Drive.freeWheelSpeedMetersPerSec, 1 - RobotContainer.calculateTriggerDeadBand(controller.getR2Axis())),

        -RobotContainer.calculateJoyStickDeadBand(controller.getLeftX()) *
        GeometryUtil.doubleLerp(1, Constants.DriveTrain.Drive.freeWheelSpeedMetersPerSec, 1 - RobotContainer.calculateTriggerDeadBand(controller.getR2Axis())),

        -RobotContainer.calculateJoyStickDeadBand(controller.getRightX()) *
        GeometryUtil.doubleLerp(1, Constants.DriveTrain.Drive.freeWheelSpeedMetersPerSec, 1 - RobotContainer.calculateTriggerDeadBand(controller.getR2Axis())) 
        );
    }

    @Override
    public void end(boolean interrupt){}

    @Override
    public boolean isFinished() {
      return angleToRing <= Constants.Vision.aimToRingToleranceDegrees;
    }
  }
}