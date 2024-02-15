// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.Climb.ClimbSequence;
import frc.robot.commands.ShooterCommands.QuikAim;
import frc.robot.commands.armCommands.MoveToFree;
import frc.robot.commands.armCommands.MoveToHome;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.DriveTrain.DriveBase;
import frc.robot.subsystem.VisionSubSystem.Vision;
import frc.robot.subsystem.VisionSubSystem.Vision.CameraInterface.CameraLocation;

public class RobotContainer {
  public static DriveBase driveBase;
  public static Arm arm;
  public static Vision vision;

  public static CommandPS5Controller driveController;
  public static SendableChooser<Command> autoChooser;
  
  public static boolean aimingAtSpeaker = true;

  public static BooleanSupplier isBlueAllince = () -> {
    if(DriverStation.getAlliance().isEmpty()) return true;
    return DriverStation.getAlliance().get().equals(DriverStation.Alliance.Blue);
  };

  public static BooleanSupplier isRedAllince = () -> {
    return !isBlueAllince.getAsBoolean();
  };

  public RobotContainer() {
    driveController = new CommandPS5Controller(0);

    driveBase = new DriveBase();
    arm = Arm.getInstance();
    vision = new Vision();

    configureBindings();
    configChooser();
    configureNamedCommands();
    autoChooser = new SendableChooser<Command>();

    new Trigger(DriverStation::isDSAttached).onTrue(new InstantCommand(() -> Logger.recordOutput("allince", DriverStation.getAlliance().get().toString())));
  }

  private void configureBindings() {
    driveController.options().onTrue(new InstantCommand(() -> driveBase.resetOnlyDirection()));

    SmartDashboard.putBoolean("Main Motor", true);
    SmartDashboard.putBoolean("Quasistatic", true);

    // driveController.square().onTrue(new IntakeToFloor());
    // driveController.triangle().whileTrue(new TestShoot()).onFalse(new MoveToHome());

    // driveController.cross().onTrue(new Collect());
    // driveController.circle().onTrue(new Shoot());
    // driveController.cross().onTrue(new MoveToFree());
    // driveController.square().onTrue(new MoveToHome());
    // driveController.cross().onTrue(new InstantCommand(() -> Arm.getInstance().getShooterSub().setShooterPower(0.5))).onFalse(new InstantCommand(() -> Arm.getInstance().getShooterSub().stopMotors()));
    driveController.cross().whileTrue(new QuikAim());

    // driveController.cross().onTrue(new ClimbSequence());
    // driveController.cross().onTrue(new InstantCommand(() -> vision.setPipelineIndex(CameraLocation.Back, 1)));
    // driveController.square().onTrue(new InstantCommand(() -> vision.setPipelineIndex(CameraLocation.Back, 0)));
  }

  private void configChooser(){
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("auto path", new InstantCommand(() -> driveBase.followPath(PathPlannerPath.fromPathFile("Dedi4")).schedule()));
    SmartDashboard.putData(autoChooser);
  }

  private void configureNamedCommands(){
    NamedCommands.registerCommand("Do Nothing", new InstantCommand());
  }

  public static double calculateJoyStickDeadBand(double value){
    if(Math.abs(value) <= Constants.Controllers.joyStickDeadBand) return 0;
    return value;
  }

  public static double calculateTriggerDeadBand(double value){
    if(value < Constants.Controllers.triggerDeadBand) return 0;
    return value;
  }

  public static boolean isAmplified(){
    return false;
  }

  public static boolean isRobotSpeakerMode(){
    return aimingAtSpeaker && arm.getIntakeSub().isGamePieceDetected();
  }

  public static boolean isRobotAmpMode(){
    return !aimingAtSpeaker && arm.getIntakeSub().isGamePieceDetected();
  }

  public static boolean isAimingAtSpeaker(){
    return aimingAtSpeaker;
  }

  public static Command getAutoCommand(){
    return autoChooser.getSelected();
  }

   /**
   * gets the zone the robot is in based on the robot's position
   * @return the index of the zone the robot is in (from 1 to 4 inclusive)
   */

  public static int getRobotZone(){
    return Constants.robotZones.indexOf(driveBase.getPose().getTranslation().nearest(Constants.robotZones)) + 1;
  }

}
