// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeCommands.Collect;
import frc.robot.commands.IntakeCommands.IntakeToFloor;
import frc.robot.commands.ShooterCommands.QuikAim;
import frc.robot.commands.ShooterCommands.Shoot;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.DriveTrain.DriveBase;
import frc.robot.subsystem.VisionSubSystem.Vision;

public class RobotContainer {
  public static DriveBase driveBase;
  public static Arm arm;
  public static Vision vision;

  public static CommandPS5Controller driveController;
  public static SendableChooser<Command> autoChooser;
  
  public static boolean aimingAtSpeaker = true;
  public static boolean isQuickAiming = false;

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

    // arm.setDefaultCommand(new QuikAim());

    configureBindings();
    configChooser();
    configureNamedCommands();
    autoChooser = new SendableChooser<Command>();

    new Trigger(DriverStation::isDSAttached).onTrue(new InstantCommand(() -> {
      if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) Constants.SwapToRed();}).ignoringDisable(true));

    new Trigger(DriverStation::isDSAttached).onTrue(new InstantCommand(() -> Logger.recordOutput("allince", DriverStation.getAlliance().get().toString())).ignoringDisable(true));
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
    var QuickAim = new QuikAim();

    driveController.cross().onTrue(QuickAim);
    // driveController.cross().onTrue(new InstantCommand(() -> isQuickAiming = !isQuickAiming));
    driveController.square().onTrue(new IntakeToFloor());
    driveController.circle().onTrue(new Collect());
    driveController.triangle().onTrue(new Shoot()).onFalse(new InstantCommand(() -> {QuickAim.cancel(); driveBase.isControlled = false;}));

    driveController.touchpad().whileTrue(DriveBase.OrchestraCommand.getInstance());

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
