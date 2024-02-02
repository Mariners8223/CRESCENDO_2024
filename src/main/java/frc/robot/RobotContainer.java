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
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.DriveTrain.DriveBase;
import frc.util.humanIO.CommandPS5Controller;

public class RobotContainer {
  public static DriveBase driveBase;
  public static Arm arm;

  public static CommandPS5Controller driveController;
  public static SendableChooser<Command> autoChooser;

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

    configureBindings();
    configChooser();
    configureNamedCommands();

    new Trigger(DriverStation::isDSAttached).onTrue(new InstantCommand(() -> Logger.recordOutput("allince", DriverStation.getAlliance().get().toString())));
  }

  private void configureBindings() {
    driveController.circle().onTrue(DriveBase.lockSwerveInXPatternCommand.getInstance());
    driveController.square().onTrue(new InstantCommand(() -> DriveBase.lockSwerveInXPatternCommand.getInstance().cancel()));
    driveController.options().onTrue(new InstantCommand(() -> driveBase.resetOnlyDirection()));
    driveController.triangle().onTrue(new InstantCommand(() -> driveBase.Reset()));

    driveController.cross().whileTrue(DriveBase.OrchestraCommand.getInstance().ignoringDisable(true));
    driveController.share().onTrue(DriveBase.CalibrateSwerveCommand.getInstance().ignoringDisable(true));

  }

  private void configChooser(){
    autoChooser = AutoBuilder.buildAutoChooser();
    autoChooser.setDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("auto path", new InstantCommand(() -> driveBase.followPath(PathPlannerPath.fromPathFile("Dedi4")).schedule()));
    SmartDashboard.putData(autoChooser);
  }

  public static class IntakeToFloor extends InstantCommand{
    
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

  public static Command getAutoCommand(){
    return autoChooser.getSelected();
  }

   /**
   * gets the zone the robot is in based on the x position
   * @return the zone the robot is in (from 1 to 6 inclusive, 6 is the default value if the robot is not in any zone)
   */
  public static int getRobotZone(){
    if(driveBase.getPose().getTranslation().getX() >= Constants.robotZones[0][0].getX() &&
    driveBase.getPose().getTranslation().getX() <= Constants.robotZones[0][1].getX()) return 1;

    if(driveBase.getPose().getTranslation().getX() >= Constants.robotZones[1][0].getX() &&
    driveBase.getPose().getTranslation().getX() <= Constants.robotZones[1][1].getX()) return 2;

    if(driveBase.getPose().getTranslation().getX() >= Constants.robotZones[2][0].getX() &&
    driveBase.getPose().getTranslation().getX() <= Constants.robotZones[2][1].getX()) return 3;

    if(driveBase.getPose().getTranslation().getX() >= Constants.robotZones[3][0].getX() &&
    driveBase.getPose().getTranslation().getX() <= Constants.robotZones[3][1].getX()) return 4;

    if(driveBase.getPose().getTranslation().getX() >= Constants.robotZones[4][0].getX() &&
    driveBase.getPose().getTranslation().getX() <= Constants.robotZones[4][1].getX()) return 5;

    return 6;
  }

}
