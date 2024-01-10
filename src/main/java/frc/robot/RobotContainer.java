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
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.subsystem.DriveTrain.DriveBase;
import frc.util.humanIO.CommandPS5Controller;

public class RobotContainer {
  public static DriveBase driveBase;

  public static CommandPS5Controller driveController;
  public static SendableChooser<Command> autoChooser;

  public static DriverStation.Alliance currentAllince;
  public static boolean allianceSet;

  public RobotContainer() {
    driveController = new CommandPS5Controller(0);

    driveBase = new DriveBase();

    configureBindings();
    configureNamedCommands();
    configChooser();

    //sets the deafult allince as blue
    currentAllince = Alliance.Blue;
    //sets the allince to false
    allianceSet = false;
    //creats a bool supplier for if the allince is not set
    BooleanSupplier isAllinceNotSet = () -> !allianceSet;

    //creates a trigger that will set the allince if the driver station is connected
    new Trigger(DriverStation::isDSAttached).and(isAllinceNotSet).and(DriverStation.getAlliance()::isPresent).onTrue(new InstantCommand(() -> {
      currentAllince = DriverStation.getAlliance().get();
      allianceSet = true;
      Logger.recordOutput("Alliance Set to " + currentAllince.toString());
    }).ignoringDisable(true));
  }

  /**
   * returns if the allince was set (drivers station was connected)
   * @return
   */
  public static boolean isAllinceSet(){
    return allianceSet;
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

}
