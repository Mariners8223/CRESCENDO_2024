// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Test.AimToRing;
import frc.robot.commands.Climb.ClimbSequence;
import frc.robot.commands.IntakeCommands.Collect;
import frc.robot.commands.IntakeCommands.Collect_noProxy;
import frc.robot.commands.IntakeCommands.IntakeToFloor;
import frc.robot.commands.IntakeCommands.RollOut;
import frc.robot.commands.ShooterCommands.AimAndShootToAmpArea_Auto;
import frc.robot.commands.ShooterCommands.AimShooter;
import frc.robot.commands.ShooterCommands.QuickAim;
import frc.robot.commands.ShooterCommands.Shoot;
import frc.robot.commands.armCommands.MoveToAlphaPose_close;
import frc.robot.commands.armCommands.MoveToFree;
import frc.robot.commands.armCommands.MoveToHome;
import frc.robot.commands.armCommands.MoveToStow;
import frc.robot.commands.autonomous.ShootNote;
import frc.robot.commands.sequences.AimRegularToSpeaker;
import frc.robot.commands.sequences.ShootToAmp;
import frc.robot.commands.sequences.ShootToAmp.MiniShoot;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.DriveTrain.DriveBase;
import frc.robot.subsystem.VisionSubSystem.Vision;

public class RobotContainer {
  public static DriveBase driveBase;
  public static Arm arm;
  public static Vision vision;

  public static CommandPS5Controller driveController;
  public static CommandPS5Controller armController;

  public static Command aimCommand;
  // public static CommandXboxController tempController;

  // public static SendableChooser<Command> autoChooser;
  public static LoggedDashboardChooser<Command> autoChooser;
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
    armController = new CommandPS5Controller(1);

    driveBase = new DriveBase();
    arm = Arm.getInstance();
    vision = new Vision();

    // arm.setDefaultCommand(new QuikAim());
    configureNamedCommands();
    configureBindings();
    configChooser();

    new Trigger(DriverStation::isDSAttached).onTrue(new InstantCommand(() -> {
      if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) Constants.SwapToRed();}).ignoringDisable(true));

    new Trigger(DriverStation::isDSAttached).onTrue(new InstantCommand(() -> Logger.recordOutput("allince", DriverStation.getAlliance().get().toString())).ignoringDisable(true));
  }

  private void configureBindings() {
    var ringAim = new AimToRing();

    driveController.options().onTrue(new InstantCommand(() -> driveBase.resetOnlyDirection()));
    driveController.touchpad().whileTrue(DriveBase.OrchestraCommand.getInstance());
    // driveController.cross().onTrue(ringAim).onFalse(new InstantCommand(() -> { ringAim.cancel(); driveBase.isControlled = false; }));
    driveController.L1().whileTrue(new AimToRing().onlyIf(() -> Arm.getInstance().lastknownPosition == Arm.knownArmPosition.Intake));
    
    aimCommand = new AimRegularToSpeaker();
    var collect = new Collect_noProxy();

    driveController.square().onTrue(aimCommand);
    driveController.triangle().onTrue(new Shoot());

    armController.cross().onTrue(new IntakeToFloor());
    armController.circle().onTrue(collect).onFalse(new InstantCommand(() -> collect.cancel()));
    armController.square().onTrue(new ShootToAmp());
    armController.triangle().onTrue(new Shoot());
    armController.povLeft().onTrue(new MoveToAlphaPose_close());

    armController.povDown().onTrue(new MoveToHome());
    armController.L1().onTrue(new RollOut());
    armController.R1().onTrue(new MiniShoot());
    armController.povUp().onTrue(new MoveToStow());
  }

  private void configChooser(){
    List<String> namesOfAutos = AutoBuilder.getAllAutoNames();
    List<PathPlannerAuto> autosOfAutos = new ArrayList<>();

    autoChooser = new LoggedDashboardChooser<>("");


    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());

    for (String autoName : namesOfAutos) {
      PathPlannerAuto auto = new PathPlannerAuto(autoName);
        autosOfAutos.add(auto);
    }

    autosOfAutos.forEach(auto -> autoChooser.addOption(auto.getName(), auto));

    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    autoChooser.addOption("Shoot Note", new ShootNote());
    SmartDashboard.putData(autoChooser.getSendableChooser());
  }

  private void configureNamedCommands(){
    // NamedCommands.registerCommand("Do Nothing", new InstantCommand());
    //AUTOSSSSS related shit
    NamedCommands.registerCommand("Shoot", new Shoot());
    NamedCommands.registerCommand("QuikAim", new QuickAim());
    NamedCommands.registerCommand("Collect", new Collect());
    NamedCommands.registerCommand("IntakeToFloor", new IntakeToFloor());
    NamedCommands.registerCommand("MoveToFree", new MoveToFree());
    NamedCommands.registerCommand("MoveToHome", new MoveToHome());
    NamedCommands.registerCommand("ShootToAmp", new ShootToAmp());
    NamedCommands.registerCommand("MoveToAlpha", new MoveToAlphaPose_close());
    NamedCommands.registerCommand("AimToAmpArea", new AimAndShootToAmpArea_Auto());
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
    return autoChooser.get();
  }

   /**
   * gets the zone the robot is in based on the robot's position
   * @return the index of the zone the robot is in (from 1 to 4 inclusive)
   */

  public static int getRobotZone(){
    return Constants.robotZones.indexOf(driveBase.getPose().getTranslation().nearest(Constants.robotZones)) + 1;
  }

}
