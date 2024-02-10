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
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Test.SmallIntake;
import frc.robot.commands.IntakeCommands.Collect;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Arm.SysIDArm;
import frc.robot.subsystem.DriveTrain.DriveBase;

public class RobotContainer {
  public static DriveBase driveBase;
  public static Arm arm;

  public static Arm.SysIDArm sysIDArm;

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

    sysIDArm = arm.new SysIDArm();

    configureBindings();
    configChooser();
    configureNamedCommands();

    new Trigger(DriverStation::isDSAttached).onTrue(new InstantCommand(() -> Logger.recordOutput("allince", DriverStation.getAlliance().get().toString())));
  }

  private void configureBindings() {
    driveController.options().onTrue(new InstantCommand(() -> driveBase.resetOnlyDirection()));

    // driveController.cross().whileTrue(DriveBase.OrchestraCommand.getInstance().ignoringDisable(true));
    // driveController.share().onTrue(DriveBase.CalibrateSwerveCommand.getInstance().ignoringDisable(true));


    // driveController.cross().debounce(0.1).and(() -> getRobotZone() == 1 || getRobotZone() == 2).onTrue(new InstantCommand(() -> aimingAtSpeaker = !aimingAtSpeaker));
    // new Trigger(() -> getRobotZone() == 1 || getRobotZone() == 2).onTrue(new InstantCommand(() -> aimingAtSpeaker = true));

    SmartDashboard.putBoolean("Main Motor", true);
    SmartDashboard.putBoolean("Quasistatic", true);

    driveController.cross().onTrue(new SmallIntake());

    // driveController.circle().whileTrue(new Collect());

    // driveController.square().and(isMainMotor).and(isQuasistatic).whileTrue(sysIDArm.quasistaticMain(SysIdRoutine.Direction.kForward));
    // driveController.circle().and(isMainMotor).and(isQuasistatic).whileTrue(sysIDArm.quasistaticMain(SysIdRoutine.Direction.kReverse));

    // driveController.square().and(isMainMotor).and(isDynamic).whileTrue(sysIDArm.dynamicMain(SysIdRoutine.Direction.kForward));
    // driveController.circle().and(isMainMotor).and(isDynamic).whileTrue(sysIDArm.dynamicMain(SysIdRoutine.Direction.kReverse));


    // driveController.square().and(isQuasistatic).whileTrue(sysIDArm.quasistaticSecondary(SysIdRoutine.Direction.kForward));
    // driveController.circle().and(isQuasistatic).whileTrue(sysIDArm.quasistaticSecondary(SysIdRoutine.Direction.kReverse));

    // driveController.square().and(isDynamic).whileTrue(sysIDArm.dynamicSecondary(SysIdRoutine.Direction.kForward));
    // driveController.circle().and(isDynamic).whileTrue(sysIDArm.dynamicSecondary(SysIdRoutine.Direction.kReverse));
  }

  // private boolean isMainMotor(){
  //   return SmartDashboard.getBoolean("mainMotor", true);
  // }
  private BooleanSupplier isMainMotor = () -> { return SmartDashboard.getBoolean("Main Motor", true);};
  private BooleanSupplier isQuasistatic = () -> {return SmartDashboard.getBoolean("Quasistatic", true);};
  
  private BooleanSupplier isSecondaryMotor = () -> { return !SmartDashboard.getBoolean("Main Motor", true);};
  private BooleanSupplier isDynamic = () -> {return !SmartDashboard.getBoolean("Quasistatic", true);};

  // private boolean isQuas(){
  //   return SmartDashboard.getBoolean("quas", true);
  // }

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
