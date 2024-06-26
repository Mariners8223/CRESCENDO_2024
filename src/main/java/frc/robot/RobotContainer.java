// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.ArrayList;
import java.util.List;
import java.util.function.BooleanSupplier;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.commands.IntakeCommands.IntakeToFloor;
import frc.robot.commands.IntakeCommands.RollOut;
import frc.robot.commands.IntakeCommands.SourceCollect;
import frc.robot.commands.IntakeCommands.Collect.CollectFloor;
import frc.robot.commands.ShooterCommands.AlphaAimToAmpArea;
import frc.robot.commands.ShooterCommands.QuikAim_Auto;
import frc.robot.commands.ShooterCommands.Shoot;
import frc.robot.commands.ShooterCommands.ShooterStarter;
import frc.robot.commands.ShooterCommands.SlowShoot;
import frc.robot.commands.armCommands.MoveToAlphaPose_close;
import frc.robot.commands.armCommands.MoveToFree;
import frc.robot.commands.armCommands.MoveToHome;
import frc.robot.commands.armCommands.MoveToSourceCollection;
import frc.robot.commands.armCommands.MoveToStartShootPose_Auto;
import frc.robot.commands.armCommands.MoveToStow;
import frc.robot.commands.autonomous.AimAndShootToAmpArea_Auto;
import frc.robot.commands.autonomous.AimToRingAuto;
import frc.robot.commands.autonomous.BetaAim_Auto;
import frc.robot.commands.autonomous.HigherAim_Auto;
import frc.robot.commands.autonomous.LowerAim_Auto;
import frc.robot.commands.autonomous.ShootNote;
import frc.robot.commands.autonomous.Shoot_Auto;
import frc.robot.commands.autonomous.ShooterStarter_Auto;
import frc.robot.commands.sequences.AimRegularToSpeaker;
import frc.robot.commands.sequences.AimToRing;
import frc.robot.commands.sequences.ShootToAmp;
import frc.robot.commands.sequences.ShootToAmp.MiniShoot;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.ArmUtil;
import frc.robot.subsystem.Arm.Arm.knownArmPosition;
import frc.robot.subsystem.DriveTrain.DriveBase;
import frc.robot.subsystem.VisionSubSystem.Vision;
import frc.robot.subsystem.VisionSubSystem.Vision.CameraInterface.CameraLocation;

public class RobotContainer {
  public static DriveBase driveBase;
  public static Arm arm;
  public static Vision vision;

  public static CommandPS5Controller driveController;
  public static CommandPS5Controller armController;

  public static Command AlphaAimCommand;

  public static LoggedDashboardChooser<Command> autoChooser;
  public static boolean aimingAtSpeaker = true;
  public static boolean isQuickAiming = false;

  public static BooleanSupplier isBlueAllince = () -> {
    if(DriverStation.getAlliance().isEmpty()) return true;
    return DriverStation.getAlliance().get() == DriverStation.Alliance.Blue;
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

    configureNamedCommands();
    configureBindings();
    configChooser();

    // new Trigger(DriverStation::isDSAttached).onTrue(new InstantCommand(() -> {
    //   if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) Constants.SwapToRed();}).ignoringDisable(true));

    // new Trigger(DriverStation::isDSAttached).onTrue(new InstantCommand(() -> Logger.recordOutput("allince", DriverStation.getAlliance().get().toString())).ignoringDisable(true)); 
  }

  private void configureBindings() {
    // var ringAim = new AimToRing();

    driveController.options().onTrue(new InstantCommand(() -> driveBase.resetOnlyDirection()));
    driveController.touchpad().whileTrue(DriveBase.OrchestraCommand.getInstance());
    driveController.cross().whileTrue(new AimToRing().onlyIf(() -> Arm.getInstance().lastknownPosition == knownArmPosition.Intake && vision.hasTarget(CameraLocation.Front_Arm))); //.onFalse(new InstantCommand(() -> { ringAim.cancel(); driveBase.isControlled = false; new RollOut(); }));
    driveController.circle().whileTrue(new InstantCommand(() -> { driveBase.setIsControlled(true);
    driveBase.setTargetRotation(Rotation2d.fromRadians(ArmUtil.getChassisAngle() - Math.PI), false); })).onFalse(new InstantCommand(() -> driveBase.setIsControlled(false)));
    // driveController.cross().whileTrue(new AimToRingAuto().onlyIf(() -> Arm.getInstance().lastknownPosition == knownArmPosition.Intake && vision.hasTarget(CameraLocation.Front_Arm))); //.onFalse(new InstantCommand(() -> { ringAim.cancel(); driveBase.isControlled = false; new RollOut(); }));

    // driveController.L1().whileTrue(new AimToRing().onlyIf(() -> Arm.getInstance().lastknownPosition == Arm.knownArmPosition.Intake));
    
    AlphaAimCommand = new AimRegularToSpeaker();
    // AlphaAimCommand = new QuikAim_Auto();
    // AlphaAimCommand = new AlphaAimToAmpArea();


    //BetaAimCommand = new QuickAim();
    // var collect = new Collect_noProxy();

    // driveController.square().onTrue(aimCommand);
    // driveController.triangle().onTrue(new Shoot());

    armController.cross().onTrue(new IntakeToFloor()).onTrue(new InstantCommand(() -> AlphaAimCommand.cancel()));
    // armController.circle().onTrue(collect).onFalse(new InstantCommand(() -> collect.cancel()));
    // armController.circle().whileTrue(new Collect_noProxy());
    armController.circle().whileTrue(new CollectFloor());
    armController.povLeft().onTrue(new MoveToSourceCollection());
    armController.square().whileTrue(new SourceCollect());
    // armController.square().onTrue(new ShootToAmp()).onTrue(new InstantCommand(() -> AlphaAimCommand.cancel()));
    armController.triangle().onTrue(new Shoot());
    armController.button(9).whileTrue(new ShooterStarter()); // share button
    armController.options().onTrue(new ShootToAmp());
    //check if still necesery
    // armController.povLeft().onTrue(new MoveToAlphaPose_close()).onTrue(new InstantCommand(() -> AlphaAimCommand.cancel())).onTrue(new InstantCommand(() -> BetaAimCommand.cancel()));

    armController.povRight().onTrue(AlphaAimCommand);
    //armController.povRight().onTrue(BetaAimCommand);

    // armController.touchpad().onTrue(new MoveToAlphaPose_close());
    armController.touchpad().onTrue(new SlowShoot());

    armController.povDown().onTrue(new MoveToHome()).onTrue(new InstantCommand(() -> AlphaAimCommand.cancel()));
    armController.L1().onTrue(new RollOut());
    armController.R1().onTrue(new MiniShoot());
    armController.povUp().onTrue(new MoveToStow()).onTrue(new InstantCommand(() -> AlphaAimCommand.cancel()));
  }

  private void configChooser(){
    List<String> namesOfAutos = AutoBuilder.getAllAutoNames();
    List<PathPlannerAuto> autosOfAutos = new ArrayList<>();

    autoChooser = new LoggedDashboardChooser<>("chooser");
    for (String autoName : namesOfAutos) {
      PathPlannerAuto auto = new PathPlannerAuto(autoName);
        autosOfAutos.add(auto);
    }

    autosOfAutos.forEach(auto -> autoChooser.addOption(auto.getName(), auto));

    autoChooser.addDefaultOption("Do Nothing", new InstantCommand());
    // autoChooser.addOption("Shoot Note", new ShootNote());
    SmartDashboard.putData("chooser", autoChooser.getSendableChooser());

    new Trigger(RobotState::isEnabled).and(RobotState::isTeleop).onTrue(new InstantCommand(() -> driveBase.getField2d().getObject("AutoPath").setPoses()).ignoringDisable(true));
  }


  public static void updateFieldFromAuto(String autoName){
    List<Pose2d> poses = new ArrayList<>();
    boolean DoesExsit = false;
    for (String name : AutoBuilder.getAllAutoNames()) {
      if(name.equals(autoName)) DoesExsit = true;      
    }
    if(!DoesExsit){
      driveBase.getField2d().getObject("AutoPath").setPoses();
      return;
    }
    PathPlannerAuto.getPathGroupFromAutoFile(autoName).forEach(path -> {
      if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red) path = path.flipPath();
      path.getPathPoses().forEach(pose -> poses.add(pose));
    });
    driveBase.getField2d().getObject("AutoPath").setPoses(poses);
  }

  private void configureNamedCommands(){
    // NamedCommands.registerCommand("Do Nothing", new InstantCommand());
    //AUTOSSSSS related shit
    NamedCommands.registerCommand("Shoot", new Shoot_Auto());
    NamedCommands.registerCommand("QuikAim", new SequentialCommandGroup(new BetaAim_Auto(), new WaitCommand(0.3)));
    NamedCommands.registerCommand("Collect", new SequentialCommandGroup(new CollectFloor()));
    NamedCommands.registerCommand("IntakeToFloor", new IntakeToFloor());
    NamedCommands.registerCommand("MoveToFree", new MoveToFree());
    NamedCommands.registerCommand("MoveToHome", new MoveToHome());
    NamedCommands.registerCommand("ShootToAmp", new ShootToAmp());
    NamedCommands.registerCommand("MoveToAlpha", new MoveToAlphaPose_close());
    NamedCommands.registerCommand("AimToAmpArea", new AimAndShootToAmpArea_Auto());
    NamedCommands.registerCommand("StartPosition", new MoveToStartShootPose_Auto());
    NamedCommands.registerCommand("RollOut", new RollOut());
    NamedCommands.registerCommand("AutoCollect", new AimToRingAuto());
    NamedCommands.registerCommand("shooter starter", new ShooterStarter_Auto());
    NamedCommands.registerCommand("Shoot Note", new ShootNote());

    //Aiming positions
    NamedCommands.registerCommand("lower aim", new SequentialCommandGroup(new LowerAim_Auto(), new WaitCommand(0.3)));
    NamedCommands.registerCommand("higher aim", new SequentialCommandGroup(new HigherAim_Auto(), new WaitCommand(0.3)));

    NamedCommands.registerCommand("Start Intake and Shoter motors", new InstantCommand(() ->
     {Arm.getInstance().getShooterSub().setShooterPower(0.5); Arm.getInstance().getIntakeSub().setMotor(0.8);}));
    NamedCommands.registerCommand("Stop Intake and Shoter motors",  new InstantCommand(() ->
     {Arm.getInstance().getShooterSub().stopMotors(); Arm.getInstance().getIntakeSub().stopMotor();}));
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
