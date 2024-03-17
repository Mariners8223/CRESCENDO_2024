// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.ArmUtil;
import frc.util.LocalADStarAK;

public class Robot extends LoggedRobot {
  @AutoLog
  public static class pathPLannerInputs{
    Pose2d targetPose = new Pose2d();
    // Pose2d[] path;
    // List<Pose2d> path = new ArrayList<>();
    Pose2d[] path = new Pose2d[0];
  }

  String lastAutoName = "InstantCommand";
  Boolean driverStationWasConnected = false;
  pathPLannerInputsAutoLogged pathPlannerInputs;


  @Override
  public void robotInit() {
    Pathfinding.setPathfinder(new LocalADStarAK());

    ArmUtil.StartArmUtil();

    if(isReal()){
      Logger.addDataReceiver(new NT4Publisher());
      Logger.addDataReceiver(new WPILOGWriter("/U/logs"));
    }
    // else setUseTiming(false);
    // Logger.addDataReceiver(new NT4Publisher());

    Logger.start();

    new RobotContainer();

    pathPlannerInputs = new pathPLannerInputsAutoLogged();

    PathPlannerLogging.setLogTargetPoseCallback((pose) -> pathPlannerInputs.targetPose = pose);
    PathPlannerLogging.setLogActivePathCallback((posees) -> pathPlannerInputs.path = posees.toArray(new Pose2d[0]));

    Logger.processInputs("pathPlanner", pathPlannerInputs);
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    ArmUtil.UpdateParameters();
    // SmartDashboard.putNumber("Arm angle", 180 - Units.radiansToDegrees(ArmUtil.getArmAngle()));
    // SmartDashboard.putNumber("Robot (chassis) angle", Units.radiansToDegrees(ArmUtil.getChassisAngle()));
    // SmartDashboard.putNumber("zone 1 angle", Units.radiansToDegrees(ArmUtil.getZone1()));
    // SmartDashboard.putNumber("zone 2 angle", Units.radiansToDegrees(ArmUtil.getZone2()));
    // SmartDashboard.putNumber("diff between zone 1 and 2", Units.radiansToDegrees(ArmUtil.getZone2() - ArmUtil.getZone1()));
    // SmartDashboard.putNumber("dx", ArmUtil.getDx());
    // SmartDashboard.putNumber("dy", ArmUtil.getDy());
    // SmartDashboard.putNumber("dz", ArmUtil.getDz());
    // SmartDashboard.putBoolean("is in zone 1", ArmUtil.isZone1());
    // SmartDashboard.putBoolean("is arm in position", Arm.getInstance().isArmInPosition());
    // SmartDashboard.putNumber("main real Angle", 360 * RobotContainer.arm.getMainMotorRotation());
    // SmartDashboard.putNumber("distance to speaker", ArmUtil.get)
    // SmartDashboard.putNumber("arm x", Arm.getInstance().getShooterPosition().x);
    // SmartDashboard.putNumber("angle to ring", RobotContainer.vision.getAngleToObjects(CameraLocation.Front_Arm)[0]);
    // SmartDashboard.putNumber("Chassis angle", RobotContainer.driveBase.getAngle());
    // SmartDashboard.putNumber("ArmPose z", Arm.getInstance().getShooterPosition().y);
    // // SmartDashboard.putNumber("armX", Arm.getInstance().getShooterPosition().x);

    // SmartDashboard.putBoolean("is at selected velocity", Arm.getInstance().getShooterSub().isAtSelctedVelocity());
    
    // SmartDashboard.putNumber("wanted speed", Units.radiansPerSecondToRotationsPerMinute(ArmUtil.getWantedSpeed() / Constants.Shooter.wheelRadius));
    // // Constants.Shooter.frictionPowerParameterForGPVelocity = SmartDashboard.getNumber("cof", Constants.Shooter.frictionPowerParameterForGPVelocity);
    // SmartDashboard.putNumber("angle to gp", RobotContainer.vision.getAngleToBestObject(CameraLocation.Front_Right));
    // SmartDashboard.putNumber("distance to gp", RobotContainer.vision.getDistanceToBestObject(CameraLocation.Front_Right));


    // Constants.Shooter.GPAirTimeZone1 = SmartDashboard.getNumber("Zone1", Constants.Shooter.GPAirTimeZone1);
    // Constants.Shooter.GPAirTimeZone2 = SmartDashboard.getNumber("Zone2", Constants.Shooter.GPAirTimeZone2);

    // SmartDashboard.putBoolean("isAtSelctedVelocity", Arm.getInstance().getShooterSub().isAtSelctedVelocity());
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
    if(RobotContainer.getAutoCommand() != null && RobotContainer.getAutoCommand().getName() != lastAutoName){
      lastAutoName = RobotContainer.getAutoCommand().getName();
      RobotContainer.updateFieldFromAuto(lastAutoName);
    }

    if(DriverStation.isDSAttached() && !driverStationWasConnected && DriverStation.getAlliance().isPresent()){
      driverStationWasConnected = true;
      Logger.recordOutput("Allince", DriverStation.getAlliance().get().name());
      if(DriverStation.getAlliance().get() == Alliance.Red){
        Constants.SwapToRed();
      }
    }
  }

  @Override
  public void disabledExit() {
  }

  @Override
  public void autonomousInit() {
    if (RobotContainer.getAutoCommand() != null) {
      RobotContainer.getAutoCommand().schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {
    Logger.processInputs("pathPlanner", pathPlannerInputs);
  }

  @Override
  public void autonomousExit() {
    Arm.getInstance().getShooterSub().stopMotors();
  }

  @Override
  public void teleopInit() {
    if (RobotContainer.getAutoCommand() != null) {
      RobotContainer.getAutoCommand().cancel();
    }
  }

  @Override
  public void teleopPeriodic() {
  }

  @Override
  public void teleopExit() {
  }

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
