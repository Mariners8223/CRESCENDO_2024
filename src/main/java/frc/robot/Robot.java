// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.ArmUtil;
import frc.util.LocalADStarAK;

public class Robot extends LoggedRobot {

  @Override
  public void robotInit() {
    Pathfinding.setPathfinder(new LocalADStarAK());

    if(isReal()){
      Logger.addDataReceiver(new NT4Publisher());
      Logger.addDataReceiver(new WPILOGWriter("/U/logs"));
    }
    // else setUseTiming(false);

    Logger.start();

    new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    ArmUtil.UpdateParameters();
    SmartDashboard.putNumber("Arm angle", 180 - Units.radiansToDegrees(ArmUtil.getArmAngle()));
    // SmartDashboard.putNumber("Robot (chassis) angle", Units.radiansToDegrees(ArmUtil.getChassisAngle()));
    SmartDashboard.putNumber("zone 1 angle", Units.radiansToDegrees(ArmUtil.getZone1()));
    SmartDashboard.putNumber("zone 2 angle", Units.radiansToDegrees(ArmUtil.getZone2()));
    SmartDashboard.putNumber("diff between zone 1 and 2", Units.radiansToDegrees(ArmUtil.getZone2() - ArmUtil.getZone1()));
    // SmartDashboard.putNumber("dx", ArmUtil.getDx());
    // SmartDashboard.putNumber("dy", ArmUtil.getDy());
    // SmartDashboard.putNumber("dz", ArmUtil.getDz());
    SmartDashboard.putBoolean("is in zone 1", ArmUtil.isZone1());
    SmartDashboard.putBoolean("is arm in position", Arm.getInstance().isArmInPosition());
    SmartDashboard.putNumber("main real Angle", 360 * RobotContainer.arm.getMainMotorRotation());
    // SmartDashboard.putNumber("distance to speaker", ArmUtil.get)
    // SmartDashboard.putNumber("arm x", Arm.getInstance().getShooterPosition().x);
    // SmartDashboard.putNumber("angle to ring", RobotContainer.vision.getAngleToObjects(CameraLocation.Front_Arm)[0]);
    // SmartDashboard.putNumber("Chassis angle", RobotContainer.driveBase.getAngle());
    // SmartDashboard.putNumber("ArmPose z", Arm.getInstance().getShooterPosition().y);
    // // SmartDashboard.putNumber("armX", Arm.getInstance().getShooterPosition().x);

    SmartDashboard.putBoolean("is at selected velocity", Arm.getInstance().getShooterSub().isAtSelctedVelocity());
    
    SmartDashboard.putNumber("wanted speed", Units.radiansPerSecondToRotationsPerMinute(ArmUtil.getWantedSpeed() / Constants.Shooter.wheelRadius));
    // // Constants.Shooter.frictionPowerParameterForGPVelocity = SmartDashboard.getNumber("cof", Constants.Shooter.frictionPowerParameterForGPVelocity);
    // SmartDashboard.putNumber("angle to gp", RobotContainer.vision.getAngleToBestObject(CameraLocation.Front_Right));
    // SmartDashboard.putNumber("distance to gp", RobotContainer.vision.getDistanceToBestObject(CameraLocation.Front_Right));


    // Constants.Shooter.GPAirTimeZone1 = SmartDashboard.getNumber("Zone1", Constants.Shooter.GPAirTimeZone1);
    // Constants.Shooter.GPAirTimeZone2 = SmartDashboard.getNumber("Zone2", Constants.Shooter.GPAirTimeZone2);

    SmartDashboard.putBoolean("isAtSelctedVelocity", Arm.getInstance().getShooterSub().isAtSelctedVelocity());
  }

  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {}

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
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

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
