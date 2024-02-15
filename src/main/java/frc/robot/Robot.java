// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import com.pathplanner.lib.pathfinding.Pathfinding;

import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.util.LocalADStarAK;

public class Robot extends LoggedRobot {

  @Override
  public void robotInit() {
    Pathfinding.setPathfinder(new LocalADStarAK());

    if(isReal()){
      Logger.addDataReceiver(new NT4Publisher());
      // Logger.addDataReceiver(new WPILOGWriter("/U/Logs"));
    }
    else setUseTiming(false);

    Logger.start();

    new RobotContainer();
  }

  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
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
  public void teleopPeriodic() {}

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
