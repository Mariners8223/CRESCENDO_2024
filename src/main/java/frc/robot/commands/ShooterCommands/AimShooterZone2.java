// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Arm.ArmPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimShooterZone2 extends InstantCommand {
  private static ArmPosition target = new ArmPosition(0.4, 0.4, 0);
  private static double StartSpeed;
  private static double Dy;//y axis to calc the distance from speaker
  private static double distanceToSpeaker;
  private static double angle = 45;
  public static boolean IsDeadZone;

  public AimShooterZone2() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Arm.getInstance());
  }

  public static double Equation(){
    try {
      return (Math.pow(StartSpeed, 2) - Math.pow(StartSpeed, 2)
     * Math.sqrt(1 - (2 * Dy*9.8*Math.pow(StartSpeed, 2) + Math.pow(9.8*distanceToSpeaker, 2))/Math.pow(StartSpeed, 4)))/(9.8*distanceToSpeaker);
    } catch (Exception e) {
      // TODO: handle exception
      return angle;
    }  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotContainer.driveBase.getPose().getTranslation().getY() < Constants.Speaker.SpeakerIsCenterRatioBottomLocation) {
      IsDeadZone = true;
          Dy = Constants.Speaker.SpeakerBottomLocationY
     + Constants.Speaker.SpeakerIsCenterRatioReverse * Constants.Speaker.SpeakerIsCenterRatioBottomLocation;
    }
    else{
      IsDeadZone = false;
          Dy = Constants.Speaker.SpeakerBottomLocationY
     + Constants.Speaker.SpeakerIsCenterRatioReverse * RobotContainer.driveBase.getPose().getTranslation().getY();
    }
    
    distanceToSpeaker = Math.sqrt(Math.pow(RobotContainer.driveBase.getPose().getTranslation().getX() - Constants.Speaker.SpeakerTranslation.getX(), 2) + Math.pow(Dy, 2));

    StartSpeed = Arm.getInstance().getShooterSub().getTrueXAxisVelocity_RobotRelative();

    angle = Equation();
    target.rotation = Math.atan(angle);
    Arm.getInstance().moveShooterToPose(target);

  }
}
