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
      return Math.atan((Math.pow(StartSpeed, 2)
     - Math.sqrt(Math.pow(StartSpeed, 4)
     - 2*(Constants.SpeakerTranslation.getZ() - target.y - Constants.Arm.RobotHightFromGround)//hieght
      *Constants.gGravity_phisics*Math.pow(StartSpeed, 2) - Math.pow(Constants.gGravity_phisics*(RobotContainer.driveBase.getPose().getTranslation().getX() - Constants.SpeakerTranslation.getX()), 2)))
      /((RobotContainer.driveBase.getPose().getTranslation().getX() - Constants.SpeakerTranslation.getX())*Constants.gGravity_phisics)
    );
    } catch (Exception e) {
      return angle;
    }  
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotContainer.driveBase.getPose().getTranslation().getY() <= Constants.Arm.SpeakerIsCenterRatioBottomLocation) {
      IsDeadZone = true;
          Dy = Constants.Arm.SpeakerBottomLocationY + Constants.Arm.SpeakerLength;//aime to the most right corner (robot prespective)
    }
    else{
      IsDeadZone = false;
          Dy = Constants.Arm.SpeakerBottomLocationY
     + Constants.Arm.SpeakerLength - Constants.Arm.SpeakerIsCenterRatio * (RobotContainer.driveBase.getPose().getTranslation().getY()
      - Constants.Arm.SpeakerIsCenterRatioBottomLocation);//aim to a point prespective to the robot location in the chosen shooting zone
    }
    
    distanceToSpeaker = Math.hypot(RobotContainer.driveBase.getPose().getTranslation().getX() - Constants.SpeakerTranslation.getX(), Dy);

    StartSpeed = Arm.getInstance().getShooterSub().getShooterVelocity();

    angle = Equation();
    try {
      target.rotation = Math.atan((StartSpeed*Math.sin(angle))
    /(StartSpeed*Math.cos(angle) + RobotContainer.driveBase.getChassisSpeeds().vxMetersPerSecond));
    } catch (Exception e) {
      target.rotation = angle;
    }
    Arm.getInstance().moveShooterToPose(target);

  }
}
