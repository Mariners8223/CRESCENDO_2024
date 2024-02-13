// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;


import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Arm.ArmPostion;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimShooterZone1 extends InstantCommand {
  private static ArmPostion target = new ArmPostion(0.4, 0.4, 0);
  private static double angle;
  private static double distanceToSpeaker;
  private static double Y;
  public static boolean IsDeadZone;

  public AimShooterZone1() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Arm.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (RobotContainer.driveBase.getPose().getTranslation().getY() <= Constants.ArmConstants.SpeakerIsCenterRatioBottomLocation) {
      IsDeadZone = true;
          Y = Constants.ArmConstants.SpeakerBottomLocationY + Constants.ArmConstants.SpeakerLength;//aime to the most right corner (robot prespective)
    }
    else{
      IsDeadZone = false;
          Y = Constants.ArmConstants.SpeakerBottomLocationY
     + Constants.ArmConstants.SpeakerLength - Constants.ArmConstants.SpeakerIsCenterRatio * (RobotContainer.driveBase.getPose().getTranslation().getY()
      - Constants.ArmConstants.SpeakerIsCenterRatioBottomLocation);//aim to a point prespective to the robot location in the chosen shooting zone
    }
    
    distanceToSpeaker = Math.hypot(RobotContainer.driveBase.getPose().getTranslation().getX() - Constants.SpeakerTranslation.getX(), Y);
  
    angle = Math.atan((Constants.SpeakerTranslation.getZ() - target.y - Constants.ArmConstants.RobotHightFromGround) / distanceToSpeaker);
    target.rotation = Math.atan((RobotContainer.arm.getShooter().getShooterVelocity()*Math.sin(angle))
    /(RobotContainer.arm.getShooter().getShooterVelocity()*Math.cos(angle) + RobotContainer.driveBase.getChassisSpeeds().vxMetersPerSecond));

    Arm.getInstance().moveShooterToPose(target);
  }
}
