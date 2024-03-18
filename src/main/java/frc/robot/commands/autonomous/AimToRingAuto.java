// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.commands.IntakeCommands.IntakeToFloor;
import frc.robot.commands.IntakeCommands.Collect.CollectFloor;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.DriveTrain.DriveBase;
import frc.robot.subsystem.VisionSubSystem.Vision;
import frc.robot.subsystem.VisionSubSystem.Vision.CameraInterface.CameraLocation;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AimToRingAuto extends SequentialCommandGroup {
  /** Creates a new AimToRingAuto. */
  public AimToRingAuto() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeToFloor(),
      new ParallelRaceGroup(
        new AimToRingAutoTrigo(),
        new CollectFloor()
      )
      // new AimToRingAuto1(),
      // new ParallelRaceGroup(
      //   new AimToRingAuto2(),
      //   new CollectFloor()
      // )
    );
  }

  public static class AimToRingAutoTrigo extends Command{
    DriveBase driveBase;
    Vision vision;
    double angleToRing;
    double startTime;

    public AimToRingAutoTrigo(){
      driveBase = RobotContainer.driveBase;
      vision = RobotContainer.vision;

      addRequirements(driveBase);
    }

    @Override
    public void initialize(){
      driveBase.setIsControlled(true);
      startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute(){
      angleToRing = vision.getAngleToBestObject(CameraLocation.Front_Arm);

      if(angleToRing == -1000){
        // driveBase.robotRelativeDrive(1, 0, 0);
        return;

      }

      driveBase.setTargetRotation(Rotation2d.fromDegrees(driveBase.getAngle() - angleToRing), true);
      
      // driveBase.robotRelativeDrive(1.2*Math.cos(Units.degreesToRadians(angleToRing)), -1.2*Math.sin(Units.degreesToRadians(angleToRing)), 0);
      driveBase.robotRelativeDrive(1.2*Math.cos(Units.degreesToRadians(angleToRing)), -1.2*Math.sin(Units.degreesToRadians(angleToRing)), 0);
    }

    @Override
    public void end(boolean interrupted){
      driveBase.setIsControlled(false);
      driveBase.robotRelativeDrive(0, 0, 0);
    }

    @Override
    public boolean isFinished(){
      return Arm.getInstance().getIntakeSub().getLaserReading() || Timer.getFPGATimestamp() - startTime >= 1.1;

    }
  }

  public static class AimToRingAuto1 extends Command{
    DriveBase driveBase;
    Vision vision;
    double angleToRing;
    double startTime;


    public AimToRingAuto1(){
      driveBase = RobotContainer.driveBase;
      vision = RobotContainer.vision;
      angleToRing = 0;

      addRequirements(driveBase);
    }

    @Override
    public void initialize(){
      startTime = Timer.getFPGATimestamp();
      driveBase.setIsControlled(true);
    }

    @Override
    public void execute(){
      angleToRing = vision.getAngleToBestObject(CameraLocation.Front_Arm);

      if(angleToRing == -1000){
        cancel();
        return;
      }

      driveBase.setTargetRotation(Rotation2d.fromDegrees(driveBase.getAngle() - angleToRing), true);

      driveBase.robotRelativeDrive(0, 0, 0);
    }

    @Override
    public void end(boolean interrupted){
      driveBase.setIsControlled(false);
      driveBase.robotRelativeDrive(0, 0, 0);
    }

    @Override
    public boolean isFinished(){
      return angleToRing <= Constants.Vision.aimToRingToleranceDegrees || Timer.getFPGATimestamp() - startTime >= 0.25;
    }

  }

  public static class AimToRingAuto2 extends Command{
    DriveBase driveBase;
    Vision vision;
    double angleToRing;
    double startTime;

    public AimToRingAuto2(){
      driveBase = RobotContainer.driveBase;
      vision = RobotContainer.vision;

      addRequirements(driveBase);
    }

    @Override
    public void initialize(){
      driveBase.setIsControlled(true);
      startTime = Timer.getFPGATimestamp();
    }

    @Override
    public void execute(){
      angleToRing = vision.getAngleToBestObject(CameraLocation.Front_Arm);

      if(angleToRing == -1000){
        cancel();
        return;
      }

      driveBase.setTargetRotation(Rotation2d.fromDegrees(driveBase.getAngle() - angleToRing), true);

      driveBase.robotRelativeDrive(1, 0, 0);
    }

    @Override
    public void end(boolean interrupted){
      driveBase.setIsControlled(false);
      driveBase.robotRelativeDrive(0, 0, 0);
    }

    @Override
    public boolean isFinished(){
      return Arm.getInstance().getIntakeSub().isGamePieceDetected() || Timer.getFPGATimestamp() - startTime >= 1;
    }
  }
}
