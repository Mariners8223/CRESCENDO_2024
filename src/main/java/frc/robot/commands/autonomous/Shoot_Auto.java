// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.ArmUtil;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class Shoot_Auto extends SequentialCommandGroup {
  public Shoot_Auto() {
    addCommands(new Shoot_Auto1(),
    new WaitCommand(0.6),
    new InstantCommand(() -> {Arm.getInstance().getShooterSub().stopMotors(); Arm.getInstance().getIntakeSub().stopMotor();}));
  }
  public class Shoot_Auto1 extends Command{
    private static Arm arm;
    private static int timer;

    public Shoot_Auto1(){
      arm = Arm.getInstance();
      timer = 0;
    }
    @Override
    public void initialize() {
      if (DriverStation.isAutonomous() && ArmUtil.isZone1()) {
        arm.getShooterSub().setShooterRPM(4000);
      }
      else arm.getShooterSub().setShooterVelocity(ArmUtil.getWantedSpeed());
      timer = 0;
    }

    @Override
    public void execute() {
      timer++;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      arm.getIntakeSub().setMotor(1);
      Arm.getInstance().getIntakeSub().setIsGamePieceDetected(false);
      if(interrupted){
        Arm.getInstance().getShooterSub().stopMotors();
        Arm.getInstance().getIntakeSub().stopMotor();
      }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return (arm.getShooterSub().isAtSelctedVelocity()) || timer > 60 || (arm.getShooterSub().isMotorsAtSameSpeed());
    }
  }
}
