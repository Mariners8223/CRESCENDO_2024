// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.armCommands.MoveToHome;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Arm.knownArmPosition;
import frc.robot.subsystem.Arm.Intake.Intake;
import frc.robot.subsystem.Arm.Shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootToAmp extends SequentialCommandGroup {
  /** Creates a new ShootToAmp. */
  public ShootToAmp() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveToHome().onlyIf(() -> Arm.getInstance().lastknownPosition != knownArmPosition.Home || Arm.getInstance().lastknownPosition != knownArmPosition.Free || Arm.getInstance().lastknownPosition != knownArmPosition.Intake),
      new SequentialCommandGroup(new MoveToAmp(true), new MoveToAmp(false)).onlyIf(() -> Arm.getInstance().lastknownPosition != knownArmPosition.Amp)
    );
  }

  public static class MiniShoot extends SequentialCommandGroup {

    public MiniShoot() {
      addCommands(
        new InstantCommand(() -> Arm.getInstance().getShooterSub().setShooterPower(-0.35)),
        new MiniShoot1()
        );
    }

    private static class MiniShoot1 extends Command{
    Shooter shooter;
    Intake intake;
    int timer;

    public MiniShoot1() {
      shooter = Arm.getInstance().getShooterSub();
      intake = Arm.getInstance().getIntakeSub();
    }

    @Override
    public void initialize() {
      // shooter.setShooterPower(0.65);
      intake.setMotor(-0.8);

      timer = 0;
    }

    @Override
    public void execute() {
      timer++;
    }

    @Override
    public void end(boolean interrupted){
      intake.stopMotor();
      shooter.stopMotors();
      intake.setIsGamePieceDetected(false);
    }

    @Override
    public boolean isFinished() {
      return timer >= 20;
    }
  }
  }

  private static class MoveToAmp extends Command {
    Arm arm;
    boolean isMain;

    public MoveToAmp(boolean isMain) {
      arm = Arm.getInstance();
      this.isMain = isMain;

      addRequirements(arm);
    }

    @Override
    public void initialize(){
      if(this.isMain){
      arm.moveMotorsToRotation(0.183, 0);
      }
      else arm.moveMotorsToRotation(arm.getMainMotorRotation(), 0.254);
    }

    @Override
    public void end(boolean interrupted){
      if(interrupted) arm.lastknownPosition = Arm.knownArmPosition.Unknown;
    }
    
    @Override 
    public boolean isFinished() {
      return arm.isArmInPosition();
    }
  }

  private static class MoveToAmp2 extends Command{
    Arm arm;

    public MoveToAmp2(){
      arm = Arm.getInstance();

      addRequirements(arm);
    }

    @Override
    public void initialize(){
      arm.moveMotorsToRotation(arm.getMainMotorRotation(), 0.33);
    }

    @Override
    public void end(boolean interrupted){
      if(!interrupted) arm.lastknownPosition = Arm.knownArmPosition.Amp;
      else arm.lastknownPosition = Arm.knownArmPosition.Unknown;
    }

    @Override
    public boolean isFinished(){
      return arm.isArmInPosition();
    }
  }
}