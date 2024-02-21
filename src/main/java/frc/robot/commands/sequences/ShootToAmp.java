// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.sequences;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
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
      new MoveToHome().onlyIf(() -> Arm.getInstance().lastknownPosition == knownArmPosition.Unknown),
      new MoveToAmp().onlyIf(() -> Arm.getInstance().lastknownPosition != knownArmPosition.Amp),
      new MiniShoot(),
      new MoveToHome()
    );
  }

  private static class MiniShoot extends Command {
    Shooter shooter;
    Intake intake;
    int timer;

    public MiniShoot() {
      shooter = Arm.getInstance().getShooterSub();
      intake = Arm.getInstance().getIntakeSub();

      addRequirements(Arm.getInstance());
    }

    @Override
    public void initialize() {
      intake.setMotor(0.4);
      Timer.delay(0.05);
      shooter.setShooterPower(0.4);

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
    }

    @Override
    public boolean isFinished() {
      return timer > 8;
    }
  }

  private static class MoveToAmp extends Command {
    Arm arm;

    public MoveToAmp() {
      arm = Arm.getInstance();

      addRequirements(arm);
    }

    @Override
    public void initialize(){
      arm.moveMotorsToRotation(0.35, 0.3);
    }

    @Override
    public void end(boolean interrupted){
      if(!interrupted) arm.lastknownPosition = Arm.knownArmPosition.Amp;
      else arm.lastknownPosition = Arm.knownArmPosition.Unknown;
    }
    
    @Override 
    public boolean isFinished() {
      return arm.isArmInPosition();
    }
  }
}
