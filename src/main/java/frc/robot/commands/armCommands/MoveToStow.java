// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.armCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystem.Arm.Arm;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class MoveToStow extends SequentialCommandGroup {
  /** Creates a new MoveArmToStow. */
  public MoveToStow() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveToHome().onlyIf(() -> Arm.getInstance().lastknownPosition != Arm.knownArmPosition.Home),
      new MoveToStow1()
    );
  }

  private static class MoveToStow1 extends Command{
    private Arm arm;

    public MoveToStow1(){
      arm = Arm.getInstance();

      addRequirements(arm);
    }

    @Override
    public void initialize(){
      System.out.println("stow started");
      arm.moveMotorsToRotation(0.0325, Constants.Arm.Motors.secondarySoftLimits[1]);
    }

    @Override
    public void end(boolean interrupted){
      System.out.println("stow finished");
      if(!interrupted) arm.lastknownPosition = Arm.knownArmPosition.Stow;
      else arm.lastknownPosition = Arm.knownArmPosition.Unknown;
    }

    @Override
    public boolean isFinished(){
      return arm.isArmInPosition();
    }
  }
}
