// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climb;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Test.MoveToHome;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.climb.Elavator;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ClimbSequence extends SequentialCommandGroup {
  /** Creates a new ClimbSequance. */
  public ClimbSequence() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new MoveToHome(),
      new Climb1()
      // new Climb2(true),
      // new Climb2(false)
    );
  }

  public static class Climb1 extends Command{
    Arm arm;

    public Climb1(){
      arm = Arm.getInstance();
      // addRequirements();
      addRequirements(arm);
    }

    @Override
    public void initialize(){
      arm.moveIntakeToPose(0.25, 0.5);
    }

    @Override
    public boolean isFinished(){
      return arm.isArmInPosition();
    }
  }

  public static class Climb2 extends Command{
    Elavator elavator;
    boolean goingUp;

    public Climb2(boolean goingUp){
      elavator = Arm.getInstance().getElavatorSub();
      this.goingUp = goingUp;

      addRequirements(Arm.getInstance());
    }

    @Override
    public void initialize(){
      if(goingUp) elavator.setRailMotor(Constants.ClimbConstants.chainHeight - (Constants.ArmConstants.armHeightFromFrameMeters + Constants.DriveTrain.Global.RobotHeightFromGround) * 100 + 8);
      else elavator.setRailMotor(Constants.ClimbConstants.chainHeight - (Constants.ArmConstants.armHeightFromFrameMeters + Constants.DriveTrain.Global.RobotHeightFromGround) * 100 - 15);
    }

    @Override 
    public void end(boolean interrupted){
      System.out.println("Climb 2 ended");
    }

    @Override
    public boolean isFinished(){
      return elavator.isRailMotorInPosition();
    }
  }
}
