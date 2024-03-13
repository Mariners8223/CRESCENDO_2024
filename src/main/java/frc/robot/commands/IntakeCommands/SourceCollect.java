// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.armCommands.MoveToSourceCollection;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Intake.Intake;
import frc.robot.subsystem.Arm.Shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SourceCollect extends SequentialCommandGroup {
  /** Creates a new SourceCollect. */
  public SourceCollect() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new InnerSourceCollect1(), new InnerSourceCollect2(), new InnerSourceCollect3(),
     new InnerSourceCollect4());
  }

  public class InnerSourceCollect1 extends Command {
    private static Intake intake;
    private static Shooter shooter;
    
    public InnerSourceCollect1(){
      intake = Arm.getInstance().getIntakeSub();
      shooter = Arm.getInstance().getShooterSub();
    }

    @Override
    public void initialize() {
      shooter.setShooterPower(-0.35);
      intake.setMotor(-0.3);
    }

    @Override
    public void end(boolean interrupted) {
      System.out.println("Note is in");
      if (interrupted) {
        intake.stopMotor();
        shooter.stopMotors();
        intake.setIsGamePieceDetected(false);
      }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return intake.getLaserReading(); 
    }
  }
  public class InnerSourceCollect2 extends Command {
    private static Intake intake;
    private static Shooter shooter;
    private double timer;

    public InnerSourceCollect2(){
      intake = Arm.getInstance().getIntakeSub();
      shooter = Arm.getInstance().getShooterSub();
    }

    @Override
    public void initialize(){
      shooter.setShooterPower(-0.2);
      intake.setMotor(-0.3);
      timer = 0;
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      System.out.println("Note is in 1");
      if (interrupted){
        intake.stopMotor();
        shooter.stopMotors();
        intake.setIsGamePieceDetected(false);
      }
    }

    @Override
    public void execute(){
      timer ++;
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return !intake.getLaserReading(); 
    }
  }
  public class InnerSourceCollect3 extends Command {
    private static Intake intake;
    private static Shooter shooter;

    public InnerSourceCollect3(){
      intake = Arm.getInstance().getIntakeSub();
      shooter = Arm.getInstance().getShooterSub();
    }

    @Override
    public void initialize(){
      intake.setMotor(-0.2);
      shooter.setShooterPower(-0.2);
      // shooter.stopMotors();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
      System.out.println("Note is in 2");
      if (interrupted) {
        intake.stopMotor();
        shooter.stopMotors();
        intake.setIsGamePieceDetected(false);
      }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return intake.getLaserReading(); 
    }
  }
  public class InnerSourceCollect4 extends Command {
    private static Intake intake;
    private static Shooter shooter;
    
    public InnerSourceCollect4(){
      intake = Arm.getInstance().getIntakeSub();
      shooter = Arm.getInstance().getShooterSub();
    }

    

    // @Override
    // public void initialize() {
    //   // shooter.setShooterPower(-0.5);
    //   intake.setMotor(-0.3);
    // }

    @Override
    public void end(boolean interrupted) {
      System.out.println("Note is in position");
      shooter.stopMotors();
      // intake.stopMotor();
      intake.setPosition(intake.getMotorPosition());
      intake.setIsGamePieceDetected(true);
      if (interrupted) {
        intake.stopMotor();
        shooter.stopMotors();
        intake.setIsGamePieceDetected(false);
      }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
      return !intake.getLaserReading(); 
    }
  }

  private class InnerSourceCollect5 extends Command{
    private Intake intake;

    private InnerSourceCollect5(){
      intake = Arm.getInstance().getIntakeSub();
    }

    @Override
    public void initialize(){
      intake.setMotor(0.2);
    }

    @Override
    public void end(boolean interrupted){
      intake.setPosition(intake.getMotorPosition());
      intake.setIsGamePieceDetected(true);
      if(interrupted){
        intake.stopMotor();
        intake.setIsGamePieceDetected(false);
      }
    }

    @Override
    public boolean isFinished(){
      return intake.getLaserReading();
    }
  }
}
