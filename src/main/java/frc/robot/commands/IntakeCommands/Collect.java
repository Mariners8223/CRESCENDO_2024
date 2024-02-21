package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Intake.Intake;
import frc.robot.Constants;

public class Collect extends Command{
  private Intake intake;
  private int timer;
  private boolean wasGamePieceDetected;

  public Collect(){
    intake = Arm.getInstance().getIntakeSub();

    addRequirements(Arm.getInstance());
  }

  @Override
  public void initialize(){
    wasGamePieceDetected = intake.isGamePieceDetected();
    timer = 0;

    if(!wasGamePieceDetected) intake.setMotor(Constants.Intake.intakeMotorSpeed);
    else intake.setMotor(-Constants.Intake.intakeMotorSpeed);
    Timer.delay(0.15);
  }

  @Override
  public void execute(){
    // Check if Motors are in stall for too much time
    if (intake.getCurrent() > Constants.Intake.StallCurrent) timer++;
    else timer = 0;
  }

  @Override
  public void end(boolean interrupted){
    if(wasGamePieceDetected){
      intake.setMotor(-1);
      Timer.delay(0.05);
      intake.stopMotor();
    }
    else {
      intake.setPosition(intake.getMotorPosition() + 7);
    }

    // if (timer >= Constants.Intake.MaxStallTime){
    //     intake.setMotor(-0.3);
    //     Timer.delay(2);
    //     intake.stopMotor();
    // }
  }

  @Override
  public boolean isFinished(){
    if(!wasGamePieceDetected) return intake.isGamePieceDetected() || timer >= Constants.Intake.MaxStallTime;
    else return intake.getProximity() > 200 || timer >= Constants.Intake.MaxStallTime;
  }
}
