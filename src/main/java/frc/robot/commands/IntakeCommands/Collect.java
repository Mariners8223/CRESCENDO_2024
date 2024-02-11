package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Intake.Intake;
import frc.robot.Constants.ArmConstants;

public class Collect extends Command{
  private Intake intake;
  private int timer;
  private boolean wasGamePieceDetected;

  public Collect(){
    intake = Arm.getInstance().getIntakeSub();
    timer = 0;
  }

  @Override
  public void initialize(){
    wasGamePieceDetected = intake.isGamePieceDetected();

    if(!wasGamePieceDetected) intake.setMotor(ArmConstants.Intake.intakeMotorSpeed);
    else intake.setMotor(-ArmConstants.Intake.intakeMotorSpeed);
  }

  @Override
  public void execute(){
    // Check if Motors are in stall for too much time
    if (intake.getCurrent() > ArmConstants.Intake.StallCurrent){
      timer++;
    }
    else{
        timer = 0;
    }
  }

  @Override
  public void end(boolean interrupted){
    if(wasGamePieceDetected){
      intake.setMotor(-1);
      Timer.delay(0.05);
      intake.stopMotor();
    }
    else {
      intake.setMotor(-0.4);
      Timer.delay(0.085);
      intake.stopMotor();
    }

    if (timer >= ArmConstants.Intake.MaxStallTime){
        intake.setMotor(-0.3);
        Timer.delay(2);
        intake.stopMotor();
    }
  }

  @Override
  public boolean isFinished(){
    if(!wasGamePieceDetected) return intake.getProximity() > ArmConstants.Intake.CloseProximity || timer >= ArmConstants.Intake.MaxStallTime;
    else return intake.getProximity() < ArmConstants.Intake.CloseProximity || timer >= ArmConstants.Intake.MaxStallTime;
  }
}