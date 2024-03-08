package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Intake.Intake;
import frc.robot.Constants;

public class Collect extends Command{

  private Intake intake;
  private int stallTimer;
  private boolean wasGamePieceDetected;

  public Collect(){
    intake = Arm.getInstance().getIntakeSub();

    // addRequirements(Arm.getInstance());
  }

  @Override
  public void initialize(){
    wasGamePieceDetected = intake.isGamePieceDetected();
    stallTimer = 0;

    if(!wasGamePieceDetected)
      intake.setMotor(Constants.Intake.intakeMotorSpeed);
    else
      intake.setMotor(-Constants.Intake.intakeMotorSpeed);

    // Timer.delay(0.15);
  }

  @Override
  public void execute(){
    // Check if Motors are in stall for too much time
    if (intake.getCurrent() > Constants.Intake.StallCurrent) stallTimer++;
    else stallTimer = 0;
  }

  @Override
  public void end(boolean interrupted){
    if(wasGamePieceDetected){
      intake.setMotor(-1);
      Timer.delay(0.05);
      intake.stopMotor();
    }
    else {
      intake.setPosition(intake.getMotorPosition() + 8);
    }
  }

  @Override
  public boolean isFinished(){
    return stallTimer >= Constants.Intake.MaxStallTime || wasGamePieceDetected != intake.isGamePieceDetected();
  }
}
