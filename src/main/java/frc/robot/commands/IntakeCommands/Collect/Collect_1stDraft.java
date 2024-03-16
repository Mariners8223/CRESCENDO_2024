package frc.robot.commands.IntakeCommands.Collect;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Intake.Intake;

public class Collect_1stDraft extends Command{

  private static Intake intake;
  private static int timer;
  private static int detectionTime;//when was it last deteced
  private static int LazerBreaks;//how many times did the gp triger the sensor
  private static boolean isStuckGamePiece;//is the game piece stuck
  private static boolean lastLazerReading;

  private static void StuckGamePieceProtocal(){
    Arm.getInstance().getShooterSub().setShooterPower(-0.2);
    intake.setMotor(-0.5);
  }

  public Collect_1stDraft(){
    intake = Arm.getInstance().getIntakeSub();
  }

  @Override
  public void initialize(){
    timer = 0;
    detectionTime = 1000;//so it wont trigger end
    LazerBreaks = 0;
    isStuckGamePiece = false;
    lastLazerReading = intake.getLaserReading();
    if (intake.isGamePieceDetected()) {
      StuckGamePieceProtocal();
    }
    else intake.setMotor(0.8);
  }

  @Override
  public void execute(){
    timer++;
    if(intake.getLaserReading() != lastLazerReading){//was there a chnge?
      lastLazerReading = intake.getLaserReading();

      if(lastLazerReading == true){//if so, is the lazer broken?
        detectionTime = timer;
        LazerBreaks++;
      } 
    }

    if (timer - detectionTime >= 10) {
      isStuckGamePiece = true;
    }
  }

  @Override
  public void end(boolean interrupted){
    if (isStuckGamePiece || intake.isGamePieceDetected()) {
      intake.setIsGamePieceDetected(false);
      StuckGamePieceProtocal();
    }
    else{
      if (!interrupted){
        intake.setIsGamePieceDetected(true);
      }
    }
    
  }

  @Override
  public boolean isFinished(){
    return isStuckGamePiece || intake.isGamePieceDetected() || LazerBreaks == 2;
  }
}
