package frc.robot.commands.IntakeCommands;

import com.revrobotics.ColorSensorV3;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystem.Arm.Intake.Intake;
import frc.robot.Constants.ArmConstants.IntakeConstants;

public class Collect extends Command{
  private Intake intake;
  private ColorSensorV3 colourSensor;
  private int timer;

  public Collect(){
    intake = Intake.getInstance();
    colourSensor = new ColorSensorV3(IntakeConstants.ColourSensorPort);
    timer = 0;
  }

  @Override
  public void initialize(){
    intake.setMotor(IntakeConstants.intakeMotorSpeed);
  }

  @Override
  public void execute(){
    // Check if Motors are in stall for too much time
    if (intake.getCurrent() > IntakeConstants.StallCurrent){
      timer++;
    }

    else if (timer > 0){
        timer = 0;
    }
  }

  @Override
  public void end(boolean interrupted){
    intake.stopMotor();

    if (timer >= IntakeConstants.MaxStallTime){
        intake.setMotor(-0.3);
        Timer.delay(2);
        intake.stopMotor();
    }
  }

  @Override
  public boolean isFinished(){
    return (colourSensor.getProximity() < IntakeConstants.CloseProximity || timer >= IntakeConstants.MaxStallTime);
  }
}