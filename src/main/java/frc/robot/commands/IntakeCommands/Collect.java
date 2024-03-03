package frc.robot.commands.IntakeCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Intake.Intake;
import frc.robot.Constants;

public class Collect extends SequentialCommandGroup{

  public Collect(){
    addCommands(new Collect1()
    // new WaitCommand(0.25)
    // ,new Collect2()
    );
  }


  private static class Collect1 extends Command{

  private Intake intake;
  private int stallTimer;
  private boolean wasGamePieceDetected;

  public Collect1(){
    intake = Arm.getInstance().getIntakeSub();

    addRequirements(Arm.getInstance());
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

  private static class Collect2 extends Command{
    PIDController controller;
    double startTimer;

    double output;
    private Collect2(){
      controller = new PIDController(0.005, 0, 0);
      controller.setTolerance(20);
      controller.setSetpoint(0);
    }

    @Override
    public void initialize(){
      startTimer = Timer.getFPGATimestamp();
      output = 0;
    }

    @Override
    public void execute(){
      output = MathUtil.clamp(controller.calculate(Arm.getInstance().getIntakeSub().getProximity()), -0.2, 0.2);
      Arm.getInstance().getIntakeSub().setMotor(-output);
      System.out.println(output);
    }

    @Override
    public void end(boolean interrupted){
      System.out.println("finsihed command");
      Arm.getInstance().getIntakeSub().setPosition(Arm.getInstance().getIntakeSub().getMotorPosition());
    }

    @Override
    public boolean isFinished(){
      return controller.atSetpoint() || Timer.getFPGATimestamp() - startTimer >= 3;
    }

  }
}
