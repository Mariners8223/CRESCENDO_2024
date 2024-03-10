package frc.robot.commands.IntakeCommands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Intake.Intake;
import frc.robot.Constants;

public class Collect extends SequentialCommandGroup{
  private static boolean wasGamePieceDetected;

  public Collect(){
    addCommands(
      new Collect1(),
      new SequentialCommandGroup(
        new InstantCommand(() -> Arm.getInstance().getIntakeSub().setMotor(-1)),
        new WaitCommand(0.05),
        new InstantCommand(() -> Arm.getInstance().getIntakeSub().stopMotor())
      ).onlyIf(() -> wasGamePieceDetected),
      new InstantCommand(() -> Arm.getInstance().getIntakeSub().setPosition(Arm.getInstance().getIntakeSub().getMotorPosition() + 8)).onlyIf(() -> !wasGamePieceDetected)

    );
  }

  private static class Collect1 extends Command{
  private Intake intake;
  private int stallTimer;

  public Collect1(){
    intake = Arm.getInstance().getIntakeSub();

  }

  @Override
  public void initialize(){
    wasGamePieceDetected = intake.isGamePieceDetected();
    stallTimer = 0;

    if(!wasGamePieceDetected)
      intake.setMotor(Constants.Intake.intakeMotorSpeed);
    else
      intake.setMotor(-Constants.Intake.intakeMotorSpeed);

  }

  @Override
  public void execute(){
    // Check if Motors are in stall for too much time
    if (intake.getCurrent() > Constants.Intake.StallCurrent) stallTimer++;
    else stallTimer = 0;
  }

  @Override
  public void end(boolean interrupted){
  }

  @Override
  public boolean isFinished(){
    return stallTimer >= Constants.Intake.MaxStallTime || wasGamePieceDetected != intake.isGamePieceDetected();
  }
}
}
