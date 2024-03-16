// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.autonomous;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.IntakeCommands.IntakeToFloor;
import frc.robot.subsystem.Arm.Arm;
import frc.robot.subsystem.Arm.Arm.knownArmPosition;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html

public class LowerAim_Auto extends SequentialCommandGroup {
  public LowerAim_Auto(){
    addCommands(new IntakeToFloor().onlyIf(() -> Arm.getInstance().lastknownPosition != knownArmPosition.Intake),
    new LowerAim_Auto1());
  }

  public class LowerAim_Auto1 extends InstantCommand {
    private static Arm arm;
    public LowerAim_Auto1() {
      // Use addRequirements() here to declare subsystem dependencies.
      arm = Arm.getInstance();
      addRequirements(arm);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {//distance from speaker = 5.3612.....
      arm.moveMotorsToRotation(0, Units.degreesToRotations(180 - 34.402));
    }
  }
}
