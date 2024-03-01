// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PivotIntake extends SequentialCommandGroup {
  /** Creates a new PivotIntake. */
  public PivotIntake(Pivot pivot, Intake intake, Shooter shooter, boolean outtake) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    if (!outtake) {
      addCommands(
          new SetPivotTarget(Constants.PivotConstants.INTAKE_SETPOINT_DEG, pivot),
          // new WaitUntilCommand(pivot::pivotAtSetpoint),
          new InstantCommand(
              () -> intake.runRollers(Constants.IntakeConstants.APPLIED_VOLTAGE), intake),
          new InstantCommand(() -> shooter.setFeedersRPM(1000)));
    } else {
      addCommands(
          new SetPivotTarget(Constants.PivotConstants.INTAKE_SETPOINT_DEG, pivot),
          // new WaitUntilCommand(pivot::pivotAtSetpoint),
          new InstantCommand(() -> shooter.setFeedersRPM(-4000)),
          new InstantCommand(
              () -> intake.runRollers(-Constants.IntakeConstants.APPLIED_VOLTAGE), intake));
    }
  }
}
