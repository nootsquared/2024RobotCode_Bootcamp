// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.Constants.PIVOT_POSITIONS;

public class RecapCommand2 extends Command {
  private final Pivot pivot;

  private PIVOT_POSITIONS setPoint;

  public RecapCommand2(PIVOT_POSITIONS setPoint, Pivot pivot) {
    this.setPoint = setPoint;
    this.pivot = pivot;

    addRequirements(pivot);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  public void initialize() {
    pivot.setPivotPosition(setPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}
  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(pivot.getPivotPositionDegs() - pivot.getPivotPosition(setPoint)) <= 2;
  }
}