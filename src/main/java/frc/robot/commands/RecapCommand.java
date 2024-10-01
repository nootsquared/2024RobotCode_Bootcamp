// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ELEVATOR_POSITIONS;
import frc.robot.subsystems.elevator.Elevator;

public class RecapCommand extends Command {
  /** Creates a new RecapCommand. */
  private final Elevator elevator;

  private ELEVATOR_POSITIONS position;
  public RecapCommand(Elevator elevator, ELEVATOR_POSITIONS position) {
    this.elevator = elevator;
    this.position = position;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    elevator.setExtenderPosition(position);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    elevator.elevatorStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(elevator.getElevatorPosition() - elevator.getExtenderPosition(position)) <= 2;
  }
}
