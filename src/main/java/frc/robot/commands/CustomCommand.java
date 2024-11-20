// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LED_STATE;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pivot.Pivot;

public class CustomCommand extends Command {
  private final LED led;
  private final Pivot pivot;
  
  private final LED_STATE color;
  private final double pivotAngle;
  
  public CustomCommand(LED led, Pivot pivot, LED_STATE color, double pivotAngle) {
    this.led = led;
    this.pivot = pivot;
    this.color = color;
    this.pivotAngle = pivotAngle;

    addRequirements(led, pivot);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    led.setState(LED_STATE.GREEN);
    pivot.setPivotGoal(pivotAngle);
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
    return Math.abs(pivot.getPivotPositionDegs() - pivotAngle) <= 2;
  }
}
