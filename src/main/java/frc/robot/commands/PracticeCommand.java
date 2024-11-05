// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LED_STATE;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;

public class PracticeCommand extends Command {
  /** Creates a new PracticeCommand. */

  private final LED led;
  private final Intake intake;

  private final LED_STATE color;
  private final double intake_volts;

  public PracticeCommand(LED led, Intake intake, LED_STATE color, double intake_volts) {
    this.led = led;
    this.intake = intake;
    this.color = color;
    this.intake_volts = intake_volts;

    addRequirements(led, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    led.setState(color);
    intake.runRollers(intake_volts);
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
    return false;
  }
}
