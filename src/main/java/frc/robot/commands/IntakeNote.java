// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LED_STATE;
import frc.robot.Constants.NoteState;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.shooter.Shooter;

public class IntakeNote extends Command {
  /** Creates a new IntakeNote. */
  private final Intake intake;
  private final LED led;
  private final Shooter shooter;

  public IntakeNote(Intake intake, Shooter shooter, LED led) {
    this.intake = intake;
    this.shooter = shooter;
    this.led = led;
    addRequirements(intake, shooter, led);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // set the intake rollers to 12 volts and the shooter feeders to 500 RPM
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // fill in the blank to log the intake RPM
    Logger.recordOutput("intake rpm", ______);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // when the command ends, stop the shooter feeder and stop the intake rollers
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // fill in the blanks to tell the command its finished when the shooter sees a note (hint: look at shooter.java)
    boolean weSawANote = ____;
    
    return weSawANote;
  }
}
