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
  private boolean isAutoAlign;

  public IntakeNote(Intake intake, Shooter shooter, LED led, boolean isAutoAlign) {
    this.intake = intake;
    this.shooter = shooter;
    this.led = led;

    this.isAutoAlign = isAutoAlign;

    addRequirements(intake, shooter, led);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    intake.runRollers(12);
    shooter.setFeedersRPM(500);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // if note(s) stuck inside the intake, turn purple
    // if intaking with autoalign, flash red
    // if intaking, turn red
    if (intake.getVolts() < 1 && intake.getAmps() > 38 && intake.getRPM() < 44.14) {
      led.setState(LED_STATE.PURPLE);
    } else {
      if (isAutoAlign) {
        led.setState(LED_STATE.FLASHING_RED);
      } else {
        led.setState(LED_STATE.RED);
      }
    }
    // if the intake is currently intaking a note or there is a note inside the robot
    if (shooter.seesNote() == NoteState.CURRENT || shooter.seesNote() == NoteState.SENSOR)
      end(true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    intake.changeLEDBoolTrue();
    shooter.setFeedersRPM(0);
    intake.stopRollers();
    // if (shooter.seesNote()) ;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return shooter.seesNote() == NoteState.SENSOR || shooter.seesNote() == NoteState.CURRENT;
  }
}
