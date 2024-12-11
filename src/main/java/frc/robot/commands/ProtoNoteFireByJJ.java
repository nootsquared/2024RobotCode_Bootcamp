// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PIDCommand;
import frc.robot.Constants;
import frc.robot.Constants.LED_STATE;
import frc.robot.Constants.NoteState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;

public class ProtoNoteFireByJJ extends Command {
  /** Creates a new ProtoNoteFireByJJ. */

  LED led;
  Pivot pivot;
  Drive drive;
  Shooter shooter;
  Intake intake;
  Command pathCommand;
  Translation2d noteTranslation2d;

  
  public ProtoNoteFireByJJ(LED led, Pivot pivot, Drive drive, Shooter shooter, Intake intake) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.led = led;
    this.pivot = pivot;
    this.drive = drive;
    this.shooter = shooter;
    this.intake =intake;   
    addRequirements(led, pivot, drive, shooter, intake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    noteTranslation2d = drive.getCachedNoteLocation();
    intake.runRollers(12);
    shooter.setFeedersRPM(499);
    pivot.setPivotGoal(Constants.PivotConstants.INTAKE_SETPOINT_DEG);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    //led.setState(LED_STATE.GREEN);
    if ((shooter.seesNote() == NoteState.CURRENT || shooter.seesNote() == NoteState.SENSOR)) {
      end(true);
    }

    Rotation2d xu = new Rotation2d(noteTranslation2d.getX(), noteTranslation2d.getY());

    PIDController pid = new PIDController(7.5, 7.5, 7.5);
    pid.se

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (shooter.seesNote() == NoteState.CURRENT || shooter.seesNote() == NoteState.SENSOR);
  }
}