// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.subsystems.drive.Drive;

public class TURNTOAMPTEMPLATE extends Command {
  private final Drive drive;
  private final CommandXboxController controller;
  private final PIDController pid;
  private double[] gains = new double[3];
  private DriverStation.Alliance alliance = null;

  public TURNTOAMPTEMPLATE (Drive drive, CommandXboxController controller) {
    this.drive = drive;

    this.controller = controller;
    addRequirements(drive);

    switch (Constants.currentMode) {
      case REAL:
        gains[0] = 7;
        gains[1] = 0;
        gains[2] = 0;
        break;
      case REPLAY:
        gains[0] = 0;
        gains[1] = 0;
        gains[2] = 0;
        break;
      case SIM:
        gains[0] = 13;
        gains[1] = 0;
        gains[2] = 01;
        break;
      default:
        gains[0] = 0;
        gains[1] = 0;
        gains[2] = 0;
        break;
    }

    pid = new PIDController(gains[0], gains[1], gains[2], 0.02);
    pid.setTolerance(0);
    pid.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double targetAngle;
    if (DriverStation.getAlliance().isPresent()) this.alliance = DriverStation.getAlliance().get();

    //turn to amp code (talk to students abt difference between amping and speaker shots)

    Logger.recordOutput("target angle amp", targetAngle);
    double linearMagnitude =
        MathUtil.applyDeadband(Math.hypot(-controller.getLeftY(), -controller.getLeftX()), 0.1);
    Rotation2d linearDirection = new Rotation2d(-controller.getLeftY(), -controller.getLeftX());

    // Square values
    linearMagnitude = linearMagnitude * linearMagnitude;
    // Calcaulate new linear velocity
    Translation2d linearVelocity =
        new Pose2d(new Translation2d(), linearDirection)
            .transformBy(new Transform2d(linearMagnitude, 0.0, new Rotation2d()))
            .getTranslation();
    double angularSpeed = pid.calculate(drive.getPose().getRotation().getDegrees());

    drive.runVelocity(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            linearVelocity.getX() * drive.getMaxLinearSpeedMetersPerSec(),
            linearVelocity.getY() * drive.getMaxLinearSpeedMetersPerSec(),
            Math.toRadians(angularSpeed),
            drive.getPose().getRotation()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return pid.atSetpoint();
  }
}
