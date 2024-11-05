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
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.LED_STATE;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.FieldConstants;

public class TURNTOCOMMAND extends Command {

  private final Drive drive;
  private final Shooter shooter;
  private final Pivot pivot;
  private final LED led;

  private double startTime;

  private final CommandXboxController controller;
  private final PIDController pid;
  private double[] gains = new double[3];
  private DriverStation.Alliance alliance = null;

  private double distanceToSpeakerMeter = 0;
  private double pivotSetpointDeg = 0;

  
  public TURNTOCOMMAND(
      Drive drive, CommandXboxController controller, Shooter shooter, Pivot pivot, LED led) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.drive = drive;
    this.shooter = shooter;
    this.pivot = pivot;
    this.led = led;

    this.controller = controller;

    addRequirements(drive, shooter, pivot, led);

    switch (Constants.currentMode) {
      case REAL:
        gains[0] = 3.0;
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
    pid.setTolerance(2);
    pid.enableContinuousInput(-180, 180);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    startTime = Timer.getFPGATimestamp();
    led.setState(LED_STATE.FLASHING_GREEN);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    turnToSpeaker();
    // angleShooter();
    Logger.recordOutput("distance from speak", Units.metersToFeet(calculateDistanceToSpeaker()));
  }

  // public void angleShooter() {
  //   if (DriverStation.getAlliance().isPresent()) this.alliance = DriverStation.getAlliance().get();
  //   distanceToSpeakerMeter = calculateDistanceToSpeaker();

  //   if (Units.metersToFeet(distanceToSpeakerMeter) < 6) {
  //     shooter.setFlywheelRPMs(4500, 4000);
  //   } 

  //   else if (Units.metersToFeet(distanceToSpeakerMeter) > 12.5) {
  //     double shootingSpeed = calculateShooterSpeed(Units.metersToFeet(distanceToSpeakerMeter));

  //     shooter.setFlywheelRPMs(shootingSpeed, shootingSpeed + 400);
  //   } 
    
  //   else shooter.setFlywheelRPMs(5700, 5000);

  //   pivot.setPivotGoal(calculatePivotAngleDeg(distanceToSpeakerMeter));
  // }

  // private double calculateShooterSpeed(double distanceToSpeakerFeet) {
  //   double shooterSpeed = -986.49 * distanceToSpeakerFeet + 17294.6;
  //   shooterSpeed = MathUtil.clamp(shooterSpeed, 4400, 5300);
  //   return shooterSpeed;
  // }

  // private double calculatePivotAngleDeg(double distanceToSpeakerMeter) {
  //   double shooterOffset = 1.323;
  //   pivotSetpointDeg =
  //       Units.radiansToDegrees(Math.atan(2.1 / distanceToSpeakerMeter)) + shooterOffset;
  //   if (Units.metersToFeet(distanceToSpeakerMeter) > 12.5) {
  //     return 32;
  //   }
  //   pivotSetpointDeg = MathUtil.clamp(pivotSetpointDeg, 32, 62);
  //   return pivotSetpointDeg;
  // }

  private double calculateDistanceToSpeaker() {
    double x = 0;
    double y = 0;

    if (alliance == DriverStation.Alliance.Red) {
      x = FieldConstants.fieldLength - drive.getPose().getX();
      y = FieldConstants.Speaker.speakerCenterY - drive.getPose().getY();
    } else {
      x = -drive.getPose().getX();
      y = FieldConstants.Speaker.speakerCenterY - drive.getPose().getY();
    }

    return Math.hypot(x, y);
  }

  public void turnToSpeaker() {
    double targetAngle;
    if (DriverStation.getAlliance().isPresent()) this.alliance = DriverStation.getAlliance().get();

    //ADD YOUR RED AND BLUE ALLIANCE TARGET ANGLE AND PIDSETPOINT HERE (HINT: 2 CONDITIONALS)

    Logger.recordOutput("Rotation error", pid.getPositionError());

    Logger.recordOutput("target angle", targetAngle);
    double linearMagnitude =
        MathUtil.applyDeadband(Math.hypot(-controller.getLeftY(), -controller.getLeftX()), 0.1);
    Rotation2d linearDirection = new Rotation2d(-controller.getLeftY(), -controller.getLeftX());

    linearMagnitude = linearMagnitude * linearMagnitude;
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

  @Override
  public void end(boolean interrupted) {
    // shooter.setFeedersRPM(3538);
    led.setState(LED_STATE.GREY);
  }

  @Override
  public boolean isFinished() {
    Logger.recordOutput("i am currently this angle", drive.getRotation().getDegrees());
    return (pid.atSetpoint())
        || (Timer.getFPGATimestamp() - startTime > 1.323);
  }
}
