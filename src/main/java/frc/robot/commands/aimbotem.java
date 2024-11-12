// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import org.littletonrobotics.junction.Logger;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
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

public class aimbotem extends Command {

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
  /** Creates a new Aimbot. */
  public aimbotem(Drive drive, CommandXboxController controller, Shooter shooter, Pivot pivot, LED led) {
    
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
    //call the angle shooter method
    angleShooter();
    Logger.recordOutput("distance from speak", Units.metersToFeet(calculateDistanceToSpeaker()));
  }

  public void angleShooter() {
    if (DriverStation.getAlliance().isPresent()) this.alliance = DriverStation.getAlliance().get();

    distanceToSpeakerMeter = calculateDistanceToSpeaker();

    //check if the distance to the speaker is less than 6 feet and if so set the flywheel RPMs to 4500 and 4000
    if (Units.metersToFeet(distanceToSpeakerMeter) < 6) {
      shooter.setFlywheelRPMs(4500, 4000);
    } 
    
    //check if the distance is greater than 12.5 feet and if so set the flywheel RPMs using the calculateShooterSpeed method
    else if (Units.metersToFeet(distanceToSpeakerMeter) > 12.5) {
      double shootingSpeed = calculateShooterSpeed(Units.metersToFeet(distanceToSpeakerMeter));
      shooter.setFlywheelRPMs(shootingSpeed, shootingSpeed + 400);
    } 
    
    //if the distance is between 6 and 12.5 feet, set the flywheel RPMs to 5700 and 5000
    else {
        shooter.setFlywheelRPMs(5700, 5000);
    }

    //set the pivot goal to the pivot angle calculated using the calculatePivotAngleDeg method
    pivot.setPivotGoal(calculatePivotAngleDeg(distanceToSpeakerMeter));
    }

    //add a parameter variable distanceToSpeakerFeet
  private double calculateShooterSpeed(double distanceToSpeakerFeet) {
    // set a new var shooterSpeed to the equation -986.49 * the parameter variable + 17294.6
    double shooterSpeed = -986.49 * distanceToSpeakerFeet + 17294.6;

    //clamp the shooterSpeed between 4400 and 5300
    shooterSpeed = MathUtil.clamp(shooterSpeed, 4400, 5300);
    return shooterSpeed;
  }

  private double calculatePivotAngleDeg(double distanceToSpeakerMeter) {
    double shooterOffset = 1.323;

    //given the height of the target speaker, calculate the pivot angle in degrees and set it equal to a variable pivotSetpointDeg (add shooterOffset at the end)
    pivotSetpointDeg =
        Units.radiansToDegrees(Math.atan(2.1 / distanceToSpeakerMeter)) + shooterOffset;

    //if the distance to the speaker is greater than 12.5 feet, return some value (discussed in class)
    if (Units.metersToFeet(distanceToSpeakerMeter) > 12.5) {
      return 32;
    }

    //clamp the pivotSetpointDeg between lowest and heighest (given by me)
    pivotSetpointDeg = MathUtil.clamp(pivotSetpointDeg, 32, 62);
    return pivotSetpointDeg;
  }

  private double calculateDistanceToSpeaker() {
    double x = 0;
    double y = 0;

    //you know what to do :)
    if (alliance == DriverStation.Alliance.Red) {
      x = FieldConstants.fieldLength - drive.getPose().getX();
      y = FieldConstants.Speaker.speakerCenterY - drive.getPose().getY();
    } else {
      x = -drive.getPose().getX();
      y = FieldConstants.Speaker.speakerCenterY - drive.getPose().getY();
    }

    return Math.hypot(x, y);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    shooter.setFeedersRPM(3538);
    led.setState(LED_STATE.GREY);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    Logger.recordOutput("i am currently this angle", drive.getRotation().getDegrees());
    return (pid.atSetpoint() && shooter.atFlywheelSetpoints() && pivot.atGoal())
        || (Timer.getFPGATimestamp() - startTime > 1.323);
    // return shooter.atFlywheelSetpoints();
  }
}
