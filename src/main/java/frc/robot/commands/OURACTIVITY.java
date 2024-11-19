// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants;
import frc.robot.Constants.LED_STATE;
import frc.robot.Constants.NoteState;
import frc.robot.subsystems.drive.Drive;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;

//ADD CODE HERE AND DRIVE.JAVA @LINE 889

public class OURACTIVITY extends Command {
  /** Creates a new AlignToNote. */


  //INSTANTIATE ALL VARS

  public OURACTIVITY(Intake intake, Pivot pivot, Shooter shooter, Drive drive, LED led, CommandXboxController controller) {

    //SET ALL VARS TO THE PARAMS PASSED IN

    pid = new PIDController(0.01, 0, 0, 0.02);
    pid.setTolerance(2);
    pid.enableContinuousInput(-180, 180);
    addRequirements(drive, intake, pivot, shooter, led);
  }

  @Override
  public void initialize() {
    //ADD INITS
  }

  @Override
  public void execute() {
    
    //CALL THE END METHOD IF THE SHOOTER SEES THE NOTE (THE SENSOR INSIDE THE SHOOTER)

    //TURN THE ROBOT TO THE ANGLE

    //DO NOT CHANGE
    //--------------------------------
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
  //--------------------------------

  @Override
  public void end(boolean interrupted) {
  }

  @Override
  public boolean isFinished() {

    //RETURN TRUE IF THE SHOOTER SEES THE NOTE (THE SENSOR INSIDE THE SHOOTER)
    //SET THE LED TO GREEN ASWELL (LED.setSTATE(LED_STATE.GREEN))

  }
}
