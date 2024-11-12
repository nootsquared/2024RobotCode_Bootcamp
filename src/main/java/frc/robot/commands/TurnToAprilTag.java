package frc.robot.commands;

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
import frc.robot.subsystems.pivot.Pivot;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.util.FieldConstants;
import frc.robot.util.LimelightHelpers;

import org.littletonrobotics.junction.Logger;

public class TurnToAprilTag extends Command{
    private final Drive drive;
    private final CommandXboxController controller;
    private final PIDController pid;
    private double[] gains = new double[3];
    private DriverStation.Alliance alliance = null;

    public TurnToAprilTag(Drive drive, CommandXboxController controller){
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

    @Override
    public void initialize() {

    }

    @Override
    public void execute(){
        double targetAngle;
        double tx = LimelightHelpers.getTX("");  // Horizontal offset from crosshair to target in degrees
        double ty = LimelightHelpers.getTY("");  // Vertical offset from crosshair to target in degrees

    }

    @Override
    public void end(boolean interrupted) {

    }

    @Override
    public boolean isFinished() {
      return pid.atSetpoint();
    }

    public void turnToSpeaker() {
        double targetAngle;
        Translation2d targetPos;
        if (DriverStation.getAlliance().isPresent()) this.alliance = DriverStation.getAlliance().get();
    
        if (alliance == DriverStation.Alliance.Red) {
          targetAngle =
              new Rotation2d(
                          (FieldConstants.fieldLength) - drive.getPose().getX(),
                          FieldConstants.Speaker.speakerCenterY - drive.getPose().getY())
                      .getDegrees()
                  + 180;
    
          pid.setSetpoint(
              new Rotation2d(
                          (FieldConstants.fieldLength) - drive.getPose().getX(),
                          FieldConstants.Speaker.speakerCenterY - drive.getPose().getY())
                      .getDegrees()
                  + 180);
        } else {
          targetAngle =
              new Rotation2d(
                          -drive.getPose().getX(),
                          FieldConstants.Speaker.speakerCenterY - drive.getPose().getY())
                      .getDegrees()
                  + 180;
    
          pid.setSetpoint(
              new Rotation2d(
                          -drive.getPose().getX(),
                          FieldConstants.Speaker.speakerCenterY - drive.getPose().getY())
                      .getDegrees()
                  + 180);
        }
    
        Logger.recordOutput("Rotation error2", pid.getPositionError());
    
        Logger.recordOutput("target angle2", targetAngle);
        double linearMagnitude =
            MathUtil.applyDeadband(Math.hypot(-controller.getLeftY(), -controller.getLeftX()), 0.1);
        Rotation2d linearDirection = new Rotation2d(-controller.getLeftY(), -controller.getLeftX());
    
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

        if (alliance == DriverStation.Alliance.Red) {
          targetPos =
              new Translation2d(
                          (FieldConstants.fieldLength) - drive.getPose().getX(),
                          FieldConstants.Speaker.speakerCenterY - drive.getPose().getY());
    
          pid.setSetpoint(
              new Pose2d(
                          (FieldConstants.fieldLength) - drive.getPose().getX(),
                          FieldConstants.Speaker.speakerCenterY - drive.getPose().getY()));
        } else {
          targetAngle =
              new Rotation2d(
                          -drive.getPose().getX(),
                          FieldConstants.Speaker.speakerCenterY - drive.getPose().getY())
                      .getDegrees()
                  + 180;
    
          pid.setSetpoint(
              new Rotation2d(
                          -drive.getPose().getX(),
                          FieldConstants.Speaker.speakerCenterY - drive.getPose().getY())
                      .getDegrees()
                  + 180);
        }


      }
}