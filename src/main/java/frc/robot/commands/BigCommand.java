// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BigCommand extends SequentialCommandGroup {
  /** Creates a new BigCommand. */
  public BigCommand(Intake intake, Shooter shooter, LED led) {

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());

    // BONUS after the last instant command, try to make it so that the robot shoots the note out
    // this must be a instant command, you would need to set the feeders and flywheels at a
    // arbituarily slow speed
    // you can use setShooterFlywheels and setFeeders
    // the shooter flywheels should be both 1000 and feeders should be 500
    // MAKE SURE THERE IS A WAIT COMMAND BEFORE THESE COMMANDS THAT WAITS 2 SECONDS

    addCommands(
        new IntakeNote(intake, shooter, led, false),
        new InstantCommand(() -> led.setState(Constants.LED_STATE.RED), led),
        new WaitCommand(1),
        new InstantCommand(() -> led.setState(Constants.LED_STATE.BLUE), led)
        // new WaitCommand(3)
        //  new InstantCommand(() -> shooter.setFlywheelRPMs(500, 500), shooter),
        //  new InstantCommand(() -> shooter.setFeedersRPM(500)),
        //  new WaitCommand(3),
        // new InstantCommand(() -> shooter.stopFlywheels(), shooter),
        // new InstantCommand(() -> shooter.stopFeeders(), shooter)
        );
  }
}
