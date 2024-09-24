// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.intake.Intake;
import frc.robot.subsystems.led.LED;
import frc.robot.subsystems.shooter.Shooter;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SampleCommand extends SequentialCommandGroup {
  /** Creates a new SampleCommand. */
  public SampleCommand(Intake intake, Shooter shooter, LED led) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new IntakeNote(intake, shooter, led, false),
      new InstantCommand(()-> [subsystem].[subsystem method], [subsystem]),
      new WaitCommand([seconds]),
      new InstantCommand(()-> [subsystem].[subsystem method], [subsystem])
    );
  }
}
