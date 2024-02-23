package frc.robot.subsystems.Elevator;

import org.littletonrobotics.junction.AutoLog;

public interface ElevatorExtenderIO {
  @AutoLog
  public static class ElevatorExtenderIOInputs {
    double elevatorPosition = 0;
    double elevatorVelocity = 0;
    double currentAmps = 0;
    double appliedVolts = 0;
    double positionSetpoint = 0;
  }

  public default void updateInputs(ElevatorExtenderIOInputs inputs) {}

  public default void setPositionSetpoint(double position, double ffVolts) {}

  public default void stop() {}

  public default void setVoltage(double volts) {}

  public default void configurePID(double kP, double kI, double kD) {}
}