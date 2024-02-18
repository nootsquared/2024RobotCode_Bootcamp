package frc.robot.subsystems.intake;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.SparkPIDController;
import frc.robot.Constants;

public class IntakeRollerIOSparkFlex implements IntakeRollerIO {
  private final CANSparkFlex rollers;
  private final SparkPIDController pid;

  public IntakeRollerIOSparkFlex(int id) {
    rollers = new CANSparkFlex(id, MotorType.kBrushless);
    rollers.restoreFactoryDefaults();
    rollers.setIdleMode(IdleMode.kCoast);

    rollers.setSmartCurrentLimit(Constants.IntakeConstants.ROLLER_CURRENT_LIMIT);
    rollers.setCANTimeout(250);
    rollers.burnFlash();

    pid = rollers.getPIDController();
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {
    inputs.rollerVelocity = rollers.getEncoder().getVelocity();

    inputs.appliedVolts = rollers.getBusVoltage();
    inputs.currentAmps = rollers.getOutputCurrent();
  }

  @Override
  public void setVelocity(double velocity) {
    rollers.set(velocity);
    // pid.setReference(velocity, ControlType.kVelocity);
  }

  @Override
  public void stop() {
    rollers.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    pid.setP(kP, 0);
    pid.setI(kI, 0);
    pid.setD(kD, 0);
  }
}