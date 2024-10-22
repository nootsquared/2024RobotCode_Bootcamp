package frc.robot.subsystems.intake;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import frc.robot.Constants;

public class IntakeRollerIOTalonFX implements IntakeRollerIO {
  private final TalonFX falcon;

  private final StatusSignal<Double> rollerRotations;
  private final StatusSignal<Double> rollerVelocityRPS;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> currentAmps;

  public IntakeRollerIOTalonFX(int id) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = Constants.IntakeConstants.CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable = Constants.IntakeConstants.CURRENT_LIMIT_ENABLED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    falcon = new TalonFX(id);

    falcon.getConfigurator().apply(config);

    rollerRotations = falcon.getPosition();
    rollerVelocityRPS = falcon.getVelocity();
    appliedVolts = falcon.getMotorVoltage();
    currentAmps = falcon.getStatorCurrent();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, rollerVelocityRPS, appliedVolts, currentAmps, rollerRotations);
  }

  @Override
  public void updateInputs(IntakeRollerIOInputs inputs) {

    // refresh the signal and get all the inputs as a double value
  }

  @Override
  public void setVoltage(double volts) {
    // set the motors voltage to volts
  }

  @Override
  public void stop() {
    // stop the motor
  }
}
