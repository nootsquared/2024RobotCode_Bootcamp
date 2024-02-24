package frc.robot.subsystems.Elevator;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.util.Units;
import frc.robot.Constants;

public class ElevatorPivotIOTalonFX implements ElevatorPivotIO {
  private final TalonFX leader;
  private final TalonFX follower;

  private final Pigeon2 pigeon;

  private double positionSetpoint;

  private final StatusSignal<Double> pivotPosition;
  private final StatusSignal<Double> pivotVelocity;
  private final StatusSignal<Double> appliedVolts;
  private final StatusSignal<Double> currentAmps;
  private final StatusSignal<Double> pitch;

  public ElevatorPivotIOTalonFX(int leadID, int followID, int gyroID) {
    TalonFXConfiguration config = new TalonFXConfiguration();
    config.CurrentLimits.StatorCurrentLimit = Constants.ElevatorConstants.PIVOT_CURRENT_LIMIT;
    config.CurrentLimits.StatorCurrentLimitEnable =
        Constants.ElevatorConstants.PIVOT_CURRENT_LIMIT_ENABLED;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.Feedback.FeedbackSensorSource = FeedbackSensorSourceValue.RotorSensor;
    leader = new TalonFX(leadID);
    follower = new TalonFX(followID);
    pigeon = new Pigeon2(gyroID);

    pigeon.getConfigurator().apply(new Pigeon2Configuration());

    leader.getConfigurator().apply(config);

    follower.setControl(new Follower(leadID, false));

    pivotPosition = leader.getPosition();
    pivotVelocity = leader.getVelocity();
    appliedVolts = leader.getMotorVoltage();
    currentAmps = leader.getStatorCurrent();

    pitch = pigeon.getRoll();

    positionSetpoint = Constants.ElevatorConstants.PIVOT_STOW;

    pigeon.optimizeBusUtilization();
    leader.optimizeBusUtilization();
    follower.optimizeBusUtilization();

    BaseStatusSignal.setUpdateFrequencyForAll(
        100, pivotPosition, pivotVelocity, appliedVolts, currentAmps);
  }

  @Override
  public void updateInputs(ElevatorPivotIOInputs inputs) {
    BaseStatusSignal.refreshAll(pivotPosition, pivotVelocity, appliedVolts, currentAmps, pitch);
    inputs.gyroConnected = BaseStatusSignal.refreshAll(pitch).equals(StatusCode.OK);
    inputs.pitch = pitch.getValueAsDouble();
    inputs.pivotPosition = Units.rotationsToDegrees(pivotPosition.getValueAsDouble());
    inputs.pivotVelocity =
        Units.rotationsPerMinuteToRadiansPerSecond(pivotVelocity.getValueAsDouble());
    inputs.appliedVolts = appliedVolts.getValueAsDouble();
    inputs.currentAmps = currentAmps.getValueAsDouble();
  }

  @Override
  public void setPositionSetpoint(double position, double ffVolts) {
    this.positionSetpoint = position;
    leader.setControl(new PositionVoltage(position, 0, false, ffVolts, 0, false, false, false));
  }

  @Override
  public void stop() {
    this.positionSetpoint = pivotPosition.getValueAsDouble();
    leader.stopMotor();
  }

  @Override
  public void configurePID(double kP, double kI, double kD) {
    Slot0Configs config = new Slot0Configs();

    config.kP = kP;
    config.kI = kI;
    config.kD = kD;

    leader.getConfigurator().apply(config);
  }
}
