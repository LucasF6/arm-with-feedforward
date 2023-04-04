// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.InterpolatingTreeMap;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import static frc.robot.Constants.Pivot.*;

import java.util.function.DoubleSupplier;

public class PivotSubsystem extends SubsystemBase {
  private CANSparkMax m_masterMotor = new CANSparkMax(MASTER_MOTOR_ID, MotorType.kBrushless);
  private CANSparkMax m_slaveMotor = new CANSparkMax(SLAVE_MOTOR_ID, MotorType.kBrushless);

  private ProfiledPIDController m_pid = new ProfiledPIDController(
      kP, kI, kD, new TrapezoidProfile.Constraints(MAX_VEL, MAX_ACC));
  private WPI_CANCoder m_encoder;

  private DoubleSupplier m_extendPosition;

  /** Creates a new PivotSubsystem. */
  public PivotSubsystem(DoubleSupplier extendPosition) {
    m_masterMotor.restoreFactoryDefaults(); // setting up motors
    m_slaveMotor.restoreFactoryDefaults();

    m_masterMotor.setInverted(true);
    m_slaveMotor.setInverted(true);
    m_slaveMotor.follow(m_masterMotor);
    
    // This CANCoder measures the rotation of the arm before a 9:1 gear ratio. So when it measures
    // 9 rotations, that is 1 full rotation of the arm.
    CANCoderConfiguration config = new CANCoderConfiguration();
    m_encoder = new WPI_CANCoder(CANCODER_ID);
    config.unitString = ("rotations");
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    config.sensorCoefficient = 1.0 / 4096.0;
    m_encoder.configAllSettings(config);

    m_encoder.setPosition(0);
  }

  @Override
  public void periodic() {
    double feedforward, feedback;
    TrapezoidProfile.State state = m_pid.getSetpoint();
    feedback = m_pid.calculate(m_encoder.getPosition());
    InterpolatingTreeMap<Double, Double> feedforwards = new InterpolatingTreeMap<>();
    for (var entry : EXTEND_POSITION_TO_FEEDFORWARD_MAP.entrySet()) {
      feedforwards.put(entry.getKey(), entry.getValue().calculate(state.position, state.velocity));
    }
    feedforward = feedforwards.get(m_extendPosition.getAsDouble());
    m_masterMotor.set(feedforward + feedback);
  }

  /** @returns The measured value of the rotation of the arm from the CANCoder */
  public double getPosition() {
    return m_encoder.getPosition();
  }

  /** @returns The desired rotation of the arm */
  public double getSetpoint() {
    return m_pid.getGoal().position;
  }

  /** Sets the desired rotation of the arm */
  public void setSetpoint(double value){
    m_pid.setGoal(value);
  }

  public Command getDownCommand() {
    return runOnce(() -> setSetpoint(DOWN_SETPOINT));
  }

  public Command getPickUpCommand() {
    return runOnce(() -> setSetpoint(PICK_UP_SETPOINT));
  }
  
  public Command getScoreCommand() {
    return runOnce(() -> setSetpoint(SCORE_SETPOINT));
  }
}
