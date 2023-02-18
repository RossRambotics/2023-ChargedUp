// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderFaults;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxAbsoluteEncoder;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** A robot arm subsystem that moves with a motion profile. */
public class UpperArm extends ProfiledPIDSubsystem {
  private final CANSparkMax m_motor = new CANSparkMax(Constants.kMotorPort, MotorType.kBrushless);
  // private final WPI_CANCoder m_encoder = new
  // WPI_CANCoder(Constants.kEncoderPort);
  private final RelativeEncoder m_encoder = m_motor.getEncoder();

  private final ArmFeedforward m_feedforward = new ArmFeedforward(
      Constants.kSVolts, Constants.kGVolts,
      Constants.kVVoltSecondPerRad, Constants.kAVoltSecondSquaredPerRad);

  /** Create a new ArmSubsystem. */
  public UpperArm() {
    super(
        new ProfiledPIDController(
            Constants.kP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                Constants.kMaxVelocityRadPerSecond,
                Constants.kMaxAccelerationRadPerSecSquared)),
        0);

    // Start arm at rest in neutral position
    setGoal(Constants.kArmOffsetRads);

    // do all of the CANCoder initialization stuff
    CANCoderConfiguration config = new CANCoderConfiguration();

    // set units of the CANCoder to radians, with velocity being radians per second
    config.sensorCoefficient = 2 * Math.PI / 4096.0;
    config.unitString = "rad";
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    // m_encoder.configAllSettings(config);

    m_encoder.setPositionConversionFactor(2 * Math.PI); // change the encoder to radians
    m_motor.setInverted(true);
    m_encoder.setPosition(Math.toRadians(-95.0));

    System.out.println("Upper Arm Position: " + m_encoder.getPosition()); // prints the position of the CANCoder
    System.out.println("Upper Arm absolute Position: " + m_encoder.getPosition());

    // System.out.println("Upper Arm absolute Position: " +
    // m_encoder.getAbsolutePosition());

    // ErrorCode error = m_encoder.getLastError(); // gets the last error generated
    // by the CANCoder
    // CANCoderFaults faults = new CANCoderFaults();
    // ErrorCode faultsError = m_encoder.getFaults(faults); // fills faults with the
    // current CANCoder faults; returns the
    // last error generated

    // m_encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10); //
    // changes the period of the sensor data frame
    // to 10ms

    // do all of the SparkMax initialization stuff

  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(setpoint.position, setpoint.velocity);
    // Add the feedforward to the PID output to get the motor output

    double volts = MathUtil.clamp(output + feedforward, -2.0, 2.0);
    m_motor.setVoltage(volts);

    DataLogManager.log("Upper Arm volts: " + volts + " output: " + output + " FF: " + feedforward + " Measurement: "
        + getMeasurement() + " Goal: "
        + this.m_controller.getGoal().position);
  }

  @Override
  public double getMeasurement() {
    return m_encoder.getPosition() + Constants.kArmOffsetRads;
  }

  public final class Constants {
    public static final int kMotorPort = 60;

    public static final double kP = 1;

    public static final double kSVolts = 1;
    public static final double kGVolts = 1;
    public static final double kVVoltSecondPerRad = 0.5;
    public static final double kAVoltSecondSquaredPerRad = 0.1;

    public static final double kMaxVelocityRadPerSecond = 1;
    public static final double kMaxAccelerationRadPerSecSquared = 1;

    public static final int kEncoderPort = 99;

    // The offset of the arm from the horizontal in its neutral position,
    // measured from the horizontal
    public static final double kArmOffsetRads = -1.65;
  }

  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("Upper Arm Goal", this.m_controller.getGoal().position);
    SmartDashboard.putNumber("Upper Arm Encoder", m_encoder.getPosition());

  }
}
