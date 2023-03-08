// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.RobotContainer;

import java.util.Map;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.CANCoderFaults;
import com.ctre.phoenix.sensors.CANCoderStatusFrame;
import com.ctre.phoenix.sensors.SensorInitializationStrategy;
import com.ctre.phoenix.sensors.SensorTimeBase;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

/** A robot arm subsystem that moves with a motion profile. */
public class LowerArm extends ProfiledPIDSubsystem {

  private GenericEntry m_nt_angle_goal;
  private GenericEntry m_nt_angle_goal_test;
  private GenericEntry m_nt_angle_actual;
  private GenericEntry m_nt_volts;
  private GenericEntry m_nt_feed_forward;

  private Boolean m_testMode = false;

  private final CANSparkMax m_motor = new CANSparkMax(Constants.kMotorPort, MotorType.kBrushless);
  private final WPI_CANCoder m_encoder = new WPI_CANCoder(Constants.kEncoderPort);
  private final ArmFeedforward m_feedforward = new ArmFeedforward(
      Constants.kSVolts, Constants.kGVolts,
      Constants.kVVoltSecondPerRad, Constants.kAVoltSecondSquaredPerRad);

  private final Joystick m_joy = new Joystick(5);

  /** Create a new ArmSubsystem. */
  public LowerArm() {
    super(
        new ProfiledPIDController(
            Constants.kP,
            0,
            0,
            new TrapezoidProfile.Constraints(
                Constants.kMaxVelocityRadPerSecond,
                Constants.kMaxAccelerationRadPerSecSquared)),
        0);

    // do all of the CANCoder initialization stuff
    CANCoderConfiguration config = new CANCoderConfiguration();

    // set units of the CANCoder to radians, with velocity being radians per second
    config.sensorCoefficient = 2 * Math.PI / 4096.0;
    config.unitString = "rad";
    config.sensorTimeBase = SensorTimeBase.PerSecond;
    config.initializationStrategy = SensorInitializationStrategy.BootToAbsolutePosition;
    config.sensorDirection = true;
    ErrorCode e = m_encoder.configAllSettings(config, 500);

    // Start arm at rest in neutral position
    setGoal(m_encoder.getPosition());

    System.out.println("Lower Arm Encoder Error Code: " + e);
    System.out.println("Lower Arm Position: " + m_encoder.getPosition()); // prints the position of the CANCoder
    System.out.println("Lower Arm absolute Position: " + m_encoder.getAbsolutePosition());

    ErrorCode error = m_encoder.getLastError(); // gets the last error generated by the CANCoder
    CANCoderFaults faults = new CANCoderFaults();
    ErrorCode faultsError = m_encoder.getFaults(faults); // fills faults with the current CANCoder faults; returns the
                                                         // last error generated

    m_encoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, 10); // changes the period of the sensor data frame
                                                                        // to 10ms

    // do all of the SparkMax initialization stuff
    m_motor.setInverted(true);
    this.setName("Lower Arm");

  }

  private Timer m_testTimer = new Timer();
  private double m_testStartRad = 0.0;
  private double m_testVolts = 0.0;
  private State m_lastSetPoint = new State(0, 0);

  public void kVTestStart(double volts) {
    this.disable();
    m_testVolts = volts;

    m_testStartRad = m_encoder.getPosition();
    m_testTimer.reset();
    m_testTimer.start();
    m_motor.setVoltage(m_testVolts);
  }

  public void kVTestStop() {

    double testEndRad = m_encoder.getPosition();
    m_motor.setVoltage(0);
    m_testTimer.stop();

    double kV = (m_testTimer.get() * m_testVolts) / Math.abs(m_testStartRad - testEndRad);
    System.out.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
    System.out.println("Start Rad: " + m_testStartRad);
    System.out.println("End Rad: " + testEndRad);
    System.out.println("Time: " + m_testTimer.get());
    System.out.println("Volts: " + m_testVolts);
    System.out.println("kV: " + kV);
    System.out.println("!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
  }

  @Override
  public void enable() {
    // do super enable
    super.enable();

    // now do redo reset with the last set point
    // super.getController().reset(m_lastSetPoint);
  }

  @Override
  public void useOutput(double output, TrapezoidProfile.State setpoint) {
    // save last setpoint to blend set points together
    m_lastSetPoint = setpoint;

    // Calculate the feedforward from the sepoint
    // need to use lowerarm setpoint along with lower arm set point
    double s = setpoint.position + RobotContainer.m_upperArm.getMeasurement();
    double feedforward = m_feedforward.calculate(s, setpoint.velocity);
    // feedforward = 0;
    // Add the feedforward to the PID output to get the motor output

    double volts = MathUtil.clamp(output + feedforward, -6.0, 6.0);
    m_nt_feed_forward.setDouble(feedforward);
    m_motor.setVoltage(volts);
    m_nt_volts.setDouble(volts);

    // DataLogManager.log("LA V: " + volts + " output: " + output + " FF: " +
    // feedforward + " Measurement: "
    // + getMeasurement() + " Goal: "
    // + this.m_controller.getGoal().position + " s: " + s);
  }

  @Override
  public double getMeasurement() {
    return m_encoder.getPosition() + Constants.kArmOffsetRads;
  }

  public void periodic() {
    super.periodic();
    SmartDashboard.putNumber("Lower Arm Goal", this.m_controller.getGoal().position);
    SmartDashboard.putNumber("Lower Arm Encoder", m_encoder.getPosition() + Constants.kArmOffsetRads);
    m_nt_angle_goal.setDouble((Math.toDegrees(this.m_controller.getGoal().position)));
    m_nt_angle_actual.setDouble(Math.toDegrees(m_encoder.getPosition() + Constants.kArmOffsetRads));

    if (m_testMode) {
      double goal = m_nt_angle_goal_test.getDouble(0.0);
      goal = Math.toRadians(goal);
      this.setGoal(goal);
      System.out.println("Upper Arm Test Radians: " + goal);
      m_testMode = false;
      this.enable();
    }
  }

  public void createShuffleBoardTab() {
    m_nt_angle_goal_test = RobotContainer.m_armTab.add("L-Arm Test deg", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withSize(4, 1)
        .withPosition(4, 0).withProperties(Map.of("min", -160, "max", 160)).getEntry();

    m_nt_angle_goal = RobotContainer.m_armTab.add("L Arm Goal deg", 0)
        .withSize(1, 1)
        .withPosition(6, 1).getEntry();

    m_nt_angle_actual = RobotContainer.m_armTab.add("L Arm Actual deg", 0)
        .withSize(1, 1)
        .withPosition(4, 1).getEntry();

    m_nt_volts = RobotContainer.m_armTab.add("L Arm Volts", 0)
        .withSize(1, 1)
        .withPosition(5, 1).getEntry();

    m_nt_feed_forward = RobotContainer.m_armTab.add("L Arm FF V", 0)
        .withSize(1, 1)
        .withPosition(5, 2).getEntry();

    ShuffleboardLayout pidLayout = RobotContainer.m_armTab.getLayout("Lower Arm", BuiltInLayouts.kList)
        .withSize(2, 3)
        .withPosition(4, 3)
        .withProperties(Map.of("Label position", "HIDDEN"));
    pidLayout.add(this.getController());

    CommandBase cmd = Commands.runOnce(
        () -> m_testMode = true,
        this);
    cmd.setName("L Arm Test");
    RobotContainer.m_armTab.add(cmd)
        .withSize(1, 1)
        .withPosition(7, 1)
        .withProperties(Map.of("Label position", "HIDDEN"));

    cmd = new FunctionalCommand(() -> this.kVTestStart(2.0), //
        () -> {
        }, //
        (b) -> this.kVTestStop(), //
        () -> {
          return false;
        }) //
        .withTimeout(1.0);
    cmd.setName("kV Test 2V, 1.0s");

    RobotContainer.m_armTab.add(cmd)
        .withSize(1, 1)
        .withPosition(7, 2);

    cmd = new FunctionalCommand(() -> this.kVTestStart(3.0), //
        () -> {
        }, //
        (b) -> this.kVTestStop(), //
        () -> {
          return false;
        }) //
        .withTimeout(1.0);
    cmd.setName("kV Test 3V, 1.0s");

    RobotContainer.m_armTab.add(cmd)
        .withSize(1, 1)
        .withPosition(7, 3);

    cmd = new FunctionalCommand(() -> this.kVTestStart(4.0), //
        () -> {
        }, //
        (b) -> this.kVTestStop(), //
        () -> {
          return false;
        }) //
        .withTimeout(1.0);
    cmd.setName("kV Test 4V, 1.0s");
    RobotContainer.m_armTab.add(cmd)
        .withSize(1, 1)
        .withPosition(7, 4);

  }

  public final class Constants {
    public static final int kMotorPort = 62;

    public static final double kP = 6;

    public static final double kSVolts = 0.01;
    public static final double kGVolts = 0.40;
    public static final double kVVoltSecondPerRad = 3.0;
    public static final double kAVoltSecondSquaredPerRad = 0.1;

    public static final double kMaxVelocityRadPerSecond = 5;
    public static final double kMaxAccelerationRadPerSecSquared = 3;

    public static final int kEncoderPort = 33;

    // The offset of the arm from the horizontal in its neutral position,
    // measured from the horizontal 154.688 degrees to center
    public static final double kArmOffsetRads = 0.61;
  }
}

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// Start Rad: -0.1579997017979622
// End Rad: 0.49854274839162827
// Time: 1.001186999999998
// Volts: 2.0
// kV: 3.0498774289792965
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// Start Rad: -0.6565424501895905
// End Rad: -0.026077620685100555
// Time: 1.000199999999997
// Volts: 2.0
// kV: 3.172897053705909
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// Start Rad: -0.704095758497715
// End Rad: -0.06442706286907196
// Time: 1.0213899999999994
// Volts: 2.0
// kV: 3.193496905444824
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
// Start Rad: -0.6227949410676956
// End Rad: 0.8574935272336006
// Time: 1.0200390000000188
// Volts: 4.0
// kV: 2.7563249240752747
// !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
