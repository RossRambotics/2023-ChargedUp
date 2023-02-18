// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import java.util.Map;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.LowerArmSetPoint;
import frc.robot.commands.Arm.UpperArmSetPoint;
import frc.util.GraphCommand.GraphCommand;
import frc.util.GraphCommand.GraphCommand.GraphCommandNode;

public class Arm extends SubsystemBase {
  private GraphCommand m_graphCommand = new GraphCommand();

  GraphCommandNode A;
  GraphCommandNode B;
  GraphCommandNode C;

  /** Creates a new Arm. */
  public Arm() {

  }

  public void initialize() {
    A = m_graphCommand.new GraphCommandNode("A",
        Arm.setpointCommandFactory("A Target", 0, 0, 1),
        Arm.setpointCommandFactory("A Waypoint", 0, 0, 5),
        new PrintCommand("Arrvided A"));
    B = m_graphCommand.new GraphCommandNode("B",
        Arm.setpointCommandFactory("B Target", -45, 10, 1),
        Arm.setpointCommandFactory("B Waypoint", -45, 10, 5),
        new PrintCommand("Arrvided A"));
    C = m_graphCommand.new GraphCommandNode("C",
        Arm.setpointCommandFactory("C Target", -45, 45, 1),
        Arm.setpointCommandFactory("C Waypoint", -45, 45, 5),
        new PrintCommand("Arrvided A"));

    m_graphCommand.setGraphRootNode(A);

    A.AddNode(B, 1);
    B.AddNode(C, 1);

    m_graphCommand.addRequirements(this);
    this.setDefaultCommand(m_graphCommand);
    m_graphCommand.setCurrentNode(A);
    m_graphCommand.initialize();
  }

  private Timer m_testTimer = new Timer();

  private boolean m_isFirstTime = true;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DriverStation.isDisabled()) {
      return;
    }

    // if (m_isFirstTime) {
    // m_graphCommand.setTargetNode();
    // m_isFirstTime = false;
    // m_testTimer.start();
    // } else {
    // if (m_testTimer.advanceIfElapsed(1.0)) {
    // m_graphCommand.setTargetNode(H);
    // }
    // }
  }

  public void createShuffleBoardTab() {
    ShuffleboardLayout commands = RobotContainer.m_buttonBoxTab.getLayout("Commands", BuiltInLayouts.kList)
        .withSize(2, 1)
        .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

    CommandBase cmd = Arm.setpointCommandFactory("Start", -100, 100, 1);
    commands.add(cmd);

    cmd = Arm.setpointCommandFactory("Midpoint", -45, 45, 1);
    commands.add(cmd);

    cmd = Arm.setpointCommandFactory("End", 0, 0, 1);
    commands.add(cmd);

    cmd = Arm.setpointCommandFactory("Pick up", -90, 80, 1);
    commands.add(cmd);

    cmd = Arm.setpointCommandFactory("Carry", -120, 110, 1);
    commands.add(cmd);

    cmd = Arm.setpointCommandFactory("hold high", 0, 45, 1);
    commands.add(cmd);

  }

  /**
   * Create a new command that moves the upper and lower arm to a specific
   * position with a tolerance
   * 
   * @param name             - the name of the command the factory appends the
   *                         angles (upper, lower)
   * @param upperArmDegrees  - the degrees the upper arm should be above (+) /
   *                         below (-) horizontal
   * @param lowerArmDegrees  - the degrees the lower arm should be above (+) /
   *                         below (-) the upper arm
   * @param toleranceDegrees - how close in degrees the command should consider
   *                         the position obtained (zero probably will never
   *                         finish)
   * @return
   */
  final static public CommandBase setpointCommandFactory(String name, double upperArmDegrees, double lowerArmDegrees,
      double toleranceDegrees) {
    CommandBase c = new ParallelCommandGroup(
        new UpperArmSetPoint(Units.degreesToRadians(upperArmDegrees), Units.degreesToRadians((toleranceDegrees))),
        new LowerArmSetPoint(Units.degreesToRadians(lowerArmDegrees), Units.degreesToRadians(toleranceDegrees)));

    c.setName(name + "(" + upperArmDegrees + " , " + lowerArmDegrees + ")");

    return c;
  }
}
