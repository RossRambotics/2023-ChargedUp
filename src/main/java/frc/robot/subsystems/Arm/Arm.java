// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import java.util.Map;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.LowerArmSetPoint;
import frc.robot.commands.Arm.UpperArmSetPoint;
import frc.util.GraphCommand.GraphCommand;
import frc.util.GraphCommand.GraphCommand.GraphCommandNode;

public class Arm extends SubsystemBase {
  private GraphCommand m_graphCommand = new GraphCommand();

  private GenericEntry m_nt_nodeName;
  private GenericEntry m_nt_nextNodeName;

  GraphCommandNode A;
  GraphCommandNode B;
  GraphCommandNode C, D, E, F, G, H, I, J;

  /** Creates a new Arm. */
  public Arm() {

  }

  public void initialize() {
    A = m_graphCommand.new GraphCommandNode("Carry",
        Arm.setpointCommandFactory("A Target", -120,
            110, 5),
        null,
        null);
    B = m_graphCommand.new GraphCommandNode("Safe Forward",
        Arm.setpointCommandFactory("B Target", -45,
            120, 5),
        null,
        null);
    C = m_graphCommand.new GraphCommandNode("Hold High",
        Arm.setpointCommandFactory("C Target", 0,
            45, 5),
        null,
        null);
    D = m_graphCommand.new GraphCommandNode("End",
        Arm.setpointCommandFactory("D Target", 0,
            0, 5),
        null,
        null);
    // E = m_graphCommand.new GraphCommandNode("C",
    // Arm.setpointCommandFactory("C Target", 50, 140, 1),
    // Arm.setpointCommandFactory("C Waypoint", 50, 140, 5),
    // Commands.runOnce(() -> m_nextNode = C));
    // F = m_graphCommand.new GraphCommandNode("C",
    // Arm.setpointCommandFactory("C Target", 20, 170, 1),
    // Arm.setpointCommandFactory("C Waypoint", 20, 170, 5),
    // Commands.runOnce(() -> m_nextNode = C));
    // G = m_graphCommand.new GraphCommandNode("B",
    // Arm.setpointCommandFactory("B Target", 100, 100, 1),
    // Arm.setpointCommandFactory("B Waypoint", 100, 100, 5),
    // Commands.runOnce(() -> m_nextNode = C));
    // H = m_graphCommand.new GraphCommandNode("C",
    // Arm.setpointCommandFactory("C Target", 80, 70, 1),
    // Arm.setpointCommandFactory("C Waypoint", 80, 70, 5),
    // Commands.runOnce(() -> m_nextNode = C));
    // I = m_graphCommand.new GraphCommandNode("B",
    // Arm.setpointCommandFactory("B Target", 150, 120, 1),
    // Arm.setpointCommandFactory("B Waypoint", 150, 120, 5),
    // Commands.runOnce(() -> m_nextNode = C));
    // J = m_graphCommand.new GraphCommandNode("C",
    // Arm.setpointCommandFactory("C Target", 120, 150, 1),
    // Arm.setpointCommandFactory("C Waypoint", 120, 150, 5),
    // Commands.runOnce(() -> m_nextNode = C));
    // // A = m_graphCommand.new GraphCommandNode("A",
    // new PrintCommand("Going to A"),
    // null,
    // Commands.runOnce(() -> m_nextNode = B));
    // B = m_graphCommand.new GraphCommandNode("B",
    // new PrintCommand(
    // "Going to B"),
    // null,
    // Commands.runOnce(() -> m_nextNode = C));
    // C = m_graphCommand.new GraphCommandNode("C",
    // new PrintCommand(
    // "Going to C"),
    // null,
    // Commands.runOnce(() -> m_nextNode = A));

    m_graphCommand.setGraphRootNode(A);

    A.AddNode(B, 1);
    B.AddNode(C, 1);
    C.AddNode(D, 1);
    C.setNextNode(D);
    D.setNextNode(C);

    m_graphCommand.addRequirements(this);
    this.setDefaultCommand(m_graphCommand);
    m_graphCommand.setCurrentNode(A);
    m_graphCommand.initialize();
  }

  public void goNextNode() {
    GraphCommandNode node = m_graphCommand.getCurrentNode();

    if (node == null) {
      return;
    }

    node = node.getNextNode();
    if (node == null) {
      return;
    }

    this.m_graphCommand.setTargetNode(node);
  }

  private Timer m_testTimer = new Timer();

  private boolean m_isFirstTime = true;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DriverStation.isDisabled()) {
      return;
    }

    if (m_graphCommand.getCurrentNode() == null) {
      m_nt_nodeName.setString("<none>");
    } else {
      m_nt_nodeName.setString(m_graphCommand.getCurrentNode().getNodeName());
    }

    if (m_graphCommand.getCurrentNode().getNextNode() == null) {
      m_nt_nextNodeName.setString("<none>");
    } else {
      m_nt_nextNodeName.setString(m_graphCommand.getCurrentNode().getNextNode().getNodeName());
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
    // this.initialize();
    ShuffleboardLayout graph = RobotContainer.m_buttonBoxTab.getLayout("Panel", BuiltInLayouts.kGrid)
        .withSize(7, 5)
        .withPosition(2, 0)
        .withProperties(Map.of("Label position", "HIDDEN", "Number of columns", 7, " Number of rows", 5));

    ShuffleboardLayout commands = RobotContainer.m_buttonBoxTab.getLayout("Commands", BuiltInLayouts.kList)
        .withSize(2, 7)
        .withPosition(0, 0)
        .withProperties(Map.of("Label position", "HIDDEN", "Number of columns", 7, " Number of rows", 5));

    m_nt_nodeName = RobotContainer.m_buttonBoxTab.add("Curr Node Name", "<none>")
        .withPosition(9, 0).getEntry();
    m_nt_nextNodeName = RobotContainer.m_buttonBoxTab.add("Next Node Name", "<none>")
        .withPosition(9, 1).getEntry();

    CommandBase cmd = Arm.targetNodeCommandFactory(this, A);
    graph.add(cmd).withPosition(3, 3);

    cmd = Arm.targetNodeCommandFactory(this, C);
    graph.add(cmd).withPosition(5, 0);

    cmd = Arm.setpointCommandFactory("Start", -100, 100, 1);
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

    cmd = Arm.setpointCommandFactory("safe forward", -45, 120, 1);
    commands.add(cmd);

    cmd = Commands.runOnce(() -> this.goNextNode());
    cmd.setName("Next Node");
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
        new LowerArmSetPoint(Units.degreesToRadians(lowerArmDegrees), Units.degreesToRadians(toleranceDegrees)))
        .withTimeout(2.0);

    c.setName(name + "(" + upperArmDegrees + " , " + lowerArmDegrees + ")");

    return c;
  }

  final static public CommandBase targetNodeCommandFactory(Arm arm, GraphCommandNode node) {
    CommandBase cmd = Commands.runOnce(() -> arm.m_graphCommand.setTargetNode(node)).withTimeout(2.0);
    cmd.setName(node.getNodeName());

    return cmd;
  }
}
