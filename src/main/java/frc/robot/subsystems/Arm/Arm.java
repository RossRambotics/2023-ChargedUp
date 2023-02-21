// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import java.security.spec.ECParameterSpec;
import java.util.Map;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.GenericEntry;
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
  private GraphCommandNode m_nextNode = null;

  private GenericEntry m_nt_nodeName;
  private GenericEntry m_nt_nextNodeName;

  GraphCommandNode A;
  GraphCommandNode B;
  GraphCommandNode C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, OO, PP, QQ, NN;

  /** Creates a new Arm. */
  public Arm() {

  }

  public void initialize() {
    A = m_graphCommand.new GraphCommandNode("A",
        Arm.setpointCommandFactory("A Target", 0, 0, 1),
        Arm.setpointCommandFactory("A Waypoint", 0, 0, 5),
        Commands.runOnce(() -> m_nextNode = B));
    B = m_graphCommand.new GraphCommandNode("B",
        Arm.setpointCommandFactory("B Target", 90, 90, 1),
        Arm.setpointCommandFactory("B Waypoint", 90, 90, 5),
        Commands.runOnce(() -> m_nextNode = C));
    C = m_graphCommand.new GraphCommandNode("C",
        Arm.setpointCommandFactory("C Target", 30, 110, 1),
        Arm.setpointCommandFactory("C Waypoint", 30, 110, 5),
        Commands.runOnce(() -> m_nextNode = C));
    D = m_graphCommand.new GraphCommandNode("D",
        Arm.setpointCommandFactory("D Target", 20, 50, 1),
        Arm.setpointCommandFactory("D Waypoint", 20, 50, 5),
        Commands.runOnce(() -> m_nextNode = E));
    E = m_graphCommand.new GraphCommandNode("E",
        Arm.setpointCommandFactory("E Target", 50, 140, 1),
        Arm.setpointCommandFactory("E Waypoint", 50, 140, 5),
        Commands.runOnce(() -> m_nextNode = C));
    F = m_graphCommand.new GraphCommandNode("F",
        Arm.setpointCommandFactory("F Target", 20, 170, 1),
        Arm.setpointCommandFactory("F Waypoint", 20, 170, 5),
        Commands.runOnce(() -> m_nextNode = C));
    G = m_graphCommand.new GraphCommandNode("G",
        Arm.setpointCommandFactory("G Target", 100, 100, 1),
        Arm.setpointCommandFactory("G Waypoint", 100, 100, 5),
        Commands.runOnce(() -> m_nextNode = C));
    H = m_graphCommand.new GraphCommandNode("H",
        Arm.setpointCommandFactory("H Target", 80, 70, 1),
        Arm.setpointCommandFactory("H Waypoint", 80, 70, 5),
        Commands.runOnce(() -> m_nextNode = C));
    I = m_graphCommand.new GraphCommandNode("I",
        Arm.setpointCommandFactory("I Target", 150, 120, 1),
        Arm.setpointCommandFactory("I Waypoint", 150, 120, 5),
        Commands.runOnce(() -> m_nextNode = C));
    J = m_graphCommand.new GraphCommandNode("J",
        Arm.setpointCommandFactory("J Target", 120, 150, 1),
        Arm.setpointCommandFactory("J Waypoint", 120, 150, 5),
        Commands.runOnce(() -> m_nextNode = C));
    K = m_graphCommand.new GraphCommandNode("K",
        Arm.setpointCommandFactory("K Target", 120, 150, 1),
        Arm.setpointCommandFactory("K Waypoint", 120, 150, 5),
        Commands.runOnce(() -> m_nextNode = C));
    L = m_graphCommand.new GraphCommandNode("L",
        Arm.setpointCommandFactory("L Target", 120, 150, 1),
        Arm.setpointCommandFactory("L Waypoint", 120, 150, 5),
        Commands.runOnce(() -> m_nextNode = C));
    M = m_graphCommand.new GraphCommandNode("M",
        Arm.setpointCommandFactory("M Target", 120, 150, 1),
        Arm.setpointCommandFactory("M Waypoint", 120, 150, 5),
        Commands.runOnce(() -> m_nextNode = C));
    N = m_graphCommand.new GraphCommandNode("N",
        Arm.setpointCommandFactory("N Target", 120, 150, 1),
        Arm.setpointCommandFactory("N Waypoint", 120, 150, 5),
        Commands.runOnce(() -> m_nextNode = C));
    O = m_graphCommand.new GraphCommandNode("O",
        Arm.setpointCommandFactory("O Target", 120, 150, 1),
        Arm.setpointCommandFactory("O Waypoint", 120, 150, 5),
        Commands.runOnce(() -> m_nextNode = C));
    P = m_graphCommand.new GraphCommandNode("P",
        Arm.setpointCommandFactory("P Target", 120, 150, 1),
        Arm.setpointCommandFactory("P Waypoint", 120, 150, 5),
        Commands.runOnce(() -> m_nextNode = C));
    Q = m_graphCommand.new GraphCommandNode("Q",
        Arm.setpointCommandFactory("Q Target", 120, 150, 1),
        Arm.setpointCommandFactory("Q Waypoint", 120, 150, 5),
        Commands.runOnce(() -> m_nextNode = C));
    R = m_graphCommand.new GraphCommandNode("R",
        Arm.setpointCommandFactory("R Target", 120, 150, 1),
        Arm.setpointCommandFactory("R Waypoint", 120, 150, 5),
        Commands.runOnce(() -> m_nextNode = C));
    S = m_graphCommand.new GraphCommandNode("S",
        Arm.setpointCommandFactory("S Target", 120, 150, 1),
        Arm.setpointCommandFactory("S Waypoint", 120, 150, 5),
        Commands.runOnce(() -> m_nextNode = C));
    T = m_graphCommand.new GraphCommandNode("T",
        Arm.setpointCommandFactory("T Target", 120, 150, 1),
        Arm.setpointCommandFactory("T Waypoint", 120, 150, 5),
        Commands.runOnce(() -> m_nextNode = C));
    NN = m_graphCommand.new GraphCommandNode("NN",
        Arm.setpointCommandFactory("NN Target", 120, 150, 1),
        Arm.setpointCommandFactory("NN Waypoint", 120, 150, 5),
        Commands.runOnce(() -> m_nextNode = C));
    OO = m_graphCommand.new GraphCommandNode("OO",
        Arm.setpointCommandFactory("OO Target", 120, 150, 1),
        Arm.setpointCommandFactory("00 Waypoint", 120, 150, 5),
        Commands.runOnce(() -> m_nextNode = C));
    PP = m_graphCommand.new GraphCommandNode("PP",
        Arm.setpointCommandFactory("PP Target", 120, 150, 1),
        Arm.setpointCommandFactory("PP Waypoint", 120, 150, 5),
        Commands.runOnce(() -> m_nextNode = C));
    QQ = m_graphCommand.new GraphCommandNode("QQ",
        Arm.setpointCommandFactory("QQ Target", 120, 150, 1),
        Arm.setpointCommandFactory("QQ Waypoint", 120, 150, 5),
        Commands.runOnce(() -> m_nextNode = C));
    // A = m_graphCommand.new GraphCommandNode("A",
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

    A.AddNode(S, 1);
    A.AddNode(NN, 1);
    A.AddNode(N, 1);
    NN.AddNode(R, 1);
    R.AddNode(T, 1);
    R.AddNode(OO, 1);
    R.AddNode(PP, 1);
    R.AddNode(QQ, 1);
    N.AddNode(O, 1);
    N.AddNode(P, 1);
    N.AddNode(Q, 1);
    N.AddNode(B, 1);
    B.AddNode(C, 1);
    N.AddNode(D, 1);
    D.AddNode(E, 1);
    N.AddNode(F, 1);
    F.AddNode(G, 1);
    N.AddNode(H, 1);
    H.AddNode(I, 1);
    N.AddNode(J, 1);
    J.AddNode(K, 1);
    N.AddNode(L, 1);
    L.AddNode(M, 1);

    m_graphCommand.addRequirements(this);
    this.setDefaultCommand(m_graphCommand);
    m_graphCommand.setCurrentNode(A);
    m_graphCommand.initialize();
    m_nextNode = B;
  }

  public void goNextNode() {
    if (m_nextNode == null) {
      return;
    }

    this.m_graphCommand.setTargetNode(m_nextNode);
    m_nextNode = null;
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

    if (m_nextNode == null) {
      m_nt_nextNodeName.setString("<none>");
    } else {
      m_nt_nextNodeName.setString(m_nextNode.getNodeName());
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

    cmd = Arm.targetNodeCommandFactory(this, B);
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
        new LowerArmSetPoint(Units.degreesToRadians(lowerArmDegrees), Units.degreesToRadians(toleranceDegrees)));

    c.setName(name + "(" + upperArmDegrees + " , " + lowerArmDegrees + ")");

    return c;
  }

  final static public CommandBase targetNodeCommandFactory(Arm arm, GraphCommandNode node) {
    CommandBase cmd = Commands.runOnce(() -> arm.m_graphCommand.setTargetNode(node));
    cmd.setName(node.getNodeName());

    return cmd;
  }
}
