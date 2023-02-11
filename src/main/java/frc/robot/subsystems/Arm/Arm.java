// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.util.GraphCommand.GraphCommand;
import frc.util.GraphCommand.GraphCommand.GraphCommandNode;

public class Arm extends SubsystemBase {
  private GraphCommand m_graphCommand = new GraphCommand();

  GraphCommandNode A = m_graphCommand.new GraphCommandNode("A", null, null, null);
  GraphCommandNode B = m_graphCommand.new GraphCommandNode("B", new PrintCommand("Targeting B"),
      new PrintCommand("Waypoint B"), new PrintCommand("Arrived @ B"));
  GraphCommandNode C = m_graphCommand.new GraphCommandNode("C", null, null, null);
  GraphCommandNode D = m_graphCommand.new GraphCommandNode("D", null, null, null);
  GraphCommandNode E = m_graphCommand.new GraphCommandNode("E", null, null, null);
  GraphCommandNode F = m_graphCommand.new GraphCommandNode("F", null, null, null);
  GraphCommandNode G = m_graphCommand.new GraphCommandNode("G", new PrintCommand("Targeting G"),
      null, new PrintCommand("Arrived G"));
  GraphCommandNode H = m_graphCommand.new GraphCommandNode("H", new PrintCommand("Targeting G"), null, null);

  /** Creates a new Arm. */
  public Arm() {

    m_graphCommand.setGraphRootNode(A);

    A.AddNode(B, 1.0);
    B.AddNode(C, 1.0);
    B.AddNode(D, 1.0);
    B.AddNode(G, 1.0);
    B.AddNode(H, 6.0);
    D.AddNode(E, 1.0);
    D.AddNode(F, 1.0);
    G.AddNode(H, 1.0);

    m_graphCommand.addRequirements(this);
    this.setDefaultCommand(m_graphCommand);

    // m_graphCommand.initialize();

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    m_graphCommand.setTargetNode(H);
  }
}
