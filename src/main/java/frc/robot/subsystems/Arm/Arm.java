// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.Arm;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.Arm.LowerArmSetPoint;
import frc.robot.commands.Arm.UpperArmSetPoint;
import frc.util.GraphCommand.GraphCommand;
import frc.util.GraphCommand.GraphCommand.GraphCommandNode;

public class Arm extends SubsystemBase {
  private GraphCommand m_graphCommand = new GraphCommand();

  GraphCommandNode A = m_graphCommand.new GraphCommandNode("A", new PrintCommand("Targeting A"),
      new PrintCommand("Waypointing A"), new PrintCommand("Arrvided A"));
  GraphCommandNode B = m_graphCommand.new GraphCommandNode("B",
      new PrintCommand("Targeting B"),
      new SequentialCommandGroup(new PrintCommand("Waypoint B and waiting... "),
          new WaitCommand(10.0),
          new PrintCommand("Finished waiting.")),
      new PrintCommand("Arrived @ B"));
  GraphCommandNode C = m_graphCommand.new GraphCommandNode("C", new PrintCommand("Targeting C"),
      new PrintCommand("Waypointing C"), new PrintCommand("Arrvided C"));
  GraphCommandNode D = m_graphCommand.new GraphCommandNode("D", new PrintCommand("Targeting D"),
      new PrintCommand("Waypointing D"), new PrintCommand("Arrvided D"));
  GraphCommandNode E = m_graphCommand.new GraphCommandNode("E", new PrintCommand("Targeting E"),
      new PrintCommand("Waypointing E"), new PrintCommand("Arrvided E"));
  GraphCommandNode F = m_graphCommand.new GraphCommandNode("F", new PrintCommand("Targeting F"),
      new PrintCommand("Waypointing F"), new PrintCommand("Arrvided F"));
  GraphCommandNode G = m_graphCommand.new GraphCommandNode("G", new PrintCommand("Targeting G"),
      null, new PrintCommand("Arrived G"));
  GraphCommandNode H = m_graphCommand.new GraphCommandNode("H", new PrintCommand("Targeting H"), null, null);
  GraphCommandNode Q = m_graphCommand.new GraphCommandNode("Q", new PrintCommand("Targeting Q"),
      null, new PrintCommand("Arrived Q"));

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
    A.AddNode(Q, 1.0);

    m_graphCommand.addRequirements(this);
    this.setDefaultCommand(m_graphCommand);
    m_graphCommand.setCurrentNode(A);
    // m_graphCommand.initialize();

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
  final static public Command setpointCommandFactory(String name, double upperArmDegrees, double lowerArmDegrees,
      double toleranceDegrees) {
    Command c = new ParallelCommandGroup(
        new UpperArmSetPoint(Units.degreesToRadians(upperArmDegrees), Units.degreesToRadians((toleranceDegrees))),
        new LowerArmSetPoint(Units.degreesToRadians(lowerArmDegrees), Units.degreesToRadians(toleranceDegrees)));

    c.setName(name + "(" + upperArmDegrees + " , " + lowerArmDegrees + ")");

    return c;
  }

  private Timer m_testTimer = new Timer();

  private boolean m_isFirstTime = true;

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (DriverStation.isDisabled()) {
      return;
    }

    if (m_isFirstTime) {
      m_graphCommand.setTargetNode(F);
      m_isFirstTime = false;
      m_testTimer.start();
    } else {
      if (m_testTimer.advanceIfElapsed(1.0)) {
        m_graphCommand.setTargetNode(H);
      }
    }
  }
}
