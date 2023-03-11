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
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.WidgetType;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.RobotContainer;
import frc.robot.commands.Arm.LowerArmSetPoint;
import frc.robot.commands.Arm.UpperArmSetPoint;
import frc.robot.commands.Grabber.AutoGrab;
import frc.util.GraphCommand.GraphCommand;
import frc.util.GraphCommand.GraphCommand.GraphCommandNode;

public class Arm extends SubsystemBase {
        private GraphCommand m_graphCommand = new GraphCommand();

        private GenericEntry m_nt_nodeName;
        private GenericEntry m_nt_nextNodeName;
        private GenericEntry m_nt_leftSwitch;
        private GenericEntry m_nt_midSwitch;
        private GenericEntry m_nt_rightSwitch;
        private GenericEntry m_nt_buttonBox;
        private GenericEntry m_nt_dial;

        public GraphCommandNode A, AO;
        public GraphCommandNode B;
        public GraphCommandNode C, D, E, F, G, H, I, J, K, L, M, N, O, P, Q, R, S, T, OO, PP, QQ, NN, Z, X, Y, YY, W;

        /** Creates a new Arm. */
        public Arm() {

        }

        public void initialize() {
                int tolerance = 5;
                int wp_tolerance = 15;

                A = m_graphCommand.new GraphCommandNode("A",
                                Arm.setpointCommandFactory("A Target", -132, 123, tolerance),
                                null,
                                null);
                B = m_graphCommand.new GraphCommandNode("B",
                                Arm.setpointCommandFactory("B Target", 20, 0, tolerance),
                                null,
                                null);
                C = m_graphCommand.new GraphCommandNode("C",
                                Arm.setpointCommandFactory("C Target", 20, -36, tolerance),
                                null,
                                null);
                D = m_graphCommand.new GraphCommandNode("D",
                                Arm.setpointCommandFactory("D Target", -45, 118, tolerance),
                                null,
                                null);
                E = m_graphCommand.new GraphCommandNode("E",
                                Arm.setpointCommandFactory("E Target", -60, 99, tolerance),
                                null,
                                null);
                F = m_graphCommand.new GraphCommandNode("F",
                                Arm.setpointCommandFactory("F Target", 20, 170, tolerance),
                                Arm.setpointCommandFactory("F Waypoint", 20, 170, tolerance),
                                null);
                G = m_graphCommand.new GraphCommandNode("G",
                                Arm.setpointCommandFactory("G Target", 100, 100, tolerance),
                                Arm.setpointCommandFactory("G Waypoint", 100, 100, tolerance),
                                null);
                H = m_graphCommand.new GraphCommandNode("H",
                                Arm.setpointCommandFactory("H Target", 80, 70, tolerance),
                                Arm.setpointCommandFactory("H Waypoint", 80, 70, tolerance),
                                null);
                I = m_graphCommand.new GraphCommandNode("I",
                                Arm.setpointCommandFactory("I Target", 150, 120, tolerance),
                                Arm.setpointCommandFactory("I Waypoint", 150, 120, tolerance),
                                null);
                J = m_graphCommand.new GraphCommandNode("J",
                                Arm.setpointCommandFactory("J Target", 120, 150, tolerance),
                                Arm.setpointCommandFactory("J Waypoint", 120, 150, tolerance),
                                null);
                K = m_graphCommand.new GraphCommandNode("K",
                                Arm.setpointCommandFactory("K Target", 120, 150, tolerance),
                                Arm.setpointCommandFactory("K Waypoint", 120, 150, tolerance),
                                null);
                L = m_graphCommand.new GraphCommandNode("L",
                                Arm.setpointCommandFactory("L Target", -90, 105, tolerance),
                                null,
                                null);
                M = m_graphCommand.new GraphCommandNode("M",
                                Arm.setpointCommandFactory("M Target", 120, 150, tolerance),
                                Arm.setpointCommandFactory("M Waypoint", 120, 150, tolerance),
                                null);
                N = m_graphCommand.new GraphCommandNode("N",
                                Arm.setpointCommandFactory("N Target", -63, 135, tolerance),
                                Arm.setpointCommandFactory("N Target", -63, 135,
                                                wp_tolerance),
                                null);
                O = m_graphCommand.new GraphCommandNode("O",
                                Arm.setpointCommandFactory("Low Arm", -94, 82, tolerance),
                                null,
                                new AutoGrab());

                AO = m_graphCommand.new GraphCommandNode("AO",
                                Arm.setpointCommandFactory("Low Arm", -96, 105, tolerance),
                                null,
                                null);
                P = m_graphCommand.new GraphCommandNode("P",
                                Arm.setpointCommandFactory("P Target", 120, 150, tolerance),
                                Arm.setpointCommandFactory("P Waypoint", 120, 150, tolerance),
                                null);
                Q = m_graphCommand.new GraphCommandNode("Q",
                                Arm.setpointCommandFactory("Q Target", 120, 150, tolerance),
                                Arm.setpointCommandFactory("Q Waypoint", 120, 150, tolerance),
                                null);
                R = m_graphCommand.new GraphCommandNode("R",
                                Arm.setpointCommandFactory("R Target", 120, 150, tolerance),
                                Arm.setpointCommandFactory("R Waypoint", 120, 150, tolerance),
                                null);
                S = m_graphCommand.new GraphCommandNode("S",
                                Arm.setpointCommandFactory("S Target", -127, 105, tolerance),
                                null,
                                null);
                T = m_graphCommand.new GraphCommandNode("T",
                                Arm.setpointCommandFactory("T Target", 120, 150, tolerance),
                                Arm.setpointCommandFactory("T Waypoint", 120, 150, tolerance),
                                null);
                NN = m_graphCommand.new GraphCommandNode("NN",
                                Arm.setpointCommandFactory("NN Target", 120, 150, tolerance),
                                Arm.setpointCommandFactory("NN Waypoint", 120, 150, tolerance),
                                null);
                OO = m_graphCommand.new GraphCommandNode("OO",
                                Arm.setpointCommandFactory("OO Target", 120, 150, tolerance),
                                Arm.setpointCommandFactory("00 Waypoint", 120, 150, tolerance),
                                null);
                PP = m_graphCommand.new GraphCommandNode("PP",
                                Arm.setpointCommandFactory("PP Target", 120, 150, tolerance),
                                Arm.setpointCommandFactory("PP Waypoint", 120, 150, tolerance),
                                null);
                QQ = m_graphCommand.new GraphCommandNode("QQ",
                                Arm.setpointCommandFactory("QQ Target", 120, 150, tolerance),
                                Arm.setpointCommandFactory("QQ Waypoint", 120, 150, tolerance),
                                null);
                W = m_graphCommand.new GraphCommandNode("W",
                                Arm.setpointCommandFactory("W Target", -124.5, 120, tolerance),
                                null,
                                new AutoGrab());
                Z = m_graphCommand.new GraphCommandNode("Z",
                                Arm.setpointCommandFactory("Z Target", -35, 120, tolerance),
                                Arm.setpointCommandFactory("Z Target", -35, 120,
                                                wp_tolerance),
                                null);
                X = m_graphCommand.new GraphCommandNode("X",
                                Arm.setpointCommandFactory("X Target", 0, 0, tolerance),
                                Arm.setpointCommandFactory("X Waypoint", 0, 0, tolerance),
                                null);
                Y = m_graphCommand.new GraphCommandNode("Y",
                                Arm.setpointCommandFactory("Y Target", 0, 0, tolerance),
                                Arm.setpointCommandFactory("Y Waypoint", 0, 0, tolerance),
                                null);
                YY = m_graphCommand.new GraphCommandNode("YY",
                                Arm.setpointCommandFactory("YY Target", -87, 146, tolerance),
                                null,
                                new AutoGrab());

                m_graphCommand.setGraphRootNode(A);

                A.AddNode(S, 1);
                // A.AddNode(NN, 1);
                A.AddNode(N, 1);
                A.AddNode(AO, 1);
                AO.AddNode(O, 1);
                // NN.AddNode(R, 1);
                // R.AddNode(T, 1);
                // R.AddNode(X, 1);
                // X.AddNode(OO, 1);
                // X.AddNode(PP, 1);
                // X.AddNode(QQ, 1);
                N.AddNode(O, 1);
                // N.AddNode(P, 1);
                // N.AddNode(Q, 1);
                N.AddNode(Z, 1);
                Z.AddNode(B, 1);
                B.AddNode(C, 1);
                N.AddNode(D, 1);
                D.AddNode(E, 1);
                YY.AddNode(N, 1);
                YY.AddNode(L, 1);
                A.AddNode(W, 1);

                // N.AddNode(Y, 1);
                C.setNextNode(B);
                B.setNextNode(C);
                D.setNextNode(E);
                E.setNextNode(D);
                // F.setNextNode(G);
                // G.setNextNode(F);
                // H.setNextNode(I);
                // I.setNextNode(H);
                // J.setNextNode(K);
                // K.setNextNode(J);
                // M.setNextNode(L);
                // L.setNextNode(M);
                A.AddNode(YY, 1);

                m_graphCommand.addRequirements(this);
                this.setDefaultCommand(m_graphCommand);
                m_graphCommand.setCurrentNode(S);
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

                RobotContainer.m_GridSelector.setLeftSwitch(m_nt_leftSwitch.getBoolean(false));
                RobotContainer.m_GridSelector.setMidSwitch(m_nt_midSwitch.getBoolean(false));
                RobotContainer.m_GridSelector.setRightSwitch(m_nt_rightSwitch.getBoolean(false));
                RobotContainer.m_GridSelector.setButtonBox(m_nt_buttonBox.getBoolean(false));
                RobotContainer.m_GridSelector.setDial(m_nt_dial.getInteger(1));

        }

        public void createShuffleBoardTab() {
                // this.initialize();
                ShuffleboardLayout graph = RobotContainer.m_buttonBoxTab.getLayout("Panel", BuiltInLayouts.kGrid)
                                .withSize(7, 5)
                                .withPosition(2, 0)
                                .withProperties(Map.of("Label position", "HIDDEN", "Number of columns", 7,
                                                " Number of rows", 5));

                ShuffleboardLayout commands = RobotContainer.m_buttonBoxTab.getLayout("Commands", BuiltInLayouts.kList)
                                .withSize(2, 7)
                                .withPosition(0, 0)
                                .withProperties(Map.of("Label position", "HIDDEN", "Number of columns", 7,
                                                " Number of rows", 5));

                m_nt_nodeName = RobotContainer.m_buttonBoxTab.add("Curr Node Name", "<none>")
                                .withPosition(9, 0).getEntry();
                m_nt_nextNodeName = RobotContainer.m_buttonBoxTab.add("Next Node Name", "<none>")
                                .withPosition(9, 1).getEntry();

                CommandBase cmd = Arm.targetNodeCommandFactory(this, N);
                cmd.setName("Forward Store");
                graph.add(cmd).withPosition(1, 3);

                cmd = Arm.targetNodeCommandFactory(this, Z);
                cmd.setName("Forward High");
                graph.add(cmd).withPosition(1, 2);

                cmd = Arm.targetNodeCommandFactory(this, A);
                cmd.setName("Forward Low");
                graph.add(cmd).withPosition(1, 4);

                cmd = Arm.targetNodeCommandFactory(this, T);
                cmd.setName("Trunk");
                graph.add(cmd).withPosition(0, 4);

                // Arm spot X doesn't have a Value
                cmd = Arm.targetNodeCommandFactory(this, X);
                cmd.setName("High Back No");
                graph.add(cmd).withPosition(0, 2);

                cmd = Arm.targetNodeCommandFactory(this, R);
                cmd.setName("Store back");
                graph.add(cmd).withPosition(0, 3);

                cmd = Arm.targetNodeCommandFactory(this, B);
                cmd.setName("3rd Floor");
                graph.add(cmd).withPosition(6, 2);

                cmd = Arm.targetNodeCommandFactory(this, D);
                cmd.setName("2nd Floor");
                graph.add(cmd).withPosition(6, 3);

                cmd = Arm.targetNodeCommandFactory(this, F);
                cmd.setName("1st Floor");
                graph.add(cmd).withPosition(6, 4);

                // arm spot has no value
                cmd = Arm.targetNodeCommandFactory(this, Y);
                cmd.setName("Human NO");
                graph.add(cmd).withPosition(6, 1);

                cmd = new RunCommand(() -> RobotContainer.m_grabber.openJaws());
                cmd.setName("Open");
                graph.add(cmd).withPosition(5, 0);

                cmd = new RunCommand(() -> RobotContainer.m_grabber.closeJaws());
                cmd.setName("Close");
                graph.add(cmd).withPosition(4, 0);

                cmd = new RunCommand(() -> RobotContainer.m_positioning.resetVision());
                cmd.setName("Vision Reset");
                graph.add(cmd).withPosition(1, 0);

                m_nt_leftSwitch = graph.add("Left Switch", false)
                                .withPosition(2, 1)
                                .withWidget(BuiltInWidgets.kToggleButton).getEntry();

                m_nt_midSwitch = graph.add("Mid Switch", false)
                                .withPosition(3, 1)
                                .withWidget(BuiltInWidgets.kToggleButton).getEntry();

                m_nt_rightSwitch = graph.add("Right Switch", false)
                                .withPosition(4, 1)
                                .withWidget(BuiltInWidgets.kToggleButton).getEntry();

                m_nt_buttonBox = graph.add("Button Box", false)
                                .withPosition(0, 0)
                                .withWidget(BuiltInWidgets.kToggleButton)
                                .getEntry();

                m_nt_dial = graph.add("Dial", 1)
                                .withPosition(4, 3)
                                .withWidget(BuiltInWidgets.kTextView)
                                .getEntry();

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

                cmd = new AutoGrab();
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
        final static public CommandBase setpointCommandFactory(String name, double upperArmDegrees,
                        double lowerArmDegrees,
                        double toleranceDegrees) {
                CommandBase c = new ParallelCommandGroup(
                                new UpperArmSetPoint(Units.degreesToRadians(upperArmDegrees),
                                                Units.degreesToRadians((toleranceDegrees))),
                                new LowerArmSetPoint(Units.degreesToRadians(lowerArmDegrees),
                                                Units.degreesToRadians(toleranceDegrees)))
                                .withTimeout(5.0);

                c.setName(name + "(" + upperArmDegrees + " , " + lowerArmDegrees + ")");

                return c;
        }

        final static public CommandBase targetNodeCommandFactory(Arm arm, GraphCommandNode node) {
                CommandBase cmd = Commands.runOnce(() -> arm.m_graphCommand.setTargetNode(node)).withTimeout(2.0);
                cmd.setName(node.getNodeName());

                return cmd;
        }
}