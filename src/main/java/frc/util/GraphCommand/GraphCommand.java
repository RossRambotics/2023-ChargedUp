// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util.GraphCommand;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;

import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;

public class GraphCommand extends CommandBase {

    private GraphCommandNode m_rootNode = null;
    private GraphCommandNode m_currentNode = null;
    private GraphCommandNode m_previousNode = null;
    private GraphCommandNode m_targetNode = null;
    private boolean m_isTransitioning = false;
    private boolean m_isInitialized = false;

    private Command m_command = null;

    /** Creates a new GraphCommand. */
    public GraphCommand() {
        // Use addRequirements() here to declare subsystem dependencies.
    }

    /* The graph structure is cached during the initialize */
    @Override
    public void initialize() {
        // get a list of all of the reachable nodes
        if (m_rootNode == null) {
            System.err.println("Error: Root Node cannot be null");
            return;
        }

        if (m_isInitialized) {
            DataLogManager.log("GraphCommand initialization again.  Skipping optimize.");
            return;
        } else {
            m_isInitialized = true;
        }

        Map<String, GraphCommandNode> m_nodes = new HashMap<>();
        m_rootNode.getNodeMap(m_nodes);

        // initialize all of the nodes in the graph
        Iterator<String> iterator = m_nodes.keySet().iterator();
        while (iterator.hasNext()) {
            String nodeName = iterator.next();
            GraphCommandNode node = m_nodes.get(nodeName);

            DataLogManager.log("Optimizing Node: " + node.m_nodeName);
            node.optimizeGraph(node, null, null, 0);
            node.printNeighbors();
            DataLogManager.log(" ");
            node.printLinks();
            DataLogManager.log(" ");

        }
    }

    public boolean isTransitioning() {
        return m_isTransitioning;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // if graph is transitioning state, skip
        if (m_isTransitioning) {
            if (m_command.isScheduled()) {

                if (Robot.isSimulation()) {
                    m_command.cancel();
                }

                return;
            } else {
                // just finished running
                m_isTransitioning = false;
                m_command = m_currentNode.m_arrivedCommand;

                DataLogManager.log("GraphCommand arrived: " + m_currentNode.getNodeName());

                if (m_command != null) {
                    m_command.schedule();
                    m_command = null;
                }
            }
        }

        if (m_currentNode == null) {
            System.err.println("Error: Current Node cannot be null.  Did you set a root node?");
            return;
        }

        // if graph is at target node, skip
        if (m_currentNode.equals(m_targetNode)) {
            return;
        }

        // otherwise find next node
        GraphCommandNode node = m_currentNode.getNextNodeGivenTarget(m_targetNode);

        // can't get to target
        if (node == null) {
            return;
        }
        // update state
        m_isTransitioning = true;
        m_previousNode = m_currentNode;
        m_currentNode = node;

        // determine if waypoint or target
        if (node.equals(m_targetNode)) {
            // target node
            m_command = node.m_targetCommand;
            if (m_command == null) {
                m_isTransitioning = false;
                return;
            }
        } else {
            // waypoint
            m_command = node.m_waypointCommand;
            if (m_command == null) {
                m_command = node.m_targetCommand;
            }
            if (m_command == null) {
                m_isTransitioning = false;
                return;
            }
        }
        m_command.schedule();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }

    public void setCurrentNode(GraphCommandNode node) {
        m_currentNode = node;
    }

    public GraphCommandNode getCurrentNode() {
        return m_currentNode;
    }

    public void setTargetNode(GraphCommandNode node) {
        // if the graph isn't transitioning set the next node and move on
        if (!m_isTransitioning) {
            m_targetNode = node;
            return;
        }

        // if we are already going there...
        if (m_targetNode.equals(node)) {
            return;
        }

        // see if the next node is already where we are going
        GraphCommandNode n = m_previousNode.getNextNodeGivenTarget(node);

        // this can be null if the new target node is the previous node
        if (n == null) {
            if (node.equals(m_previousNode)) {
                n = m_previousNode;
            } else {
                return;
            }
        }

        if (n.equals(m_currentNode)) {
            if (m_currentNode == node) {
                // doing this to get the commands recreated using waypoints, arrived, etc.
                // correctly.
                // in case the currentNode is the targetNode
                m_currentNode = m_previousNode;
                m_isTransitioning = false;
                m_command.cancel();
                m_targetNode = node;
            } else {
                // we can just keep going
                m_targetNode = node;
                return;
            }
        } else {
            // see if we can go back to previous node
            n = m_currentNode.getNextNodeGivenTarget(m_previousNode);
            if (n.equals(m_previousNode)) {
                // since the previous node will be the next node we can do this...
                m_isTransitioning = false;
                m_command.cancel();
                m_targetNode = node;
            } else {
                // figure out this situation!
                assert (true) : "Should never get here.";
                return;
            }
        }

    }

    /**
     * Set's the root of the graph. All there must be a way to get from the root
     * node to every other node in the graph. It can take zero, one or more
     * waypoints to reach each node.
     * 
     * @param node The node to set as the root of the graph.
     */
    public void setGraphRootNode(GraphCommandNode node) {
        m_rootNode = node;
        m_currentNode = node;
        m_targetNode = node;
    }

    public class GraphCommandNode {
        private Map<String, GraphCommandNodeLink> m_neighborLinks = new HashMap<>();
        private Map<String, GraphCommandNodeLink> m_optimizedLinks = null;
        private String m_nodeName;
        private Command m_targetCommand;
        private Command m_waypointCommand;
        private Command m_arrivedCommand;
        private GraphCommandNode m_nextNode = null;

        /**
         * 
         * 
         * @param nodeName        - the name of the node
         * @param targetCommand   - the command to run if the node is the target
         * @param waypointCommand - the command to run if the node is a waypoint (not
         *                        the final target)
         * @param arrivedCommand  - the command to run when the graph is at the node and
         *                        the node was the target
         */
        public GraphCommandNode(String nodeName, Command targetCommand, Command waypointCommand,
                Command arrivedCommand) {
            m_nodeName = nodeName;
            m_targetCommand = targetCommand;
            m_waypointCommand = waypointCommand;
            m_arrivedCommand = arrivedCommand;
        }

        public double getCost(GraphCommandNode target) {
            assert (target != null) : " Target node cannot be null.";
            assert (m_optimizedLinks != null) : " Graph must be optimized first.";

            GraphCommandNodeLink link = m_optimizedLinks.get(target.m_nodeName);

            if (link == null) {
                // there isn't a path
                return Double.MAX_VALUE;
            }

            return link.m_cost;
        }

        public GraphCommandNode getNextNodeGivenTarget(GraphCommandNode node) {
            GraphCommandNodeLink link = m_optimizedLinks.get(node.m_nodeName);

            // cannot get to the node
            if (link == null) {
                return null;
            }

            // alredy at the node
            if (link.m_wayPointNode == null) {
                return null;
            }

            // this is the target node
            DataLogManager.log(
                    "GraphCommand next node for: " + node.getNodeName() + " is " + link.m_wayPointNode.getNodeName());
            return link.m_wayPointNode;
        }

        public void getNodeMap(Map<String, GraphCommandNode> nodes) {
            nodes.put(m_nodeName, this);

            Iterator<String> iterator = m_neighborLinks.keySet().iterator();

            // add all of the child nodes
            while (iterator.hasNext()) {
                String nodeName = iterator.next();
                GraphCommandNodeLink link = m_neighborLinks.get(nodeName);

                if (nodes.get(nodeName) == null) {
                    // node hasn't been added so add it
                    link.m_node.getNodeMap(nodes);
                }
            }
        }

        /**
         * For each attached node add their child nodes and add the cost of the attached
         * node to the child node costs. Parent cost is assume zero so stop seach if
         * parent is reached.
         * 
         */
        public void optimizeGraph(GraphCommandNode rootNode,
                GraphCommandNode parentNode, Map<String, GraphCommandNodeLink> bestLinks, double cost) {

            if (bestLinks == null) {
                bestLinks = new HashMap<>();
            }

            // mark this node as processed
            GraphCommandNodeLink thisLink = new GraphCommandNodeLink();
            thisLink.m_cost = cost;
            thisLink.m_node = this;
            thisLink.m_nodeName = m_nodeName;
            thisLink.m_wayPointNode = parentNode;
            bestLinks.put(m_nodeName, thisLink);

            // for each child node
            // initialize each child node
            Iterator<String> iterator = m_neighborLinks.keySet().iterator();

            while (iterator.hasNext()) {
                String neighborNodeName = iterator.next();
                GraphCommandNodeLink neighborLink = m_neighborLinks.get(neighborNodeName);
                GraphCommandNodeLink bestLink = bestLinks.get(neighborNodeName);

                // check to see if this node has been visited already
                if (bestLink != null) {
                    // if existing path cost is lower skip it
                    if (bestLink.m_cost < neighborLink.m_cost + cost) {
                        continue;
                    }
                }

                // Optimize the neighbor node
                if (rootNode == this) {
                    parentNode = neighborLink.m_node;
                }
                neighborLink.m_node.optimizeGraph(rootNode, parentNode, bestLinks, neighborLink.m_cost + cost);
            }

            // if we were initializing for this node update least cost links
            if (rootNode.m_nodeName.equals(this.m_nodeName)) {
                m_optimizedLinks = new HashMap<>(bestLinks);
            }
        }

        /**
         * Adds a node as a child to a node.
         * 
         * 
         * @param childNode - Node to connect. Node cannot be connected to itself.
         * @param cost      - the cost of using this node as
         *                  an intermediate node. Must be
         *                  positive number.
         * @param isOneWay  - Default value is false and it will add both directions
         *                  with equal cost.
         * @return
         */
        public void AddNode(GraphCommandNode childNode, double cost, boolean isOneWay) {
            // make sure the node isn't a child of itself
            assert (!childNode.equals(this)) : " Node must not be added as child of itself.";
            assert (cost >= 0.0) : " Cost must not be negative.";

            // this --> child
            GraphCommandNodeLink link = new GraphCommandNodeLink();
            link.m_nodeName = childNode.m_nodeName;
            link.m_node = childNode;
            link.m_cost = cost;
            m_neighborLinks.put(link.m_nodeName, link);

            // child --> this
            if (!isOneWay) {
                GraphCommandNodeLink link2 = new GraphCommandNodeLink();
                link2.m_nodeName = m_nodeName;
                link2.m_node = this;
                link2.m_cost = cost;
                childNode.m_neighborLinks.put(link2.m_nodeName, link2);
            }
        }

        /**
         * Adds a node as a child to a node.
         * 
         * 
         * @param childNode - Node to connect. Node cannot be connected to itself. This
         *                  method creates a bi-directional link with equal cost.
         * @param cost      - the cost of using this node as
         *                  an intermediate node. Must be
         *                  positive number.
         * @return
         */
        public void AddNode(GraphCommandNode childNode, double cost) {
            this.AddNode(childNode, cost, false);
        }

        public void printNeighbors() {
            Iterator<String> iterator = m_neighborLinks.keySet().iterator();

            // add all of the child nodes
            while (iterator.hasNext()) {
                String nodeName = iterator.next();
                GraphCommandNodeLink link = m_neighborLinks.get(nodeName);

                DataLogManager.log("Neighbor: " + nodeName + " Cost: " + link.m_cost);
            }
        }

        public void printLinks() {
            if (m_optimizedLinks == null) {
                this.optimizeGraph(this, null, null, 0);
            }

            Iterator<String> iterator = m_optimizedLinks.keySet().iterator();

            // add all of the child nodes
            while (iterator.hasNext()) {
                String nodeName = iterator.next();
                GraphCommandNodeLink link = m_optimizedLinks.get(nodeName);

                if (link.m_wayPointNode == null) {
                    DataLogManager.log("Link: " + nodeName + " Cost: " + link.m_cost + " Waypoint: null");
                } else {
                    DataLogManager.log("Link: " + nodeName + " Cost: " + link.m_cost + " Waypoint: "
                            + link.m_wayPointNode.m_nodeName);
                }

            }
        }

        private class GraphCommandNodeLink {
            public String m_nodeName = null;
            public GraphCommandNode m_node = null;
            public GraphCommandNode m_wayPointNode = null;
            public Double m_cost = null;

            public GraphCommandNodeLink() {
            }

            public GraphCommandNodeLink(GraphCommandNodeLink link) {
                m_nodeName = link.m_nodeName;
                m_node = link.m_node;
                m_wayPointNode = link.m_wayPointNode;
                m_cost = link.m_cost;
            }
        }

        public String getNodeName() {
            return m_nodeName;
        }

        public void setNextNode(GraphCommandNode node) {
            DataLogManager.log("GraphCommand setNextNode: " + node.getNodeName());
            m_nextNode = node;
        }

        public GraphCommandNode getNextNode() {
            return m_nextNode;
        }
    }

}
