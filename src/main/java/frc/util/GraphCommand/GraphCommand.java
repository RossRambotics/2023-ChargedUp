// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util.GraphCommand;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.SortedMap;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandBase;

public class GraphCommand extends CommandBase {

  private GraphCommandNode m_rootNode = null;

  /** Creates a new GraphCommand. */
  public GraphCommand() {
    // Use addRequirements() here to declare subsystem dependencies.
  }

  /* The graph structure is cached during the initialize */
  @Override
  public void initialize() {
    // get a list of all of the reachable nodes
    Map<String, GraphCommandNode> m_nodes = new HashMap<>();
    m_rootNode.getNodeMap(m_nodes);

    // TODO call initializeGraph for every node

    Map<String, GraphCommandNode> nodeMap = new HashMap<>();
    m_rootNode.initializeGraph(m_rootNode, nodeMap);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
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

  public void setGraphRootNode(GraphCommandNode node) {
    m_rootNode = node;
  }

  public class GraphCommandNode {
    private Map<String, GraphCommandNodeLink> m_nodes = new HashMap<>();
    private Map<String, GraphCommandNodeLink> m_leastCostLink = null;
    private String m_nodeName;

    /**
     * 
     * 
     * @param nodeName        - the name of the node
     * @param targetCommand   - the command to run if the node is the target
     * @param waypointCommand - the command to run if the node is a waypoint (not
     *                        the final target)
     * @param arrivedCommand  - the command to run when the graph is at the node
     */
    public GraphCommandNode(String nodeName, Command targetCommand, Command waypointCommand, Command arrivedCommand) {
      m_nodeName = nodeName;
    }

    public void getNodeMap(Map<String, GraphCommandNode> nodes) {
      nodes.put(m_nodeName, this);

      Iterator<String> iterator = m_nodes.keySet().iterator();

      // add all of the child nodes
      while (iterator.hasNext()) {
        String nodeName = iterator.next();
        GraphCommandNodeLink link = m_nodes.get(nodeName);

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
     * @param nodeMap - a map of the nodes that have been initialized
     */
    public Map<String, GraphCommandNodeLink> initializeGraph(GraphCommandNode rootNode,
        Map<String, GraphCommandNode> nodeMap) {
      nodeMap.put(m_nodeName, this);

      // copy the map to pre-populate all the child links
      Map<String, GraphCommandNodeLink> tempLinks = new HashMap<>(m_nodes);

      // for each child node
      // initialize each child node
      Iterator<String> iterator = m_nodes.keySet().iterator();

      while (iterator.hasNext()) {
        String nodeName = iterator.next();
        GraphCommandNodeLink link = m_nodes.get(nodeName);

        // if this node has all ready been visited skip it
        if (nodeMap.get(link.m_nodeName) != null) {
          continue;
        }

        // make sure the node isn't a child of itself
        assert (!nodeName.equals(m_nodeName)) : " Node must not be added as child of itself.";

        // otherwise intilize this node
        Map<String, GraphCommandNodeLink> childLinks = link.m_node.initializeGraph(rootNode, nodeMap);

        Iterator<String> childInterator = childLinks.keySet().iterator();
        while (childInterator.hasNext()) {

          // add path replacing if lower cost is found
          String childNodeName = childInterator.next();

          // don't add link to self
          if (childNodeName.equals(m_nodeName)) {
            continue;
          }

          GraphCommandNodeLink childLink = childLinks.get(childNodeName);

          GraphCommandNodeLink n = tempLinks.get(childLink.m_nodeName);
          if (n == null) {
            // doesn't exist so add it
            GraphCommandNodeLink tmpLink = new GraphCommandNodeLink(childLink);
            tmpLink.m_wayPointNode = link.m_node;
            tmpLink.m_cost += link.m_cost;
            tempLinks.put(childLink.m_nodeName, tmpLink);
          } else {
            if (n.m_cost > childLink.m_cost + link.m_cost) {
              // replace link with lower cost link
              GraphCommandNodeLink newLink = new GraphCommandNodeLink(childLink);
              newLink.m_cost += link.m_cost;
              newLink.m_wayPointNode = link.m_node;
              tempLinks.put(n.m_nodeName, newLink);
            }

          }
        }

      }

      // if we were initializing for this node update least cost links
      if (rootNode.m_nodeName.equals(this.m_nodeName)) {
        m_leastCostLink = new HashMap<>(tempLinks);
      }

      return tempLinks;
    }

    /**
     * Adds a node as a child to a node.
     * 
     * 
     * @param childNode - Node to connect. Node cannot be connected to itself.
     * @param cost      - the cost of using this node as
     *                  an intermediate node. Must be
     *                  positive number.
     * @param isOneWay  - Default value is false and it will add both directions.
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
      m_nodes.put(link.m_nodeName, link);

      // child --> this
      if (!isOneWay) {
        GraphCommandNodeLink link2 = new GraphCommandNodeLink();
        link2.m_nodeName = m_nodeName;
        link2.m_node = this;
        link2.m_cost = cost;
        childNode.m_nodes.put(link2.m_nodeName, link2);
      }
    }

    public void AddNode(GraphCommandNode childNode, double cost) {
      this.AddNode(childNode, cost, false);
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
  }

}
