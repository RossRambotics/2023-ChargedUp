// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.util.GraphCommand;

import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
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

    // initialize all of the nodes in the graph
    Iterator<String> iterator = m_nodes.keySet().iterator();
    while (iterator.hasNext()) {
      String nodeName = iterator.next();
      GraphCommandNode node = m_nodes.get(nodeName);

      System.out.println("Optimizing Node: " + node.m_nodeName);
      node.optimizeGraph(node, null, null, 0);
      node.printNeighbors();
      System.out.println();
      node.printLinks();
      System.out.println();

    }
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
    private Map<String, GraphCommandNodeLink> m_neighborLinks = new HashMap<>();
    private Map<String, GraphCommandNodeLink> m_optimizedLinks = null;
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

    public void AddNode(GraphCommandNode childNode, double cost) {
      this.AddNode(childNode, cost, false);
    }

    public void printNeighbors() {
      Iterator<String> iterator = m_neighborLinks.keySet().iterator();

      // add all of the child nodes
      while (iterator.hasNext()) {
        String nodeName = iterator.next();
        GraphCommandNodeLink link = m_neighborLinks.get(nodeName);

        System.out.println("Neighbor: " + nodeName + " Cost: " + link.m_cost);
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
          System.out
              .println("Link: " + nodeName + " Cost: " + link.m_cost + " Waypoint: null");
        } else {
          System.out
              .println("Link: " + nodeName + " Cost: " + link.m_cost + " Waypoint: " + link.m_wayPointNode.m_nodeName);
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
  }

}
