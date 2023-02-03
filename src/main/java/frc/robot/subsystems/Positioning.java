// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Positioning extends SubsystemBase {
  private NetworkTable m_limelight = null;
  private DoubleArraySubscriber camtransub;
  private DoubleArraySubscriber posesub;
  private Translation2d tran2d;
  private Rotation2d r2d;
  private Pose2d p2d;
  private NetworkTable m_table;
  private String m_tableName;

  NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
  int tv = m_limelight.getEntry("tv").getNumber(0).intValue();
  int botpose_wpiblue = m_limelight.getEntry("botpose_wpiblue").getNumber(0).intValue();

  
  
 
  Pose2d m_botpose = null;

  



  /** Creates a new Positioning. */
  public Positioning() {

    m_tableName = "limelight";
    m_table = NetworkTableInstance.getDefault().getTable(m_tableName);
    posesub = m_table.getDoubleArrayTopic("botpose").subscribe(new double[] {});

    m_limelight = NetworkTableInstance.getDefault().getTable("limelight-rambot");

    
    double m_bluepose[];
    m_bluepose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[6]);

    Pose2d m_botpose = new Pose2d(m_bluepose[0], m_bluepose[1], null);
    

  }

  public Pose2d getRobotPose() {

    System.out.println("Geting Robot Pose");

    double[] result = posesub.get();
    tran2d = new Translation2d(result[0], result[1]);
    r2d = new Rotation2d(result[3]);
    p2d = new Pose2d(tran2d, r2d);

    return p2d;
}

  public void updateVision(SwerveDrivePoseEstimator odometry) {


    // double bluepose[];
    // bluepose = NetworkTableInstance.getDefault().getTable("limelight").getEntry("botpose_wpiblue").getDoubleArray(new double[6]);

    // Pose2d botpose = new Pose2d(bluepose[0], bluepose[1], null);
    
    if (tv == 1) {
      odometry.addVisionMeasurement(getRobotPose(), 1);
      
    }
  }

  @Override
  public void periodic() {
        // This method will be called once per scheduler run

        // check that we have a valid potpose
        if (tv == 1) {

          DrivetrainSubsystem.getOdometry().addVisionMeasurement(getRobotPose(), 1);
        }

        // get the odometry from drive train

        // update the vision estimate
  }
}