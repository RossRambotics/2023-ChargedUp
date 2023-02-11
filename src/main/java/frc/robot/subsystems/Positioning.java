// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.networktables.DoubleArraySubscriber;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.LimelightHelpers;
import frc.robot.Robot;
import frc.robot.RobotContainer;

public class Positioning extends SubsystemBase {
  private Timer m_timer = new Timer();

  /** Creates a new Positioning. */
  public Positioning() {

    m_timer.start();
  }

  public void updateVision(SwerveDrivePoseEstimator odometry) {




    if (!m_timer.advanceIfElapsed(0.02)) {
      DataLogManager.log("Skipping vision update...");
      return;
    }

    if (Robot.isSimulation()) {
      Pose2d botPose = new Pose2d(14.0, 2.0, new Rotation2d());

      try {
        odometry.addVisionMeasurement(botPose, Timer.getFPGATimestamp());
      } catch (Exception e) {
        DataLogManager.log("Vision Measurement Error: " + e.getClass());
      }
      return;
    }

    // check that we have a valid potpose
    if (LimelightHelpers.getTV("") == 1.0) {
      Pose2d botPose = LimelightHelpers.getBotPose2d_wpiBlue("");
      // Pose2d botPose = new Pose2d(14.0, 2.0, new Rotation2d());

      Translation2d odometry_xy = odometry.getEstimatedPosition().getTranslation();
      Translation2d vision_xy = botPose.getTranslation();

      if (LimelightHelpers.getFiducialID("") == 0.0) {
        return;
      }

      if (vision_xy.getDistance(new Translation2d()) == 0.0) {
        System.out.println("Zero botpose.");
        return;
      }

      // if the poses is more than 1.0m different
      // set the post rather than use add measurement
      double distance = odometry_xy.getDistance(vision_xy);
      if (distance > 1.0) {
        DataLogManager.log("Odometry & Vision mismatch.  Updating pose in odometry from Vision. Distance: " + distance);
        //botPose = new Pose2d(botPose.getTranslation(), odometry.getEstimatedPosition().getRotation());
        RobotContainer.m_drivetrainSubsystem.setOdometryPose(botPose);
        System.out.println("Botpose: " + botPose);
        return;
      }

      try {
        //odometry.addVisionMeasurement(botPose, Timer.getFPGATimestamp());
        //odometry.addVisionMeasurement(botPose, Timer.getFPGATimestamp() - LimelightHelpers.getLatency_Pipeline(""));
      } catch (Exception e) {
        DataLogManager.log("Vision Measurement Error: " + e.getClass());
      }
    }
  
  }

  @Override
  public void periodic() {
  }
}
