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
    private Pose2d m_lastVisionPose = new Pose2d();
    private double m_lasttime = 0;

    /** Creates a new Positioning. */
    public Positioning() {

        m_timer.start();
        LimelightHelpers.setStreamMode_PiPSecondary("");
    }

    public void resetVision() {
        if (LimelightHelpers.getTV("") == false) {
            return;
        }

        Pose2d botPose = LimelightHelpers.getBotPose2d_wpiBlue("");

        // Pose2d botPose = new Pose2d(14.0, 2.0, new Rotation2d());

        DataLogManager.log("Updating pose in odometry from Vision. botPose: " + botPose);
        RobotContainer.m_drivetrainSubsystem.setOdometryPose(botPose);
        return;

    }

    public void updateVision(SwerveDrivePoseEstimator odometry) {

        if (Robot.isSimulation()) {
            Pose2d botPose = new Pose2d(14.0, 2.0, new Rotation2d());

            try {
                // odometry.addVisionMeasurement(botPose, Timer.getFPGATimestamp());
            } catch (Exception e) {
                DataLogManager.log("Vision Measurement Error: " + e.getClass());
            }
            return;
        }

        // check that we have a valid potpose
        if (LimelightHelpers.getTV("") == true) {
            Pose2d botPose = LimelightHelpers.getBotPose2d_wpiBlue("");
            // Pose2d botPose = new Pose2d(14.0, 2.0, new Rotation2d());

            Translation2d odometry_xy = odometry.getEstimatedPosition().getTranslation();
            Translation2d vision_xy = botPose.getTranslation();

            // image too small to tryst
            if (LimelightHelpers.getTA("") < .5) {
                return;
            }

            // no id found
            double fID = LimelightHelpers.getFiducialID("");
            if (fID == 0.0) {
                return;
            }

            // botpose is ZERO!!
            if (vision_xy.getDistance(new Translation2d()) == 0.0) {
                DataLogManager.log("Zero botpose.");
                return;
            }

            m_lastVisionPose = botPose;

            // if the poses is more than 1.0m different
            // skip it
            double distance = odometry_xy.getDistance(vision_xy);
            if (distance > 1.0) {
                // DataLogManager.log("Odometry & Vision mismatch. Skipping update. Distance: "
                // + distance);

                RobotContainer.m_LEDPanel.showVisionStatusRed();

                return;
            } else if (distance <= 1.0 && distance > 0.10) {
                RobotContainer.m_LEDPanel.showVisionStatusYellow();
            } else {
                RobotContainer.m_LEDPanel.showVisionStatusGreen();
            }

            try {
                if (false) {
                    odometry.addVisionMeasurement(botPose,
                            Timer.getFPGATimestamp() - LimelightHelpers.getLatency_Pipeline("") - 10);
                }
            } catch (Exception e) {
                DataLogManager.log("Vision Measurement Error: " + e.getClass());
            }
        }

    }

    public Pose2d getLastBotPose() {
        return m_lastVisionPose;
    }

    @Override
    public void periodic() {
    }
}
