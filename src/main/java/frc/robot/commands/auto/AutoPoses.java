// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.auto;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive.SnapDriveToPoseField;

/** Add your docs here. */
public class AutoPoses {
    public final static Pose2d BluePortal = new Pose2d(14.359, 7.5, new Rotation2d(Math.toRadians(90)));
    public final static Pose2d RedPortal = new Pose2d(2.4, 7.2, new Rotation2d(Math.toRadians(90)));

    public final static Pose2d BlueOne = new Pose2d(1.88, 4.96, new Rotation2d(Math.toRadians(180)));
    public final static Pose2d BlueOneBack = new Pose2d(4.0, 4.96, new Rotation2d(Math.toRadians(180)));
    public final static Pose2d GP_BlueOne = new Pose2d(6.0, 4.59, new Rotation2d(Math.toRadians(0)));

    public final static Pose2d BlueNine = new Pose2d(1.88, 0.51, new Rotation2d(Math.toRadians(180)));
    public final static Pose2d BlueNineBack = new Pose2d(2.5, 0.75, new Rotation2d(Math.toRadians(180)));
    public final static Pose2d GP_BlueNine = new Pose2d(6.0, 0.91, new Rotation2d(Math.toRadians(0)));

    public final static Pose2d BlueFive = new Pose2d(1.88, 2.71, new Rotation2d(Math.toRadians(180)));
    public final static Pose2d BlueThree = new Pose2d(1.88, 3.88, new Rotation2d(Math.toRadians(180)));
    public final static Pose2d BlueSix = new Pose2d(1.88, 2.18, new Rotation2d(Math.toRadians(180)));
    public final static Pose2d BlueSeven = new Pose2d(1.88, 1.61, new Rotation2d(Math.toRadians(180)));
    public final static Pose2d BlueFiveBack = new Pose2d(5.5, 2.71, new Rotation2d(Math.toRadians(180)));
    public final static Pose2d GP_BlueFive = new Pose2d(6.0, 3.4, new Rotation2d(Math.toRadians(180)));

    // public final static Pose2d GP_BlueOne = new Pose2d(3.0, 4.96, new
    // Rotation2d(Math.toRadians(180)));
    // public final static Pose2d GP_BlueOne2 = new Pose2d(3.0, 4.96, new
    // Rotation2d(Math.toRadians(-90)));
    public final static Pose2d RedOne = new Pose2d(14.63, 0.50, new Rotation2d(Math.toRadians(0)));
    public final static Pose2d RedOneBack = new Pose2d(14, 0.5, new Rotation2d(Math.toRadians(0)));
    public final static Pose2d GP_RedOne = new Pose2d(10.5, 0.9, new Rotation2d(Math.toRadians(180)));

    public final static Pose2d RedNine = new Pose2d(14.63, 4.97, new Rotation2d(Math.toRadians(0)));
    public final static Pose2d RedNineBack = new Pose2d(12.55, 4.8, new Rotation2d(Math.toRadians(0)));
    public final static Pose2d GP_RedNine = new Pose2d(10.5, 4.6, new Rotation2d(Math.toRadians(180)));

    public final static Pose2d RedTwo = new Pose2d(14.63, 1.05, new Rotation2d(Math.toRadians(0)));
    public final static Pose2d RedThree = new Pose2d(14.63, 1.64, new Rotation2d(Math.toRadians(0)));
    public final static Pose2d RedSix = new Pose2d(14.63, 3.85, new Rotation2d(Math.toRadians(0)));

    public final static Pose2d RedFive = new Pose2d(14.63, 2.73, new Rotation2d(Math.toRadians(0)));

    public static void SetStartPose(Pose2d pose) {
        // RobotContainer.m_drivetrainSubsystem.zeroGyroscope();
        // RobotContainer.m_drivetrainSubsystem.resetOdometry();
        DataLogManager.log(
                "Setting Start Pose: Odometry Orignal Pose: "
                        + RobotContainer.m_drivetrainSubsystem.getOdometryPose());

        if (Robot.isSimulation()) {
            // RobotContainer.m_drivetrainSubsystem.getOdometry().resetPosition(
            // // pose.getRotation(),
            // new Rotation2d(0),
            // RobotContainer.m_drivetrainSubsystem.getSwervePositions(),
            // pose);
            // RobotContainer.m_drivetrainSubsystem.resetSimEncoders();
            RobotContainer.m_drivetrainSubsystem.setOdometryPose(pose);

        } else {
            RobotContainer.m_drivetrainSubsystem.setOdometryPose(pose);
            // RobotContainer.m_drivetrainSubsystem.getOdometry().resetPosition(pose.getRotation(),
            // RobotContainer.m_drivetrainSubsystem.getSwervePositions(),
            // pose); // uses pose, not gyro because gyro update above is async
            // // and may not be
            // // updated by now
        }
        DataLogManager.log(
                "Setting Start Pose: Odometry Updated Pose: "
                        + RobotContainer.m_drivetrainSubsystem.getOdometryPose());
    }

    public static CommandBase DriveToPose(Pose2d pose) {
        CommandBase cmd = new SnapDriveToPoseField(
                RobotContainer.m_drivetrainSubsystem,
                pose,
                0.05);

        return cmd;
    }
}
