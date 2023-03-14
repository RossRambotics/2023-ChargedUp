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
    public final static Pose2d RedPortal = new Pose2d(2.4, 7.5, new Rotation2d(Math.toRadians(90)));
    public final static Pose2d BlueOne = new Pose2d(1.88, 4.96, new Rotation2d(Math.toRadians(180)));
    public final static Pose2d GP_BlueOne = new Pose2d(7.10, 5.41, new Rotation2d(Math.toRadians(-90)));
    // public final static Pose2d GP_BlueOne = new Pose2d(3.0, 4.96, new
    // Rotation2d(Math.toRadians(180)));
    // public final static Pose2d GP_BlueOne2 = new Pose2d(3.0, 4.96, new
    // Rotation2d(Math.toRadians(-90)));
    public final static Pose2d RedOne = new Pose2d(14.63, 0.50, new Rotation2d(Math.toRadians(0)));
    public final static Pose2d Tag1 = new Pose2d(15.10, 1.25, new Rotation2d(Math.toRadians(0)));
    public final static Pose2d Tag1ConeLeft = new Pose2d(15.10, .5, new Rotation2d(Math.toRadians(0)));
    public final static Pose2d Tag1ConeRight = new Pose2d(14.59, 1.65, new Rotation2d(Math.toRadians(0)));
    public final static Pose2d Tag2 = new Pose2d(14.59, 2.75, new Rotation2d(Math.toRadians(0)));
    public final static Pose2d Tag2ConeLeft = new Pose2d(14.59, 2.2, new Rotation2d(Math.toRadians(0)));
    public final static Pose2d Tag2ConeRight = new Pose2d(14.59, 3.33, new Rotation2d(Math.toRadians(0)));
    public final static Pose2d Tag3 = new Pose2d(14.59, 4.44, new Rotation2d(Math.toRadians(0)));
    public final static Pose2d Tag3ConeLeft = new Pose2d(14.59, 3.89, new Rotation2d(Math.toRadians(0)));
    public final static Pose2d RedNine = new Pose2d(14.59, 5, new Rotation2d(Math.toRadians(180)));
    public final static Pose2d RedNineBack = new Pose2d(15, 5, new Rotation2d(Math.toRadians(180)));
    public final static Pose2d Tag6 = new Pose2d(1.97, 4.44, new Rotation2d(Math.toRadians(0)));
    public final static Pose2d Tag6ConeLeft = new Pose2d(1.97, 5, new Rotation2d(Math.toRadians(0)));
    public final static Pose2d Tag6ConeRight = new Pose2d(1.97, 3.89, new Rotation2d(Math.toRadians(0)));
    public final static Pose2d Tag7 = new Pose2d(1.97, 2.75, new Rotation2d(Math.toRadians(0)));
    public final static Pose2d Tag7ConeLeft = new Pose2d(1.97, 3.33, new Rotation2d(Math.toRadians(0)));
    public final static Pose2d Tag7ConeRight = new Pose2d(1.97, 2.2, new Rotation2d(Math.toRadians(0)));
    public final static Pose2d Tag8 = new Pose2d(1.97, 1.05, new Rotation2d(Math.toRadians(0)));
    public final static Pose2d Tag8ConeLeft = new Pose2d(1.97, 1.65, new Rotation2d(Math.toRadians(0)));
    public final static Pose2d Tag8ConeRight = new Pose2d(1.97, 0.5, new Rotation2d(Math.toRadians(0)));
    public final static Pose2d BlueLeftOuter = new Pose2d(6.49, 4.57, new Rotation2d(Math.toRadians(180)));
    public final static Pose2d BlueLeftInner = new Pose2d(6.49, 3.35, new Rotation2d(Math.toRadians(180)));
    public final static Pose2d BlueRightOuter = new Pose2d(6.49, 0.91, new Rotation2d(Math.toRadians(180)));
    public final static Pose2d BlueRightInner = new Pose2d(6.49, 2.12, new Rotation2d(Math.toRadians(180)));
    public final static Pose2d RedLeftOuter = new Pose2d(10.07, 0.91, new Rotation2d(Math.toRadians(180)));
    public final static Pose2d RedLeftInner = new Pose2d(10.07, 2.12, new Rotation2d(Math.toRadians(180)));
    public final static Pose2d RedRightOuter = new Pose2d(10.07, 4.57, new Rotation2d(Math.toRadians(180)));
    public final static Pose2d RedRightInner = new Pose2d(10.07, 3.35, new Rotation2d(Math.toRadians(180)));
    public final static Pose2d Tag4 = new Pose2d(15.63, 6.75, new Rotation2d(Math.toRadians(180)));
    public final static Pose2d Tag4Left = new Pose2d(15.63, 6.2, new Rotation2d(Math.toRadians(180)));
    public final static Pose2d Tag4Right = new Pose2d(15.63, 7.35, new Rotation2d(Math.toRadians(180)));
    public final static Pose2d Tag5 = new Pose2d(0.88, 6.75, new Rotation2d(Math.toRadians(180)));
    public final static Pose2d Tag5Left = new Pose2d(0.88, 7.47, new Rotation2d(Math.toRadians(180)));
    public final static Pose2d Tag5Right = new Pose2d(0.88, 6.15, new Rotation2d(Math.toRadians(180)));

    public static void SetStartPose(Pose2d pose) {
        // RobotContainer.m_drivetrainSubsystem.zeroGyroscope();
        DataLogManager
                .log("Setting Start Pose: Current Gyro: "
                        + RobotContainer.m_drivetrainSubsystem.getGyroscopeRotation());
        RobotContainer.m_drivetrainSubsystem.setGyroScope(pose.getRotation().getDegrees());
        DataLogManager
                .log("Setting Start Pose: Updated Gyro: "
                        + RobotContainer.m_drivetrainSubsystem.getGyroscopeRotation());
        // RobotContainer.m_drivetrainSubsystem.resetOdometry();
        DataLogManager.log(
                "Setting Start Pose: Odometry Orignal Pose: "
                        + RobotContainer.m_drivetrainSubsystem.getOdometryPose());

        if (Robot.isSimulation()) {
            RobotContainer.m_drivetrainSubsystem.getOdometry().resetPosition(
                    // pose.getRotation(),
                    new Rotation2d(0),
                    RobotContainer.m_drivetrainSubsystem.getSwervePositions(),
                    pose);
            RobotContainer.m_drivetrainSubsystem.resetSimEndoers();
        } else {
            RobotContainer.m_drivetrainSubsystem.getOdometry().resetPosition(pose.getRotation(),
                    RobotContainer.m_drivetrainSubsystem.getSwervePositions(),
                    pose); // uses pose, not gyro because gyro update above is async
                           // and may not be
                           // updated by now
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
