// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.Map;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardLayout;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;

public class Tracking extends SubsystemBase {

    private double m_currentYaw = 0;
    private double m_goalYaw = 0;
    private double m_testTargetYaw = 0;
    private ShuffleboardTab m_shuffleboardTab = Shuffleboard.getTab("Sub.Tracking");
    private PhotonCamera m_camera;
    private double m_pEntry = 0.1;
    private double m_dEntry = 0.001;
    private PowerDistribution m_PDH = null;
    private boolean m_isLightOn = false;

    private boolean m_isTesting = false;
    private final int kCUBE = 0;
    private final int kCone = 1;
    private double kP = 0.75;
    // set to -1
    private int m_currentPipeline = -1;

    /** Creates a new Tracking. */
    public Tracking() {

        m_camera = new PhotonCamera("Game_Piece");
        m_PDH = new PowerDistribution(Constants.PDH, ModuleType.kRev);
        m_camera.setDriverMode(true);
    }

    public void GamePieceCube() {
        m_currentPipeline = kCUBE;
        m_camera.setPipelineIndex(m_currentPipeline);
        DataLogManager.log("Tracking: Tracking Cube.");
    }

    public void GamePieceCone() {
        m_currentPipeline = kCone;
        m_camera.setPipelineIndex(m_currentPipeline);
        DataLogManager.log("Tracking: Tracking Cone.");
    }

    @Override
    public void periodic() {

        SmartDashboard.putNumber("PhotonVisionOffset", this.getYawOffset());
        SmartDashboard.putNumber("PhotonVisionTargetYaw", this.getTargetHeading());
        SmartDashboard.putNumber("PhotonVisionPipeline", this.m_currentPipeline);

        // stop error message for now

        if (true)
            return;

        m_currentYaw = RobotContainer.m_drivetrainSubsystem.getGyroHeading()
                .getDegrees();
        m_goalYaw = m_currentYaw + getTargetHeading();
        this.setTestTarget(m_testTargetYaw);

        if (m_isTesting) {
            return;
        }
    }

    public double getYawOffset() {
        if (m_isTesting) {
            return m_testTargetYaw - m_currentYaw;
        }

        double yaw = 0;

        // set yaw equal to yaw from photonvision
        PhotonPipelineResult result = m_camera.getLatestResult();

        if (result.hasTargets()) {
            yaw = result.getBestTarget().getYaw();
        } else {
            return 0;
        }

        return (yaw) + 6.5;
    }

    public double getTargetHeading() {
        if (m_isTesting) {
            return m_testTargetYaw - m_currentYaw;
        }

        double yaw = 0;

        // set yaw equal to yaw from photonvision
        PhotonPipelineResult result = m_camera.getLatestResult();

        if (result.hasTargets()) {
            yaw = result.getBestTarget().getYaw();
        } else {
            return RobotContainer.m_drivetrainSubsystem.getGyroHeading().getDegrees();
        }

        return (kP * yaw) + 6.5 + RobotContainer.m_drivetrainSubsystem.getGyroHeading().getDegrees();
    }

    // used only for testing
    private Rotation2d m_testTarget = new Rotation2d();

    // d is in degrees
    public void setTestTarget(double degrees) {

        m_testTarget = new Rotation2d(Units.radiansToDegrees(degrees));
    }

    public boolean isTrackingTarget() {
        if (m_isTesting) {
            return true;
        }

        PhotonPipelineResult result = m_camera.getLatestResult();

        return result.hasTargets();
    }

    public void createShuffleBoardTab() {
        ShuffleboardTab tab = m_shuffleboardTab;
        ShuffleboardLayout commands = tab.getLayout("Commands", BuiltInLayouts.kList).withSize(2, 4)
                .withProperties(Map.of("Label position", "HIDDEN")); // hide labels for commands

        CommandBase c;

        c = new frc.robot.commands.Tracking.EnableTestMode();
        c.setName("Test Mode");
        commands.add(c);

        c = new frc.robot.commands.Tracking.TrackCone();
        c.setName("Cone");
        commands.add(c);

        c = new frc.robot.commands.Tracking.TrackCube();
        c.setName("Cube");
        commands.add(c);

        c = new frc.robot.commands.Tracking.EnableLight();
        c.setName("Enable Light");
        commands.add(c);

        c = Commands.runOnce(() -> m_camera.setDriverMode(false));
        c.setName("Driver Mode Off");
        commands.add(c);

        // m_testTargetYaw = m_shuffleboardTab.add("Test Target Yaw",
        // 0).withWidget(BuiltInWidgets.kNumberSlider)
        // .withSize(4, 1)
        // .withPosition(2, 0).withProperties(Map.of("min", -100.0, "max",
        // 100.0)).getEntry();

        // m_currentYaw = m_shuffleboardTab.add("Current Yaw",
        // 0).withWidget(BuiltInWidgets.kNumberSlider)
        // .withSize(4, 1)
        // .withPosition(2, 1).withProperties(Map.of("min", -100.0, "max",
        // 100.0)).getEntry();

        // m_goalYaw = m_shuffleboardTab.add("Goal Yaw",
        // 0).withWidget(BuiltInWidgets.kNumberSlider)
        // .withSize(4, 1)
        // .withPosition(2, 2).withProperties(Map.of("min", -100.0, "max",
        // 100.0)).getEntry();
        // m_pEntry = m_shuffleboardTab.add("Angle P", 0.1)
        // .withSize(1, 1)
        // .withPosition(6, 0).getEntry();
        // m_dEntry = m_shuffleboardTab.add("Angle D", 0.001)
        // .withSize(1, 1)
        // .withPosition(6, 1).getEntry();
        // m_testTargetYaw.setDouble(45);
    }

    public double getAngleP() {
        return m_pEntry;
    }

    public double getAngleD() {
        return m_dEntry;
    }

    public void EnableTestMode() {
        m_isTesting = true;
    }

    public void enableSearchLight() {
        m_PDH.setSwitchableChannel(true);
        m_camera.setDriverMode(false);
        m_isLightOn = true;
    }

    public void disableSearchLight() {
        m_PDH.setSwitchableChannel(false);
        m_camera.setDriverMode(true);
        m_isLightOn = false;
    }
}
