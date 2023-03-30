// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Positioning;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.commands.Drive.SnapDriveToPoseField;
import frc.robot.commands.auto.AutoPoses;

public class GridStrafe extends CommandBase {
    private Pose2d m_pose;
    private CommandBase m_cmd;

    /** Creates a new GridStrafe. */
    public GridStrafe() {
        // Use addRequirements() here to declare subsystem dependencies.
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        // get correct target pose based on alliance color
        if (RobotContainer.m_GridSelector.isBlueAlliance()) {
            bluePose();
        } else {
            redPose();
        }

        DataLogManager.log("Straffing to: " + m_pose);

        m_cmd = new SnapDriveToPoseField(RobotContainer.m_drivetrainSubsystem,
                m_pose,
                0.05);
        m_cmd.schedule();
    }

    private void bluePose() {
        int i = RobotContainer.m_GridSelector.getDial() + 1;

        switch (i) {
            case 1:
                m_pose = AutoPoses.BlueOne;
                break;
            case 2:
                m_pose = AutoPoses.BlueTwo;
                break;
            case 3:
                m_pose = AutoPoses.BlueThree;
                break;
            case 4:
                m_pose = AutoPoses.BlueFour;
                break;
            case 5:
                m_pose = AutoPoses.BlueFive;
                break;
            case 6:
                m_pose = AutoPoses.BlueSix;
                break;
            case 7:
                m_pose = AutoPoses.BlueSeven;
                break;
            case 8:
                m_pose = AutoPoses.BlueEight;
                break;
            case 9:
                m_pose = AutoPoses.BlueNine;
                break;
            default:
                m_pose = AutoPoses.BlueTwo;
        }
    }

    private void redPose() {
        int i = RobotContainer.m_GridSelector.getDial() + 1;

        switch (i) {
            case 1:
                m_pose = AutoPoses.RedOne;
                break;
            case 2:
                m_pose = AutoPoses.RedTwo;
                break;
            case 3:
                m_pose = AutoPoses.RedThree;
                break;
            case 4:
                m_pose = AutoPoses.RedFour;
                break;
            case 5:
                m_pose = AutoPoses.RedFive;
                break;
            case 6:
                m_pose = AutoPoses.RedSix;
                break;
            case 7:
                m_pose = AutoPoses.RedSeven;
                break;
            case 8:
                m_pose = AutoPoses.RedEight;
                break;
            case 9:
                m_pose = AutoPoses.RedNine;
                break;
            default:
                m_pose = AutoPoses.RedTwo;
        }
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_cmd.cancel();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
