// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;
import frc.robot.subsystems.DrivetrainSubsystem;

public class SnapDriveToPoseField extends CommandBase {
    private final DrivetrainSubsystem m_drivetrainSubsystem;
    private Pose2d m_goal;
    ProfiledPIDController m_rotationPID = null;
    ProfiledPIDController m_xPID = null;
    ProfiledPIDController m_yPID = null;
    SimpleMotorFeedforward m_xFF = null;
    SimpleMotorFeedforward m_yFF = null;
    private Pose2d m_error = null;
    final private double m_maxErrorMeters;

    /** Creates a new DriveWhileTracking. */
    /**
     * 
     * @param drivetrainSubsystem Drive subsystem
     * @param goal                the goal pose that the robot should move to
     * @param maxErrorMeters      error tolerance in meters. Once the x and y error
     *                            is less than this amount the command will finish.
     */
    public SnapDriveToPoseField(DrivetrainSubsystem drivetrainSubsystem,
            Pose2d goal, double maxErrorMeters) {

        this.m_drivetrainSubsystem = drivetrainSubsystem;
        double kFACTOR = 1.0;

        m_goal = new Pose2d(goal.getX() * kFACTOR, goal.getY() * kFACTOR, goal.getRotation());
        m_maxErrorMeters = maxErrorMeters;

        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(drivetrainSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        double TRANSLATE_P = 1.0;
        double TRANSLATE_D = 0.0;
        double TRANSLATE_FF_S = 0.0; // keep zero we are going to use the PID and static FF
        double TRANSLATE_FF_V = 1.0; // Weight of the position pid to the velocity

        TrapezoidProfile.Constraints rotationConstraints = new TrapezoidProfile.Constraints(
                1.0, 0.5);
        m_rotationPID = new ProfiledPIDController(
                SnapConstants.kANGULAR_P, 0, SnapConstants.kANGULAR_D, rotationConstraints);

        // let's the theta controller know that it is a circle (ie, 180 = -180)
        m_rotationPID.enableContinuousInput(0, 360);

        TrapezoidProfile.Constraints translateConstraints = new TrapezoidProfile.Constraints(
                SnapConstants.kMAX_TRANSLATE_VELOCITY, SnapConstants.kMAX_TRANSLATE_ACCEL);
        m_xPID = new ProfiledPIDController(TRANSLATE_P, 0, TRANSLATE_D, translateConstraints);
        m_yPID = new ProfiledPIDController(TRANSLATE_P, 0, TRANSLATE_D, translateConstraints);

        m_xFF = new SimpleMotorFeedforward(TRANSLATE_FF_S, TRANSLATE_FF_V);
        m_yFF = new SimpleMotorFeedforward(TRANSLATE_FF_S, TRANSLATE_FF_V);

        // setup the X/Y PIDs

        // TODO add to the resets the current velocities
        Pose2d error = getError();
        m_xPID.reset(error.getTranslation().getX());
        m_yPID.reset(error.getTranslation().getY());

        // TODO uncomment this and test it, then add it to other Snap commands as
        // necessary...
        // m_rotationPID.reset(error.getRotation().getDegrees());

    }

    private Pose2d getError() {
        Pose2d current = RobotContainer.m_drivetrainSubsystem.getOdometryPose();

        Pose2d error = new Pose2d(m_goal.getX() - current.getX(),
                m_goal.getY() - current.getY(),
                m_goal.getRotation().minus(current.getRotation()));

        DataLogManager.log("SnapDriveToPoseField: Current: " + current + " Error: " + error);

        return error;
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // update error
        m_error = getError();

        // update PIDs
        double rotationSpeed = m_rotationPID.calculate(m_error.getRotation().getDegrees(), 0);

        rotationSpeed = MathUtil.clamp(rotationSpeed, -3.0, 3.0);

        if (Math.abs(rotationSpeed) < 0.05) {
            rotationSpeed = 0;
        }

        double translateSpeedX = m_xPID.calculate(m_error.getX(), 0);
        translateSpeedX += m_xFF.calculate(m_xPID.getSetpoint().velocity);
        translateSpeedX = MathUtil.clamp(translateSpeedX,
                -SnapConstants.kMAX_TRANSLATE_VELOCITY,
                SnapConstants.kMAX_TRANSLATE_VELOCITY);

        double TRANSLATE_FF = 0.0;
        if (translateSpeedX > 0) {
            translateSpeedX += TRANSLATE_FF;
        } else if (translateSpeedX < 0) {
            translateSpeedX -= TRANSLATE_FF;
        }

        double translateSpeedY = m_yPID.calculate(m_error.getY(), 0);
        translateSpeedY += m_yFF.calculate(m_yPID.getSetpoint().velocity);
        translateSpeedY = MathUtil.clamp(translateSpeedY,
                -SnapConstants.kMAX_TRANSLATE_VELOCITY,
                SnapConstants.kMAX_TRANSLATE_VELOCITY);

        if (translateSpeedY > 0) {
            translateSpeedY += TRANSLATE_FF;
        } else if (translateSpeedY < 0) {
            translateSpeedY -= TRANSLATE_FF;
        }

        DataLogManager.log("SnapDriveToPoseField: Corrections: X: " + translateSpeedX + " Y: " + translateSpeedY
                + " Rot: " + rotationSpeed);

        m_drivetrainSubsystem.drive(
                ChassisSpeeds.fromFieldRelativeSpeeds(
                        -translateSpeedX,
                        -translateSpeedY,
                        rotationSpeed,
                        RobotContainer.m_drivetrainSubsystem.getOdometryPose().getRotation()),
                m_goal.getRotation().getRadians());
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_drivetrainSubsystem.drive(new ChassisSpeeds(0.0, 0.0, 0.0), 0.0);
        DataLogManager.log("SnapDriveToPoseField End.");
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        // end if we have reached target Pose
        if ((Math.abs(m_error.getX()) < m_maxErrorMeters)
                && (Math.abs(m_error.getY()) < m_maxErrorMeters)
                && (Math.abs(m_error.getRotation().getDegrees()) < 1.0)) {
            return true;
        }
        return false;
    }
}