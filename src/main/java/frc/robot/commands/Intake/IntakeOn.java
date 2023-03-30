// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Intake;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IntakeOn extends CommandBase {
    private boolean m_isFinished;
    Timer m_timer = new Timer();

    /** Creates a new IntakeOn. */
    public IntakeOn() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.m_intakeWheels);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        RobotContainer.m_intakeWheels.intakeOn();
        m_isFinished = false;
        m_timer.stop();
        m_timer.reset();
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (RobotContainer.m_intakeFrame.hasGamePiece()) {
            m_isFinished = true;
            m_timer.start();

        }

    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.m_intakeWheels.intakeOff();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        if (m_timer.get() > 0.2) {
            return m_isFinished;
        }
        return false;
    }
}
