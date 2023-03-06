package com.swervedrivespecialties.swervelib;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;

public interface SwerveModule {
    MotorController getDriveMotor();

    MotorController getSteerMotor();

    AbsoluteEncoder getSteerEncoder();

    /**
     * Get module drive velocity
     * 
     * @return drive velocity in m/s
     */
    double getDriveVelocity();

    /**
     * Get module drive distance
     * 
     * @return drive distance in meters
     */
    double getDriveDistance();

    /**
     * Get module steer angle
     * 
     * @return steer angle in radians from [0, 2pi)
     */
    double getSteerAngle();

    default SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromRadians(getSteerAngle()));
    }

    default SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDriveDistance(), Rotation2d.fromRadians(getSteerAngle()));
    }

    /**
     * Reset motor or encoder position to the absolute position. May take a little bit.
     */
    void resetToAbsolute();

    void set(double driveVoltage, double steerAngle);
}
