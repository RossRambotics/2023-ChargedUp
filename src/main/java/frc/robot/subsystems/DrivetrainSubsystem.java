// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

//import com.ctre.phoenix.sensors.PigeonIMU;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import com.swervedrivespecialties.swervelib.MkModuleConfiguration;
import com.swervedrivespecialties.swervelib.MkSwerveModuleBuilder;
import com.swervedrivespecialties.swervelib.MotorType;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;
import com.swervedrivespecialties.swervelib.SwerveModule;

import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import static frc.robot.Constants.*;

public class DrivetrainSubsystem extends SubsystemBase {
        /**
         * The maximum voltage that will be delivered to the drive motors.
         * <p>
         * This can be reduced to cap the robot's maximum speed. Typically, this is
         * useful during initial testing of the robot.
         */
        public static final double MAX_VOLTAGE = 12.0;

        // FIXME Measure the drivetrain's maximum velocity or calculate the theoretical.
        // The formula for calculating the theoretical maximum velocity is:
        // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
        // pi
        // By default this value is setup for a Mk3 standard module using Falcon500s to
        // drive.
        // An example of this constant for a Mk4 L2 module with NEOs to drive is:
        // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
        // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
        /**
         * The maximum velocity of the robot in meters per second.
         * <p>
         * This is a measure of how fast the robot should be able to drive in a straight
         * line.
         */
        // public static final double MAX_VELOCITY_METERS_PER_SECOND = 6380.0 / 60.0 *
        // SdsModuleConfigurations.MK4_L1.getDriveReduction() *
        // SdsModuleConfigurations.MK4_L1.getWheelDiameter() * Math.PI; // this is
        // roughly 4.116 for our setup
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 4;

        /**
         * The maximum angular velocity of the robot in radians per second.
         * <p>
         * This is a measure of how fast the robot can rotate in place.
         */
        // Here we calculate the theoretical maximum angular velocity. You can also
        // replace this with a measured amount.

        // public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND =
        // MAX_VELOCITY_METERS_PER_SECOND /
        // Math.hypot(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS /
        // 2.0); // This formula for our setup is ~13.2
        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = 6.0;

        private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
                        // Front left
                        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Front right
                        new Translation2d(DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back left
                        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, DRIVETRAIN_WHEELBASE_METERS / 2.0),
                        // Back right
                        new Translation2d(-DRIVETRAIN_TRACKWIDTH_METERS / 2.0, -DRIVETRAIN_WHEELBASE_METERS / 2.0));

        public SwerveDriveKinematics getKinematics() {
                return m_kinematics;
        }

        // The important thing about how you configure your gyroscope is that rotating
        // the robot counter-clockwise should
        // cause the angle reading to increase until it wraps back over to zero.
        private WPI_Pigeon2 m_pigeon;

        // These are our modules. We initialize them in the constructor.
        private SwerveModule m_frontLeftModule;
        private SwerveModule m_frontRightModule;
        private SwerveModule m_backLeftModule;
        private SwerveModule m_backRightModule;

        // private Pose2d m_odometryPose = new Pose2d();
        private SwerveModuleState[] m_swerveModuleStates = new SwerveModuleState[4]; // added while adding odometry
                                                                                     // support
                                                                                     // & replaced m_chassisSpeeds

        private SwerveModulePosition[] m_swerveModulePositions = new SwerveModulePosition[4];

        // adding SwerveOdometry
        private static SwerveDrivePoseEstimator m_odometry = null;

        private Field2d m_field = new Field2d();

        private double m_lastRotationSpeed;

        private Timer m_Timer = new Timer();

        public DrivetrainSubsystem() {
                m_Timer.start();
                m_swerveModuleStates[0] = new SwerveModuleState();
                m_swerveModuleStates[1] = new SwerveModuleState();
                m_swerveModuleStates[2] = new SwerveModuleState();
                m_swerveModuleStates[3] = new SwerveModuleState();

                m_swerveModulePositions[0] = new SwerveModulePosition();
                m_swerveModulePositions[1] = new SwerveModulePosition();
                m_swerveModulePositions[2] = new SwerveModulePosition();
                m_swerveModulePositions[3] = new SwerveModulePosition();

                ShuffleboardTab tab = Shuffleboard.getTab("Drivetrain");
                ShuffleboardTab fieldtab = Shuffleboard.getTab("Field");
                fieldtab.add(m_field).withWidget(BuiltInWidgets.kField)
                                .withSize(8, 8)
                                .withPosition(0, 0);

                // gets the serial number ooff of the robot
                // Midas serial number is 031E3241
                // GraveStone serial number is 03178474
                if (RobotController.getSerialNumber().equals("03178474")) {
                        this.createGraveStoneDrivetrain();
                        DataLogManager.log("%%%%%%%%%%%%%GraveStone Drive%%%%%%%%%%%%%%%%");
                } else {
                        this.createMidasDrivetrain();
                        DataLogManager.log("%%%%%%%%%%%%%%Midas Dreive%%%%%%%%%%%%%");
                }

                m_simTimer.start();

                m_odometry = new SwerveDrivePoseEstimator(m_kinematics, getGyroscopeRotation(),
                                m_swerveModulePositions, new Pose2d(0, 0, new Rotation2d(Math.toRadians(0))));
        }

        private void createGraveStoneDrivetrain() {
                m_pigeon = new WPI_Pigeon2(DRIVETRAIN_PIGEON_ID, CANBUS_DRIVETRAIN_GRAVESTONE);

                MkModuleConfiguration moduleConfig = MkModuleConfiguration.getDefaultSteerFalcon500();
                moduleConfig.setDriveCurrentLimit(40.0);
                moduleConfig.setSteerCurrentLimit(30.0);

                m_frontLeftModule = new MkSwerveModuleBuilder(moduleConfig)
                                // .withLayout(getSMLayout(tab.getLayout("Front Left Module",
                                // BuiltInLayouts.kList))
                                // .withPosition(0, 0))
                                .withGearRatio(SdsModuleConfigurations.MK4_L1)
                                .withDriveMotor(MotorType.FALCON, FRONT_LEFT_MODULE_DRIVE_MOTOR,
                                                CANBUS_DRIVETRAIN_GRAVESTONE)
                                .withSteerMotor(MotorType.FALCON, FRONT_LEFT_MODULE_STEER_MOTOR,
                                                CANBUS_DRIVETRAIN_GRAVESTONE)
                                .withSteerEncoderPort(FRONT_LEFT_MODULE_STEER_ENCODER, CANBUS_DRIVETRAIN_GRAVESTONE)
                                .withSteerOffset(FRONT_LEFT_MODULE_STEER_OFFSET_GRAVESTONE)
                                .build();

                m_frontRightModule = new MkSwerveModuleBuilder(moduleConfig)
                                // .withLayout(getSMLayout(tab.getLayout("Front Right Module",
                                // BuiltInLayouts.kList))
                                // .withPosition(3, 0))
                                .withGearRatio(SdsModuleConfigurations.MK4_L1)
                                .withDriveMotor(MotorType.FALCON, FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                                                CANBUS_DRIVETRAIN_GRAVESTONE)
                                .withSteerMotor(MotorType.FALCON, FRONT_RIGHT_MODULE_STEER_MOTOR,
                                                CANBUS_DRIVETRAIN_GRAVESTONE)
                                .withSteerEncoderPort(FRONT_RIGHT_MODULE_STEER_ENCODER, CANBUS_DRIVETRAIN_GRAVESTONE)
                                .withSteerOffset(FRONT_RIGHT_MODULE_STEER_OFFSET_GRAVESTONE)
                                .build();

                m_backLeftModule = new MkSwerveModuleBuilder(moduleConfig)
                                // .withLayout(getSMLayout(tab.getLayout("Back Left Module",
                                // BuiltInLayouts.kList))
                                // .withPosition(6, 0))
                                .withGearRatio(SdsModuleConfigurations.MK4_L1)
                                .withDriveMotor(MotorType.FALCON, BACK_LEFT_MODULE_DRIVE_MOTOR,
                                                CANBUS_DRIVETRAIN_GRAVESTONE)
                                .withSteerMotor(MotorType.FALCON, BACK_LEFT_MODULE_STEER_MOTOR,
                                                CANBUS_DRIVETRAIN_GRAVESTONE)
                                .withSteerEncoderPort(BACK_LEFT_MODULE_STEER_ENCODER, CANBUS_DRIVETRAIN_GRAVESTONE)
                                .withSteerOffset(BACK_LEFT_MODULE_STEER_OFFSET_GRAVESTONE)
                                .build();

                m_backRightModule = new MkSwerveModuleBuilder(moduleConfig)
                                // .withLayout(getSMLayout(tab.getLayout("Back Right Module",
                                // BuiltInLayouts.kList))
                                // .withPosition(9, 0))
                                .withGearRatio(SdsModuleConfigurations.MK4_L1)
                                .withDriveMotor(MotorType.FALCON, BACK_RIGHT_MODULE_DRIVE_MOTOR,
                                                CANBUS_DRIVETRAIN_GRAVESTONE)
                                .withSteerMotor(MotorType.FALCON, BACK_RIGHT_MODULE_STEER_MOTOR,
                                                CANBUS_DRIVETRAIN_GRAVESTONE)
                                .withSteerEncoderPort(BACK_RIGHT_MODULE_STEER_ENCODER, CANBUS_DRIVETRAIN_GRAVESTONE)
                                .withSteerOffset(BACK_RIGHT_MODULE_STEER_OFFSET_GRAVESTONE)
                                .build();
        }

        private void createMidasDrivetrain() {
                m_pigeon = new WPI_Pigeon2(DRIVETRAIN_PIGEON_ID, CANBUS_DRIVETRAIN_MIDAS);

                MkModuleConfiguration moduleConfig = MkModuleConfiguration.getDefaultSteerFalcon500();
                moduleConfig.setDriveCurrentLimit(40.0);
                moduleConfig.setSteerCurrentLimit(30.0);

                m_frontLeftModule = new MkSwerveModuleBuilder(moduleConfig)
                                // .withLayout(getSMLayout(tab.getLayout("Front Left Module",
                                // BuiltInLayouts.kList))
                                // .withPosition(0, 0))
                                .withGearRatio(SdsModuleConfigurations.MK4I_L1)
                                .withDriveMotor(MotorType.FALCON, FRONT_LEFT_MODULE_DRIVE_MOTOR,
                                                CANBUS_DRIVETRAIN_MIDAS)
                                .withSteerMotor(MotorType.FALCON, FRONT_LEFT_MODULE_STEER_MOTOR,
                                                CANBUS_DRIVETRAIN_MIDAS)
                                .withSteerEncoderPort(FRONT_LEFT_MODULE_STEER_ENCODER, CANBUS_DRIVETRAIN_MIDAS)
                                .withSteerOffset(FRONT_LEFT_MODULE_STEER_OFFSET_MIDAS)
                                .build();

                m_frontRightModule = new MkSwerveModuleBuilder(moduleConfig)
                                // .withLayout(getSMLayout(tab.getLayout("Front Right Module",
                                // BuiltInLayouts.kList))
                                // .withPosition(3, 0))
                                .withGearRatio(SdsModuleConfigurations.MK4I_L1)
                                .withDriveMotor(MotorType.FALCON, FRONT_RIGHT_MODULE_DRIVE_MOTOR,
                                                CANBUS_DRIVETRAIN_MIDAS)
                                .withSteerMotor(MotorType.FALCON, FRONT_RIGHT_MODULE_STEER_MOTOR,
                                                CANBUS_DRIVETRAIN_MIDAS)
                                .withSteerEncoderPort(FRONT_RIGHT_MODULE_STEER_ENCODER, CANBUS_DRIVETRAIN_MIDAS)
                                .withSteerOffset(FRONT_RIGHT_MODULE_STEER_OFFSET_MIDAS)
                                .build();

                m_backLeftModule = new MkSwerveModuleBuilder(moduleConfig)
                                // .withLayout(getSMLayout(tab.getLayout("Back Left Module",
                                // BuiltInLayouts.kList))
                                // .withPosition(6, 0))
                                .withGearRatio(SdsModuleConfigurations.MK4I_L1)
                                .withDriveMotor(MotorType.FALCON, BACK_LEFT_MODULE_DRIVE_MOTOR,
                                                CANBUS_DRIVETRAIN_MIDAS)
                                .withSteerMotor(MotorType.FALCON, BACK_LEFT_MODULE_STEER_MOTOR,
                                                CANBUS_DRIVETRAIN_MIDAS)
                                .withSteerEncoderPort(BACK_LEFT_MODULE_STEER_ENCODER, CANBUS_DRIVETRAIN_MIDAS)
                                .withSteerOffset(BACK_LEFT_MODULE_STEER_OFFSET_MIDAS)
                                .build();

                m_backRightModule = new MkSwerveModuleBuilder(moduleConfig)
                                // .withLayout(getSMLayout(tab.getLayout("Back Right Module",
                                // BuiltInLayouts.kList))
                                // .withPosition(9, 0))
                                .withGearRatio(SdsModuleConfigurations.MK4I_L1)
                                .withDriveMotor(MotorType.FALCON, BACK_RIGHT_MODULE_DRIVE_MOTOR,
                                                CANBUS_DRIVETRAIN_MIDAS)
                                .withSteerMotor(MotorType.FALCON, BACK_RIGHT_MODULE_STEER_MOTOR,
                                                CANBUS_DRIVETRAIN_MIDAS)
                                .withSteerEncoderPort(BACK_RIGHT_MODULE_STEER_ENCODER, CANBUS_DRIVETRAIN_MIDAS)
                                .withSteerOffset(BACK_RIGHT_MODULE_STEER_OFFSET_MIDAS)
                                .build();
        }

        /**
         * Sets the gyroscope angle to zero. This can be used to set the direction the
         * robot is currently facing to the
         * 'forwards' direction.
         */
        public void zeroGyroscope() {

                m_pigeon.setYaw(0.0, 10);
        }

        /**
         * updates the current gyro yaw. this update is asynchronous and while it waits
         * 10ms for the update, the actual update may take longer.
         * 
         * @param yawDegrees
         */
        public void setGyroScope(double yawDegrees) {
                m_pigeon.setYaw(yawDegrees, 10);
        }

        public Rotation2d getGyroscopeRotation() {
                return Rotation2d.fromDegrees(m_pigeon.getYaw());
        }

        // clockwise rotoation is a positive change in angle
        public Rotation2d getGyroHeading() {
                Rotation2d r = new Rotation2d();
                r = Rotation2d.fromDegrees(-m_pigeon.getYaw());
                return r;
        }

        public void calibrateSDSModules() {
                m_frontLeftModule.set(0, 0);
                m_frontRightModule.set(0, 0);
                m_backLeftModule.set(0, 0);
                m_backLeftModule.set(0, 0);
        }

        private void updateSDSSwerveModules() {
                m_frontLeftModule.set(
                                m_swerveModuleStates[0].speedMetersPerSecond /
                                                MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                m_swerveModuleStates[0].angle.getRadians());
                m_frontRightModule.set(
                                m_swerveModuleStates[1].speedMetersPerSecond /
                                                MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                m_swerveModuleStates[1].angle.getRadians());
                m_backLeftModule.set(
                                m_swerveModuleStates[2].speedMetersPerSecond /
                                                MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                m_swerveModuleStates[2].angle.getRadians());
                m_backRightModule.set(
                                m_swerveModuleStates[3].speedMetersPerSecond /
                                                MAX_VELOCITY_METERS_PER_SECOND * MAX_VOLTAGE,
                                m_swerveModuleStates[3].angle.getRadians());

        }

        /**
         * 
         * @param chassisSpeeds
         * @param rotationSpeed - used for simulation only
         */
        public void drive(ChassisSpeeds chassisSpeeds, double rotationSpeed) {
                m_swerveModuleStates = m_kinematics.toSwerveModuleStates(chassisSpeeds);
                SwerveDriveKinematics.desaturateWheelSpeeds(
                                m_swerveModuleStates,
                                MAX_VELOCITY_METERS_PER_SECOND);

                m_lastRotationSpeed = rotationSpeed;
                // update the actual swerve modules
                this.setSwerveModulesStates();
        }

        // method created to facilitate autonomous using odometry
        public void setSwerveModulesStates() {
                SwerveDriveKinematics.desaturateWheelSpeeds(
                                m_swerveModuleStates,
                                MAX_VELOCITY_METERS_PER_SECOND);

                // update the actual swerve modules
                this.updateSDSSwerveModules();

                // update the positions
                m_swerveModulePositions[0].angle = Rotation2d.fromRadians(m_frontLeftModule.getSteerAngle());
                m_swerveModulePositions[0].distanceMeters = m_frontLeftModule.getDriveDistance();

                m_swerveModulePositions[1].angle = Rotation2d.fromRadians(m_frontRightModule.getSteerAngle());
                m_swerveModulePositions[1].distanceMeters = m_frontRightModule.getDriveDistance();

                m_swerveModulePositions[2].angle = Rotation2d.fromRadians(m_backLeftModule.getSteerAngle());
                m_swerveModulePositions[2].distanceMeters = m_backLeftModule.getDriveDistance();

                m_swerveModulePositions[3].angle = Rotation2d.fromRadians(m_backLeftModule.getSteerAngle());
                m_swerveModulePositions[3].distanceMeters = m_backLeftModule.getDriveDistance();
        }

        private double m_simEncoders[] = { 0.0, 0.0, 0.0, 0.0 };

        @Override
        public void periodic() {

                SmartDashboard.putNumber("Gyro Heading (Yaw)", this.getGyroHeading().getDegrees());
                SmartDashboard.putNumber("Gyro Pitch", this.getPitch());
                // update odometry
                if (!Robot.isSimulation()) {
                        RobotContainer.m_positioning.updateVision(m_odometry);
                        m_odometry.update(getGyroscopeRotation(), m_swerveModulePositions);
                }

                // update field sim
                if (Robot.isSimulation()) {
                        // This method will be called once per scheduler run during simulation only
                        // update the pose every
                        // 0.02 seconds
                        if (m_simTimer.advanceIfElapsed(0.02)) {

                                // we need to simulate the update of the (x,y) of the robot
                                // we will use the speed of the modules
                                m_simEncoders[0] += m_swerveModuleStates[0].speedMetersPerSecond
                                                * 0.02;
                                m_simEncoders[1] += m_swerveModuleStates[1].speedMetersPerSecond
                                                * 0.02;
                                m_simEncoders[2] += m_swerveModuleStates[2].speedMetersPerSecond
                                                * 0.02;
                                m_simEncoders[3] += m_swerveModuleStates[3].speedMetersPerSecond
                                                * 0.02;

                                m_swerveModulePositions[0].distanceMeters = m_simEncoders[0];
                                m_swerveModulePositions[0].angle = m_swerveModuleStates[0].angle;
                                m_swerveModulePositions[1].distanceMeters = m_simEncoders[1];
                                m_swerveModulePositions[1].angle = m_swerveModuleStates[1].angle;
                                m_swerveModulePositions[2].distanceMeters = m_simEncoders[2];
                                m_swerveModulePositions[2].angle = m_swerveModuleStates[2].angle;
                                m_swerveModulePositions[3].distanceMeters = m_simEncoders[3];
                                m_swerveModulePositions[3].angle = m_swerveModuleStates[3].angle;

                                RobotContainer.m_positioning.updateVision(m_odometry);
                                m_odometry.update(getGyroscopeRotation(), m_swerveModulePositions);

                                Pose2d simPose = new Pose2d(
                                                getOdometryPose().getX(),
                                                getOdometryPose().getY(),
                                                new Rotation2d(m_lastRotationSpeed));
                                m_field.setRobotPose(simPose);
                        }
                } else {
                        m_field.setRobotPose(getOdometryPose());
                }

        }

        public Pose2d getOdometryPose() {
                return m_odometry.getEstimatedPosition();
        }

        public static SwerveDrivePoseEstimator getOdometry() {
                return m_odometry;
        }

        private final Timer m_simTimer = new Timer();

        @Override
        public void simulationPeriodic() {

        }

        public SwerveModulePosition[] getSwervePositions() {
                return m_swerveModulePositions;
        }

        public void resetSimEndoers() {
                m_simEncoders[0] = 0.0;
                m_simEncoders[1] = 0.0;
                m_simEncoders[2] = 0.0;
                m_simEncoders[3] = 0.0;
        }

        public void setOdometryPose(Pose2d botPose) {
                m_odometry.resetPosition(getGyroHeading(), m_swerveModulePositions, botPose);
                System.out.println("Reseting Odometry Pose. Gyro: " + getGyroHeading() + " botPose Heading: "
                                + botPose.getRotation().getDegrees());
        }

        public double getPitch() {
                return m_pigeon.getRoll();
        }

}
