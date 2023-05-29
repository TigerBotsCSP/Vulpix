// ! Akram

package frc.robot.Swerve;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.robot.RobotContainer;

public class Drive {
    // ? Our four (expensive) motors
    // ! Change these channels!
    private final Module m_frontLeft = new Module(1, 2);
    private final Module m_frontRight = new Module(3, 4);
    private final Module m_backLeft = new Module(5, 6);
    private final Module m_backRight = new Module(7, 8);

    // ? These limiters smooth out controls (lower = slower acceleration)
    private final SlewRateLimiter m_xLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_yLimiter = new SlewRateLimiter(3);
    private final SlewRateLimiter m_rotLimiter = new SlewRateLimiter(3);

    // ? Each wheel has a position relative to the center of the robot
    // ! Change the values!
    private final Translation2d m_frontLeftLocation = new Translation2d(0, 0);
    private final Translation2d m_frontRightLocation = new Translation2d(0, -0);
    private final Translation2d m_backLeftLocation = new Translation2d(-0, 0);
    private final Translation2d m_backRightLocation = new Translation2d(-0, -0);

    // ? Helps determine the speeds for the modules
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
            m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

    // ? Helps determine the position on the field during autonomous
    private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(
            m_kinematics,
            RobotContainer.m_gyro.getRotation2d(),
            new SwerveModulePosition[] {
                    m_frontLeft.getPosition(),
                    m_frontRight.getPosition(),
                    m_backLeft.getPosition(),
                    m_backRight.getPosition()
            });

    public Drive() {
        RobotContainer.m_gyro.reset();
    }

    // * Moves the swerve drive.
    public void swerve(double xSpeed, double ySpeed, double rotSpeed, boolean... optionalRelativeToField) {
        // ? Optional parameter
        boolean relativeToField = (optionalRelativeToField.length >= 1) ? optionalRelativeToField[0] : false;
        
        // ? Here, we invert and calculate the three speeds
        xSpeed = -m_xLimiter.calculate(MathUtil.applyDeadband(xSpeed, 0.02))
                * RobotContainer.m_driveSpeed;

        ySpeed = -m_yLimiter.calculate(MathUtil.applyDeadband(ySpeed, 0.02))
                * RobotContainer.m_driveSpeed;

        rotSpeed = -m_rotLimiter.calculate(MathUtil.applyDeadband(rotSpeed, 0.02))
                * RobotContainer.m_rotationSpeed;

        // ? This returns the individual speeds for each module
        SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
                relativeToField
                        ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rotSpeed,
                                RobotContainer.m_gyro.getRotation2d())
                        : new ChassisSpeeds(xSpeed, ySpeed, rotSpeed));

        // ? Ensures speeds are not above the maximum speed
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, RobotContainer.m_driveSpeed);

        // * Set the speeds!
        m_frontLeft.set(swerveModuleStates[0]);
        m_frontRight.set(swerveModuleStates[1]);
        m_backLeft.set(swerveModuleStates[2]);
        m_backRight.set(swerveModuleStates[3]);
    }

    // ? Update robot's field position for autonomous
    public void updateOdometry() {
        m_odometry.update(
                RobotContainer.m_gyro.getRotation2d(),
                new SwerveModulePosition[] {
                        m_frontLeft.getPosition(),
                        m_frontRight.getPosition(),
                        m_backLeft.getPosition(),
                        m_backRight.getPosition()
                });
    }
}
