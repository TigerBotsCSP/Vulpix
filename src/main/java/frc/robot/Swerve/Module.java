// ! Akram

package frc.robot.Swerve;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.RobotContainer;

public class Module {
    // ! Change these values! Ensure they're accurate
    private static final double kWheelRadius = 0.420;
    private static final int kEncoderResolution = 4096;

    private static final double kModuleMaxAngularVelocity = RobotContainer.m_rotationSpeed;
    private static final double kModuleMaxAngularAcceleration = 2 * RobotContainer.m_rotationSpeed;

    // ? The NEO motors have built-in encoders
    private final CANSparkMax m_driveMotor;
    private final CANSparkMax m_turningMotor;

    // ! Gain values need to be thoroughly tested, play around with values
    private final PIDController m_drivePIDController = new PIDController(1, 0, 0);

    // ! Gain values need to be thoroughly tested, play around with values
    private final ProfiledPIDController m_turningPIDController = new ProfiledPIDController(1, 0, 0,
            new TrapezoidProfile.Constraints(
                    kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

    // ! Gain values need to be thoroughly tested, play around with values
    private final SimpleMotorFeedforward m_driveFeedforward = new SimpleMotorFeedforward(1, 3);
    private final SimpleMotorFeedforward m_turnFeedforward = new SimpleMotorFeedforward(1, 0.5);

    public Module(int driveMotor, int turningMotor) {
        m_driveMotor = new CANSparkMax(driveMotor, MotorType.kBrushless);
        m_turningMotor = new CANSparkMax(turningMotor, MotorType.kBrushless);

        // ? Sets the distance per pulse for the drive encoder
        m_driveMotor.getEncoder().setPositionConversionFactor(2 * Math.PI * kWheelRadius / kEncoderResolution);

        // ? Set the distance in radians per pulse for the turning encoder
        m_turningMotor.getEncoder().setPositionConversionFactor(2 * Math.PI / kEncoderResolution);

        // ? Limit turnings by the maximum turning speed
        m_turningPIDController.enableContinuousInput(-RobotContainer.m_rotationSpeed, RobotContainer.m_rotationSpeed);
    }

    // ? Sets a new state for the swerve module
    public void set(SwerveModuleState newState) {
        // ? Prevents spinning further than 90 degrees
        SwerveModuleState state = SwerveModuleState.optimize(newState,
                new Rotation2d(m_turningMotor.getEncoder().getPosition()));

        // ? Calculates the drive output from the drive PID controller
        final double driveOutput = m_drivePIDController.calculate(m_driveMotor.getEncoder().getVelocity(),
                state.speedMetersPerSecond);
        final double driveFeedforward = m_driveFeedforward.calculate(state.speedMetersPerSecond);

        // ? Calculate the turning motor output from the turning PID controller
        final double turnOutput = m_turningPIDController.calculate(m_turningMotor.getEncoder().getPosition(),
                state.angle.getRadians());
        final double turnFeedforward = m_turnFeedforward.calculate(m_turningPIDController.getSetpoint().velocity);

        // * Set the speeds!
        m_driveMotor.setVoltage(driveOutput + driveFeedforward);
        m_turningMotor.setVoltage(turnOutput + turnFeedforward);
    }

    // ? Get module's current position
    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(
                m_driveMotor.getEncoder().getPosition(), new Rotation2d(m_turningMotor.getEncoder().getPosition()));
    }
}
