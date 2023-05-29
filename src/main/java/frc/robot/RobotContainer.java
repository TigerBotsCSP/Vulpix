package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.I2C;

public class RobotContainer {
    // Core
    public static AHRS m_gyro = new AHRS(I2C.Port.kMXP);

    // Controllers
    public static XboxController m_controller = new XboxController(0);

    // Constants
    public static final double m_driveSpeed = 3;
    public static final double m_rotationSpeed = Math.PI;
}
