package frc.robot.subsystems.drivetrain;
import java.util.function.Supplier;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.RobotContainer;
import frc.robot.Constants.driveConstants;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem
 * so it can be used in command-based projects easily.
 */
public class CommandSwerveDrivetrain extends SwerveDrivetrain implements Subsystem {
    private static final double kSimLoopPeriod = 0.005; // 5 ms
    private Notifier m_simNotifier = null;
    private double m_lastSimTime;

    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency, SwerveModuleConstants... modules) {
        super(driveTrainConstants, OdometryUpdateFrequency, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }
    public CommandSwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        super(driveTrainConstants, modules);
        if (Utils.isSimulation()) {
            startSimThread();
        }
    }

    public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
        return run(() -> this.setControl(requestSupplier.get()));
    }

    private void startSimThread() {
        m_lastSimTime = Utils.getCurrentTimeSeconds();

        /* Run simulation at a faster rate so PID gains behave more reasonably */
        m_simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - m_lastSimTime;
            m_lastSimTime = currentTime;

            /* use the measured time delta, get battery voltage from WPILib */
            updateSimState(deltaTime, RobotController.getBatteryVoltage());
        });
        m_simNotifier.startPeriodic(kSimLoopPeriod);
    }



    // Field Oriented Drive
    public final static SwerveRequest.FieldCentric driveField = new SwerveRequest.FieldCentric()
        .withDeadband(driveConstants.MaxSpeed * driveConstants.SpeedDeadbandPercentage) // Speed deadzone
        .withRotationalDeadband(driveConstants.MaxAngularRate * driveConstants.SpeedDeadbandPercentage) // Rotation speed deadzone
        .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
        .withVelocityX(Sine(RobotContainer.joystick.getLeftX(), RobotContainer.joystick.getLeftY()) * driveConstants.MaxSpeed) // Drive forward with negative Y (forward)
        .withVelocityY(Cosine(RobotContainer.joystick.getLeftX(), RobotContainer.joystick.getLeftY()) * driveConstants.MaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(-Math.pow(Deadzone(RobotContainer.joystick.getRightX()), 3) * driveConstants.MaxAngularRate); // Drive counterclockwise with negative X (left)

    // Robot Oriented Drive
    public final static SwerveRequest.RobotCentric driveRobot = new SwerveRequest.RobotCentric()
        .withDeadband(driveConstants.MaxSpeed * driveConstants.SpeedDeadbandPercentage) // Speed deadzone
        .withRotationalDeadband(driveConstants.MaxAngularRate * driveConstants.SpeedDeadbandPercentage) // Rotation speed deadzone
        .withDriveRequestType(DriveRequestType.Velocity)
        .withVelocityX(Sine(RobotContainer.joystick.getLeftX(), RobotContainer.joystick.getLeftY()) * driveConstants.MaxSpeed) // Drive forward with negative Y (forward)
        .withVelocityY(Cosine(RobotContainer.joystick.getLeftX(), RobotContainer.joystick.getLeftY()) * driveConstants.MaxSpeed) // Drive left with negative X (left)
        .withRotationalRate(-Math.pow(Deadzone(RobotContainer.joystick.getRightX()), 3) * driveConstants.MaxAngularRate); // Drive counterclockwise with negative X (left)

    // Autonomous Robot Oriented Drive
    public final static SwerveRequest.RobotCentric driveAuto = new SwerveRequest.RobotCentric()
        .withDeadband(driveConstants.MaxSpeed * driveConstants.SpeedDeadbandPercentage) // Speed deadzone
        .withRotationalDeadband(driveConstants.MaxAngularRate * driveConstants.SpeedDeadbandPercentage) // Rotation speed deadzone
        .withDriveRequestType(DriveRequestType.Velocity);
        // Velocity and rotation set in RobotContainer



    ///// Controller Deadzone \\\\\
    public static double Deadzone(double speed) {
        if (Math.abs(speed) > driveConstants.ControllerDeadzone) return speed;
        return 0;
    }

    ///// Controller Y value curving \\\\\
    public static double Cosine(double x, double y) {
        x = Deadzone(x);
        y = Deadzone(y);
        return Math.pow(Math.sqrt((x*x)+(y*y)), driveConstants.Linearity) * Math.cos(Math.atan2(y,x));
    }

    ///// Controller X value curving \\\\\
    public static double Sine(double x, double y) {
        x = Deadzone(x);
        y = Deadzone(y);
        return Math.pow(Math.sqrt((x*x)+(y*y)), driveConstants.Linearity) * Math.sin(Math.atan2(y,x));
    }

    @Override 
    public void periodic() {
        
    }
}
