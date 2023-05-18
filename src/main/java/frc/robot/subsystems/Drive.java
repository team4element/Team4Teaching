package frc.robot.subsystems;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.ADXRS450_Gyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;

public class Drive extends SubsystemBase {
	private final MotorControllerGroup m_leftMotors;
	private final MotorControllerGroup m_rightMotors;
	private final DifferentialDrive m_drive;
	private final Encoder m_leftEncoder;
	private final Encoder m_rightEncoder;
	private final ADXRS450_Gyro m_gyro;
	private final DifferentialDriveOdometry m_odometry;

	public DifferentialDrivetrainSim m_drivetrainSimulator;
	private final EncoderSim m_leftEncoderSim;
	private final EncoderSim m_rightEncoderSim;
	private final Field2d m_fieldSim;
	private final ADXRS450_GyroSim m_gyroSim;

	public Drive() {
		m_leftMotors = new MotorControllerGroup(
				new PWMSparkMax(DriveConstants.kLeftMotor1Port),
				new PWMSparkMax(DriveConstants.kLeftMotor2Port));

		// The motors on the right side of the drive.
		m_rightMotors = new MotorControllerGroup(
				new PWMSparkMax(DriveConstants.kRightMotor1Port),
				new PWMSparkMax(DriveConstants.kRightMotor2Port));

		m_drive = new DifferentialDrive(m_leftMotors, m_rightMotors);

		m_leftEncoder = new Encoder(0, 1);
		m_rightEncoder = new Encoder(2, 3);

		m_gyro = new ADXRS450_Gyro();

		m_rightMotors.setInverted(true);
		m_leftEncoder.setDistancePerPulse(2 * Math.PI / 256);
		m_rightEncoder.setDistancePerPulse(2 * Math.PI / 256);

		resetEncoders();

		m_odometry = new DifferentialDriveOdometry(
				Rotation2d.fromDegrees(getHeading()),
				m_leftEncoder.getDistance(),
				m_rightEncoder.getDistance());

		if (RobotBase.isSimulation()) {
			m_drivetrainSimulator = new DifferentialDrivetrainSim(
					DriveConstants.kDrivetrainPlant,
					DriveConstants.kDriveGearbox,
					DriveConstants.kDriveGearing,
					DriveConstants.kTrackwidthMeters,
					DriveConstants.kWheelDiameterMeters / 2.0,
					VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));

			m_leftEncoderSim = new EncoderSim(m_leftEncoder);
			m_rightEncoderSim = new EncoderSim(m_rightEncoder);
			m_gyroSim = new ADXRS450_GyroSim(m_gyro);

			m_fieldSim = new Field2d();
			SmartDashboard.putData("Field", m_fieldSim);
		} else {
			m_leftEncoderSim = null;
			m_rightEncoderSim = null;
			m_gyroSim = null;
			m_fieldSim = null;
		}
	}

	public Field2d getField() {
		return m_fieldSim;
	}

	@Override
	public void periodic() {
		m_odometry.update(
				Rotation2d.fromDegrees(getHeading()),
				m_leftEncoder.getDistance(),
				m_rightEncoder.getDistance());
		m_fieldSim.setRobotPose(getPose());
	}

	@Override
	public void simulationPeriodic() {
		m_drivetrainSimulator.setInputs(
				m_leftMotors.get() * RobotController.getBatteryVoltage(),
				m_rightMotors.get() * RobotController.getBatteryVoltage());

		m_drivetrainSimulator.update(0.02);

		m_leftEncoderSim.setDistance(m_drivetrainSimulator.getLeftPositionMeters());
		m_leftEncoderSim.setRate(m_drivetrainSimulator.getLeftVelocityMetersPerSecond());
		m_rightEncoderSim.setDistance(m_drivetrainSimulator.getRightPositionMeters());
		m_rightEncoderSim.setRate(m_drivetrainSimulator.getRightVelocityMetersPerSecond());
		m_gyroSim.setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
	}

	public void resetEncoders() {
		m_leftEncoder.reset();
		m_rightEncoder.reset();
	}

	public double getHeading() {
		return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
	}

	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public void arcadeDrive(double fwd, double rot) {
		m_drive.arcadeDrive(fwd, rot);
	}

	/**
	 * Sets the max output of the drive. Useful for scaling the drive to drive more
	 * slowly.
	 *
	 * @param maxOutput the maximum output to which the drive will be constrained
	 */
	public void setMaxOutput(double maxOutput) {
		m_drive.setMaxOutput(maxOutput);
	}

	/** Zeroes the heading of the robot. */
	public void zeroHeading() {
		m_gyro.reset();
	}

	/**
	 * Returns the current wheel speeds of the robot.
	 *
	 * @return The current wheel speeds.
	 */
	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(m_leftEncoder.getRate(), m_rightEncoder.getRate());
	}

	/**
	 * Controls the left and right sides of the drive directly with voltages.
	 *
	 * @param leftVolts  the commanded left output
	 * @param rightVolts the commanded right output
	 */
	public void tankDriveVolts(double leftVolts, double rightVolts) {
		m_leftMotors.setVoltage(leftVolts);
		m_rightMotors.setVoltage(rightVolts);
		m_drive.feed();
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
		resetEncoders();
		m_drivetrainSimulator.setPose(pose);
		m_odometry.resetPosition(
				Rotation2d.fromDegrees(getHeading()),
				m_leftEncoder.getDistance(),
				m_rightEncoder.getDistance(),
				pose);
	}

	/**
	 * Returns the current being drawn by the drivetrain. This works in SIMULATION
	 * ONLY! If you want
	 * it to work elsewhere, use the code in
	 * {@link DifferentialDrivetrainSim#getCurrentDrawAmps()}
	 *
	 * @return The drawn current in Amps.
	 */
	public double getDrawnCurrentAmps() {
		return m_drivetrainSimulator.getCurrentDrawAmps();
	}

}
