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
import edu.wpi.first.wpilibj.simulation.ADXRS450_GyroSim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.EncoderSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.DriveConstants;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

class DriveSide {
	public final MotorControllerGroup motors;
	public final Encoder encoder;
	public final EncoderSim encoderSim;

	public DriveSide(MotorControllerGroup motors, Encoder encoder) {
		this.motors = motors;
		this.encoder = encoder;

		this.encoderSim = RobotBase.isSimulation() ? new EncoderSim(encoder) : null;
	}

	public MotorControllerGroup getMotors() {
		return motors;
	}

	public void invert(boolean isInverted) {
		motors.setInverted(isInverted);
	}

	public void configureEncoder(int pulsesPerRev) {
		encoder.setDistancePerPulse(2 * Math.PI / pulsesPerRev);
	}

	public EncoderSim getEncoderSim() {
		return encoderSim;
	}

	public Encoder getEncoder() {
		return encoder;
	}

	public void resetEncoder() {
		encoder.reset();
	}
}

class Gyro {
	private final ADXRS450_Gyro m_gyro;
	private final ADXRS450_GyroSim m_gyroSim;

	public Gyro() {
		m_gyro = new ADXRS450_Gyro();
		m_gyroSim = RobotBase.isSimulation() ? new ADXRS450_GyroSim(m_gyro) : null;
	}

	public double getAngle() {
		return m_gyro.getAngle();
	}

	public void reset() {
		m_gyro.reset();
	}

	public ADXRS450_GyroSim getGyroSim() {
		return m_gyroSim;
	}

	public double getHeading() {
		return Math.IEEEremainder(m_gyro.getAngle(), 360) * (DriveConstants.kGyroReversed ? -1.0 : 1.0);
	}
}

class Drivetrain {
	private final DriveSide m_left;
	private final DriveSide m_right;
	private final DifferentialDrive m_drive;
	private final DifferentialDrivetrainSim m_drivetrainSimulator;

	public Drivetrain(DriveSide left, DriveSide right) {
		m_drive = new DifferentialDrive(left.getMotors(), right.getMotors());
		m_left = left;
		m_right = right;

		if (RobotBase.isSimulation()) {
			m_drivetrainSimulator = new DifferentialDrivetrainSim(
					DriveConstants.kDrivetrainPlant,
					DriveConstants.kDriveGearbox,
					DriveConstants.kDriveGearing,
					DriveConstants.kTrackwidthMeters,
					DriveConstants.kWheelDiameterMeters / 2.0,
					VecBuilder.fill(0, 0, 0.0001, 0.1, 0.1, 0.005, 0.005));
		} else {
			m_drivetrainSimulator = null;
		}
	}

	public void resetEncoders() {
		m_left.resetEncoder();
		m_right.resetEncoder();
	}

	public void setArcadeDrive(double fwd, double rot) {
		m_drive.arcadeDrive(fwd, rot);
	}

	public void tankDriveVolts(double leftVolts, double rightVolts) {
		m_left.getMotors().setVoltage(leftVolts);
		m_right.getMotors().setVoltage(rightVolts);
		m_drive.feed();
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

	public void setSimPose(Pose2d pose) {
		if (RobotBase.isSimulation()) {
			m_drivetrainSimulator.setPose(pose);
		}
	}

	public double getSimCurrentDrawAmps() {
		return RobotBase.isSimulation() ? m_drivetrainSimulator.getCurrentDrawAmps() : 0;
	}

	public void updateSimulation(Gyro m_gyro) {
		if (!RobotBase.isSimulation()) {
			return;
		}
		double leftVoltage = m_left.getMotors().get();
		double rightVoltage = m_right.getMotors().get();

		m_drivetrainSimulator.setInputs(
				leftVoltage * RobotController.getBatteryVoltage(),
				rightVoltage * RobotController.getBatteryVoltage());

		m_drivetrainSimulator.update(0.02);

		double simulatedLeftPosition = m_drivetrainSimulator.getLeftPositionMeters();
		double simulatedRightPosition = m_drivetrainSimulator.getRightPositionMeters();
		double simulatedLeftVelocity = m_drivetrainSimulator.getLeftVelocityMetersPerSecond();
		double simulatedRightVelocity = m_drivetrainSimulator.getRightVelocityMetersPerSecond();

		m_left.getEncoderSim().setDistance(simulatedLeftPosition);
		m_right.getEncoderSim().setDistance(simulatedRightPosition);
		m_left.getEncoderSim().setRate(simulatedLeftVelocity);
		m_right.getEncoderSim().setRate(simulatedRightVelocity);

		m_gyro.getGyroSim().setAngle(-m_drivetrainSimulator.getHeading().getDegrees());
	}
}

public class Drive extends SubsystemBase {
	private final DriveSide m_leftSide;
	private final DriveSide m_rightSide;

	private final Drivetrain m_drivetrain;
	private final DifferentialDriveOdometry m_odometry;

	private final Field2d m_fieldSim;
	private final Gyro m_gyro1;

	public Drive() {
		m_leftSide = new DriveSide(
				new MotorControllerGroup(
						new WPI_TalonFX(0),
						new WPI_TalonFX(1)),
				new Encoder(0, 1));

		m_rightSide = new DriveSide(
				new MotorControllerGroup(
						new WPI_TalonFX(2),
						new WPI_TalonFX(3)),
				new Encoder(2, 3));

		// Configure Hardware
		m_leftSide.configureEncoder(256);
		m_rightSide.configureEncoder(256);
		m_rightSide.invert(true);

		m_drivetrain = new Drivetrain(m_leftSide, m_rightSide);
		m_drivetrain.resetEncoders();

		m_gyro1 = new Gyro();

		// TODO: Abstract to RobotState
		m_odometry = new DifferentialDriveOdometry(
				Rotation2d.fromDegrees(m_gyro1.getHeading()),
				m_leftSide.getEncoder().getDistance(),
				m_rightSide.getEncoder().getDistance());

		if (RobotBase.isSimulation()) {
			m_fieldSim = new Field2d();
			SmartDashboard.putData("Field", m_fieldSim);
		} else {
			m_fieldSim = null;
		}
	}

	public Field2d getField() {
		return m_fieldSim;
	}

	@Override
	public void periodic() {
		m_odometry.update(
				Rotation2d.fromDegrees(m_gyro1.getHeading()),
				m_leftSide.getEncoder().getDistance(),
				m_rightSide.getEncoder().getDistance());
		m_fieldSim.setRobotPose(getPose());
	}

	@Override
	public void simulationPeriodic() {
		m_drivetrain.updateSimulation(m_gyro1);
	}

	public Pose2d getPose() {
		return m_odometry.getPoseMeters();
	}

	public void arcadeDrive(double fwd, double rot) {
		m_drivetrain.setArcadeDrive(fwd, rot);
	}

	public void setMaxOutput(double maxOutput) {
		m_drivetrain.setMaxOutput(maxOutput);
	}

	/** Zeroes the heading of the robot. */
	public void zeroHeading() {
		m_gyro1.reset();
	}

	/**
	 * Returns the current wheel speeds of the robot.
	 *
	 * @return The current wheel speeds.
	 */
	public DifferentialDriveWheelSpeeds getWheelSpeeds() {
		return new DifferentialDriveWheelSpeeds(
				m_leftSide.getEncoder().getRate(),
				m_rightSide.getEncoder().getRate());
	}

	/**
	 * Controls the left and right sides of the drive directly with voltages.
	 *
	 * @param leftVolts  the commanded left output
	 * @param rightVolts the commanded right output
	 */
	public void tankDriveVolts(double leftVolts, double rightVolts) {
		m_drivetrain.tankDriveVolts(leftVolts, rightVolts);
	}

	/**
	 * Resets the odometry to the specified pose.
	 *
	 * @param pose The pose to which to set the odometry.
	 */
	public void resetOdometry(Pose2d pose) {
		m_drivetrain.resetEncoders();
		m_drivetrain.setSimPose(pose);
		m_odometry.resetPosition(
				Rotation2d.fromDegrees(m_gyro1.getHeading()),
				m_leftSide.getEncoder().getDistance(),
				m_rightSide.getEncoder().getDistance(),
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
		return m_drivetrain.getSimCurrentDrawAmps();
	}
}
