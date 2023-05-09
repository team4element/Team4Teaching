import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import java.util.List;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveVoltageConstraint;

class Utils {
	public static double deadband(double val, double deadband) {
		if (Math.abs(val) > deadband) {
			return val;
		} else {
			return 0.0;
		}
	}
}
public class PathTest {
	private final double kEpsilon = 1e-5;

	@Test
	public void samplePath() {

		DifferentialDriveKinematics kDriveKinem = new DifferentialDriveKinematics(20);
		var autoVoltageConstraint = new DifferentialDriveVoltageConstraint(
				new SimpleMotorFeedforward(
						0.22,
						1.98,
						0.2),
				kDriveKinem,
				10);

		TrajectoryConfig config =
		new TrajectoryConfig(
						10,
						4)
				// Add kinematics to ensure max speed is actually obeyed
				.setKinematics(kDriveKinem)
				// Anew DifferentialDriveKinematics(20)pply the voltage constraint
				.addConstraint(autoVoltageConstraint);

		Pose2d start = new Pose2d(0,0, new Rotation2d(0));
		List<Translation2d> waypoints = List.of(new Translation2d(1, 1), new Translation2d(2, -1));
		Pose2d end = new Pose2d(3,0, new Rotation2d(0));

		Trajectory example = TrajectoryGenerator.generateTrajectory(
			start, waypoints, end, config
		);
		// assertEquals(Utils.deadband(1, .5), 1, kEpsilon);
		// assertEquals(Utils.deadband(0.5, .1), 0.4444444, kEpsilon);
		
		// assertEquals(Utils.deadband(1, .5), 1, kEpsilon);
		// assertEquals(Utils.deadband(1, .5), 1, kEpsilon);
		// write test for 0.5, 0.1
	}
}
