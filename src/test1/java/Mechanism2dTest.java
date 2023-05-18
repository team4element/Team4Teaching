import org.junit.jupiter.api.BeforeAll;
import org.junit.jupiter.api.Test;

import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.simulation.SimHooks;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

public class Mechanism2dTest {

	@BeforeAll
	public static void setup() {
		DataLogManager.start();
	}

	@Test
	public void BasicMechanism() {
		Mechanism2d mech = new Mechanism2d(10,10);
		MechanismRoot2d root = mech.getRoot("climber", 5, 0);

		MechanismLigament2d elevator = root.append(new MechanismLigament2d("elevator", 5, 90, 6, new Color8Bit(Color.kRed)));
		MechanismLigament2d wrist = elevator.append(new MechanismLigament2d("wrist", 3, 20, 6, new Color8Bit(Color.kPurple)));

		for (int i = 20; i < 500; i++) {
			wrist.setAngle(i);
			SmartDashboard.putData("Mech2d", mech);
			SimHooks.stepTiming(0.01);
		}
	}
}
