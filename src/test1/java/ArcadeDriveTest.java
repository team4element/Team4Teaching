import org.junit.jupiter.api.AfterEach;
import org.junit.jupiter.api.BeforeEach;
import org.junit.jupiter.api.Test;
import org.mockito.ArgumentCaptor;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.mockito.Mockito.*;

class Motor implements MotorController {
	public void set(double speed) {
		System.out.println("Motor set to " + speed);
	}

	@Override
	public void disable() {
		// TODO Auto-generated method stub
	}

	@Override
	public double get() {
		// TODO Auto-generated method stub
		return 0;
	}

	@Override
	public boolean getInverted() {
		// TODO Auto-generated method stub
		return false;
	}

	@Override
	public void setInverted(boolean isInverted) {
		// TODO Auto-generated method stub

	}

	@Override
	public void stopMotor() {
		// TODO Auto-generated method stub
	}
}
public class ArcadeDriveTest {

	WPI_TalonFX leftMotor;
	WPI_TalonFX rightMotor;
	ArgumentCaptor<Double> leftMotorCaptor;
	ArgumentCaptor<Double> rightMotorCaptor;
	DifferentialDrive drive;
	
	// junit before each test, create a new instance of the class
	@BeforeEach
	public void setup() {
		leftMotor = mock(WPI_TalonFX.class);
		rightMotor = mock(WPI_TalonFX.class);
		leftMotorCaptor = ArgumentCaptor.forClass(Double.class);
		rightMotorCaptor = ArgumentCaptor.forClass(Double.class);
		drive = new DifferentialDrive(leftMotor, rightMotor);
	}
	

	private void SetAndTest (double x, double y, double left, double right) {
		drive.arcadeDrive(x, y);
		verify(leftMotor).set(leftMotorCaptor.capture());
		verify(rightMotor).set(rightMotorCaptor.capture());
		System.out.println("Left motor set to " + leftMotorCaptor.getValue());
		System.out.println("Right motor set to " + rightMotorCaptor.getValue());
		assertEquals(left, leftMotorCaptor.getValue(), "Left motor was " + leftMotorCaptor.getValue() + " instead of " + left);
		assertEquals(right, rightMotorCaptor.getValue(), "Right motor was " + rightMotorCaptor.getValue() + " instead of " + right);
	}

	@Test
	public void testArcadeDrive() {
		SetAndTest(0.0, 1, -1.0, 1.0);
	}

	@Test
	public void doNothing() {
		SetAndTest(0.0, 0.0, 0.0, 0.0);
	}

	@AfterEach
	public void teardown() {
		drive.close();
	}
}
