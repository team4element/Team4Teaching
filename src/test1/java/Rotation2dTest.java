import org.junit.jupiter.api.Test;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Disabled;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

/**
 * A rotation in a 2D coordinate frame represented by a point on the unit circle (cosine and sine).
 *
 * <p>The angle is continuous, that is if a Rotation2d is constructed with 361 degrees, it will
 * return 361 degrees. This allows algorithms that wouldn't want to see a discontinuity in the
 * rotations as it sweeps past from 360 to 0 on the second time around.
 */
class Rotation2d {
  /** Constructs a Rotation2d with a default angle of 0 degrees. */
  public Rotation2d() {
  }

  /**
   * Constructs a Rotation2d with the given radian value.
   *
   * @param value The value of the angle in radians.
   */
  public Rotation2d(double value) {
  }

  /**
   * Constructs a Rotation2d with the given x and y (cosine and sine) components.
   *
   * @param x The x component or cosine of the rotation.
   * @param y The y component or sine of the rotation.
   */
  public Rotation2d(double x, double y) {
  }

  /**
   * Constructs and returns a Rotation2d with the given radian value.
   *
   * @param radians The value of the angle in radians.
   * @return The rotation object with the desired angle value.
   */
  public static Rotation2d fromRadians(double radians) {
    return new Rotation2d(radians);
  }

  /**
   * Constructs and returns a Rotation2d with the given degree value.
   *
   * @param degrees The value of the angle in degrees.
   * @return The rotation object with the desired angle value.
   */
  public static Rotation2d fromDegrees(double degrees) {
    return new Rotation2d();
  }

  /**
   * Constructs and returns a Rotation2d with the given number of rotations.
   *
   * @param rotations The value of the angle in rotations.
   * @return The rotation object with the desired angle value.
   */
  public static Rotation2d fromRotations(double rotations) {
    return new Rotation2d();
  }

  /**
   * Adds two rotations together, with the result being bounded between -pi and pi.
   *
   * <p>For example, <code>Rotation2d.fromDegrees(30).plus(Rotation2d.fromDegrees(60))</code> equals
   * <code>Rotation2d(Math.PI/2.0)</code>
   *
   * @param other The rotation to add.
   * @return The sum of the two rotations.
   */
  public Rotation2d plus(Rotation2d other) {
    return new Rotation2d();
  }

  /**
   * Subtracts the new rotation from the current rotation and returns the new rotation.
   *
   * <p>For example, <code>Rotation2d.fromDegrees(10).minus(Rotation2d.fromDegrees(100))</code>
   * equals <code>Rotation2d(-Math.PI/2.0)</code>
   *
   * @param other The rotation to subtract.
   * @return The difference between the two rotations.
   */
  public Rotation2d minus(Rotation2d other) {
    return new Rotation2d();
  }

  /**
   * Takes the inverse of the current rotation. This is simply the negative of the current angular
   * value.
   *
   * @return The inverse of the current rotation.
   */
  public Rotation2d unaryMinus() {
    return new Rotation2d();
  }

  /**
   * Multiplies the current rotation by a scalar.
   *
   * @param scalar The scalar.
   * @return The new scaled Rotation2d.
   */
  public Rotation2d times(double scalar) {
		return new Rotation2d();
  }

  /**
   * Divides the current rotation by a scalar.
   *
   * @param scalar The scalar.
   * @return The new scaled Rotation2d.
   */
  public Rotation2d div(double scalar) {
    return new Rotation2d();
  }

  /**
   * Adds the new rotation to the current rotation using a rotation matrix.
   *
   * <p>The matrix multiplication is as follows:
   *
   * <pre>
   * [cos_new]   [other.cos, -other.sin][cos]
   * [sin_new] = [other.sin,  other.cos][sin]
   * value_new = atan2(sin_new, cos_new)
   * </pre>
   *
   * @param other The rotation to rotate by.
   * @return The new rotated Rotation2d.
   */
  public Rotation2d rotateBy(Rotation2d other) {
    return new Rotation2d();
  }

  /**
   * Returns the radian value of the Rotation2d.
   *
   * @return The radian value of the Rotation2d.
   * @see edu.wpi.first.math.MathUtil#angleModulus(double) to constrain the angle within (-pi, pi]
   */
  public double getRadians() {
    return 0;
  }

  /**
   * Returns the degree value of the Rotation2d.
   *
   * @return The degree value of the Rotation2d.
   * @see edu.wpi.first.math.MathUtil#inputModulus(double, double, double) to constrain the angle
   *     within (-180, 180]
   */
  public double getDegrees() {
    return 0;
  }

  /**
   * Returns the number of rotations of the Rotation2d.
   *
   * @return The number of rotations of the Rotation2d.
   */
  public double getRotations() {
    return 0;
  }

  /**
   * Returns the cosine of the Rotation2d.
   *
   * @return The cosine of the Rotation2d.
   */
  public double getCos() {
    return 0;
  }

  /**
   * Returns the sine of the Rotation2d.
   *
   * @return The sine of the Rotation2d.
   */
  public double getSin() {
    return 0;
  }

  /**
   * Returns the tangent of the Rotation2d.
   *
   * @return The tangent of the Rotation2d.
   */
  public double getTan() {
    return 0;
  }

  @Override
  public String toString() {
    return String.format("Rotation2d(Rads: %.2f, Deg: %.2f)", 0, Math.toDegrees(0));
  }
}


@Disabled
public class Rotation2dTest {

	@Test
	public void log() {
		DataLogManager.start();

		DataLog log = DataLogManager.getLog();
		StringLogEntry stringLog = new StringLogEntry(log, "/string");
		stringLog.append("Hello!");
	}
	
	@Test
	public void fromRadians() {
		Rotation2d rotation = Rotation2d.fromRadians(0.5);
		System.out.println(rotation.getRadians());
		Assertions.assertEquals(0.5, rotation.getRadians(), 1E-9);
		Assertions.assertEquals(0.5 / (2 * Math.PI), rotation.getRotations(), 1E-9);
	}

	@Test
	public void fromDegrees() {
		Rotation2d rotation = Rotation2d.fromDegrees(90);
		System.out.println(rotation.getRadians());
		Assertions.assertEquals(Math.PI / 2, rotation.getRadians(), 1E-9);
		Assertions.assertEquals(0.25, rotation.getRotations(), 1E-9);
	}

	@Test
	public void fromRotations() {
		Rotation2d rotation = Rotation2d.fromRotations(0.25);
		System.out.println(rotation.getRadians());
		Assertions.assertEquals(Math.PI / 2, rotation.getRadians(), 1E-9);
		Assertions.assertEquals(0.25, rotation.getRotations(), 1E-9);
	}

	@Test
	public void plus() {
		Rotation2d rotation1 = Rotation2d.fromDegrees(90);
		Rotation2d rotation2 = Rotation2d.fromDegrees(90);
		Rotation2d rotation3 = rotation1.plus(rotation2);
		System.out.println(rotation3.getRadians());
		Assertions.assertEquals(Math.PI, rotation3.getRadians(), 1E-9);
		Assertions.assertEquals(0.5, rotation3.getRotations(), 1E-9);
	}

	@Test
	public void minus() {
		Rotation2d rotation1 = Rotation2d.fromDegrees(90);
		Rotation2d rotation2 = Rotation2d.fromDegrees(90);
		Rotation2d rotation3 = rotation1.minus(rotation2);
		System.out.println(rotation3.getRadians());
		Assertions.assertEquals(0, rotation3.getRadians(), 1E-9);
		Assertions.assertEquals(0, rotation3.getRotations(), 1E-9);
	}

	@Test
	public void unaryMinus() {
		Rotation2d rotation1 = Rotation2d.fromDegrees(90);
		Rotation2d rotation2 = rotation1.unaryMinus();
		System.out.println(rotation2.getRadians());
		Assertions.assertEquals(-Math.PI / 2, rotation2.getRadians(), 1E-9);
		Assertions.assertEquals(-0.25, rotation2.getRotations(), 1E-9);
	}

	@Test
	public void times() {
		Rotation2d rotation1 = Rotation2d.fromDegrees(90);
		Rotation2d rotation2 = rotation1.times(2);
		System.out.println(rotation2.getRadians());
		Assertions.assertEquals(Math.PI, rotation2.getRadians(), 1E-9);
		Assertions.assertEquals(0.5, rotation2.getRotations(), 1E-9);
	}

	@Test
	public void div() {
		Rotation2d rotation1 = Rotation2d.fromDegrees(90);
		Rotation2d rotation2 = rotation1.div(2);
		System.out.println(rotation2.getRadians());
		Assertions.assertEquals(Math.PI / 4, rotation2.getRadians(), 1E-9);
		Assertions.assertEquals(0.125, rotation2.getRotations(), 1E-9);
	}

	@Test
	public void rotateBy() {
		Rotation2d rotation1 = Rotation2d.fromDegrees(90);
		Rotation2d rotation2 = Rotation2d.fromDegrees(90);
		Rotation2d rotation3 = rotation1.rotateBy(rotation2);
		System.out.println(rotation3.getRadians());
		Assertions.assertEquals(Math.PI, rotation3.getRadians(), 1E-9);
		Assertions.assertEquals(0.5, rotation3.getRotations(), 1E-9);
	}

	@Test
	public void getRadians() {
		Rotation2d rotation = Rotation2d.fromDegrees(90);
		System.out.println(rotation.getRadians());
		Assertions.assertEquals(Math.PI / 2, rotation.getRadians(), 1E-9);
	}

	@Test
	public void getDegrees() {
		Rotation2d rotation = Rotation2d.fromDegrees(90);
		System.out.println(rotation.getDegrees());
		Assertions.assertEquals(90, rotation.getDegrees(), 1E-9);
	}

	@Test
	public void getRotations() {
		Rotation2d rotation = Rotation2d.fromDegrees(90);
		System.out.println(rotation.getRotations());
		Assertions.assertEquals(0.25, rotation.getRotations(), 1E-9);
	}

	@Test
	public void getCos() {
		Rotation2d rotation = Rotation2d.fromDegrees(90);
		System.out.println(rotation.getCos());
		Assertions.assertEquals(0, rotation.getCos(), 1E-9);
	}

	@Test
	public void getSin() {
		Rotation2d rotation = Rotation2d.fromDegrees(90);
		System.out.println(rotation.getSin());
		Assertions.assertEquals(1, rotation.getSin(), 1E-9);
	}

	@Test
	public void getTan() {
		Rotation2d rotation = Rotation2d.fromDegrees(45);
		System.out.println(rotation.getTan());
		Assertions.assertEquals(1, rotation.getTan(), 1E-9);
	}

	@Test
	public void testToString() {
		Rotation2d rotation = Rotation2d.fromDegrees(90);
		System.out.println(rotation.toString());
		Assertions.assertEquals("Rotation2d(Rads: 1.57, Deg: 90.00)", rotation.toString());
	}
}
