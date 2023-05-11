
import java.util.Collections;
import java.util.List;

import edu.wpi.first.math.geometry.Rotation2d;

import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

 class Translation2d {
	// Private Variables here

  /** Constructs a Translation2d with X and Y components equal to zero. */
  public Translation2d() {
    this(0.0, 0.0);
  }

  /**
   * Constructs a Translation2d with the X and Y components equal to the provided values.
   *
   * @param x The x component of the translation.
   * @param y The y component of the translation.
   */
  public Translation2d(double x, double y) {
  }

  /**
   * Constructs a Translation2d with the provided distance and angle. This is essentially converting
   * from polar coordinates to Cartesian coordinates.
   *
   * @param distance The distance from the origin to the end of the translation.
   * @param angle The angle between the x-axis and the translation vector.
   */
  public Translation2d(double distance, Rotation2d angle) {
  }

  /**
   * Calculates the distance between two translations in 2D space.
   *
   * <p>The distance between translations is defined as √((x₂−x₁)²+(y₂−y₁)²).
   *
   * @param other The translation to compute the distance to.
   * @return The distance between the two translations.
   */
  public double getDistance(Translation2d other) {
		return 0;
  }

  /**
   * Returns the X component of the translation.
   *
   * @return The X component of the translation.
   */
  public double getX() {
    return 0;
  }

  /**
   * Returns the Y component of the translation.
   *
   * @return The Y component of the translation.
   */
  public double getY() {
    return 0;
  }

  /**
   * Returns the norm, or distance from the origin to the translation.
   *
   * @return The norm of the translation.
   */
  public double getNorm() {
    return 0;
  }

  /**
   * Returns the angle this translation forms with the positive X axis.
   *
   * @return The angle of the translation
   */
  public Rotation2d getAngle() {
    return new Rotation2d();
  }

  /**
   * Applies a rotation to the translation in 2D space.
   *
   * <p>This multiplies the translation vector by a counterclockwise rotation matrix of the given
   * angle.
   *
   * <pre>
   * [x_new]   [other.cos, -other.sin][x]
   * [y_new] = [other.sin,  other.cos][y]
   * </pre>
   *
   * <p>For example, rotating a Translation2d of &lt;2, 0&gt; by 90 degrees will return a
   * Translation2d of &lt;0, 2&gt;.
   *
   * @param other The rotation to rotate the translation by.
   * @return The new rotated translation.
   */
  public Translation2d rotateBy(Rotation2d other) {
		return new Translation2d(0,0);
  }

  /**
   * Returns the sum of two translations in 2D space.
   *
   * <p>For example, Translation3d(1.0, 2.5) + Translation3d(2.0, 5.5) = Translation3d{3.0, 8.0).
   *
   * @param other The translation to add.
   * @return The sum of the translations.
   */
  public Translation2d plus(Translation2d other) {
    return new Translation2d();
  }

  /**
   * Returns the difference between two translations.
   *
   * <p>For example, Translation2d(5.0, 4.0) - Translation2d(1.0, 2.0) = Translation2d(4.0, 2.0).
   *
   * @param other The translation to subtract.
   * @return The difference between the two translations.
   */
  public Translation2d minus(Translation2d other) {
    return new Translation2d();
  }

  /**
   * Returns the inverse of the current translation. This is equivalent to rotating by 180 degrees,
   * flipping the point over both axes, or negating all components of the translation.
   *
   * @return The inverse of the current translation.
   */
  public Translation2d unaryMinus() {
    return new Translation2d();
  }

  /**
   * Returns the translation multiplied by a scalar.
   *
   * <p>For example, Translation2d(2.0, 2.5) * 2 = Translation2d(4.0, 5.0).
   *
   * @param scalar The scalar to multiply by.
   * @return The scaled translation.
   */
  public Translation2d times(double scalar) {
    return new Translation2d();
  }

  /**
   * Returns the translation divided by a scalar.
   *
   * <p>For example, Translation3d(2.0, 2.5) / 2 = Translation3d(1.0, 1.25).
   *
   * @param scalar The scalar to multiply by.
   * @return The reference to the new mutated object.
   */
  public Translation2d div(double scalar) {
    return new Translation2d();
  }

  /**
   * Returns the nearest Translation2d from a list of translations.
   *
   * @param translations The list of translations.
   * @return The nearest Translation2d from the list.
   */
  public Translation2d nearest(List<Translation2d> translations) {
    return new Translation2d();
  }

  @Override
  public String toString() {
    return String.format("Translation2d(X: %.2f, Y: %.2f)", 0, 0);
  }
}

public class Translation2dTest {
	// Test each function
	double kEpsilon = 1e-5;

	@Test
	public void testGetDistance() {
		Translation2d t1 = new Translation2d(0, 0);
		Translation2d t2 = new Translation2d(1, 1);
		Assertions.assertEquals(Math.sqrt(2), t1.getDistance(t2), kEpsilon);
	}

	@Test
	public void testGetX() {
		Translation2d t1 = new Translation2d(1, 2);
		Assertions.assertEquals(1, t1.getX(), kEpsilon);
	}

	@Test
	public void testGetY() {
		Translation2d t1 = new Translation2d(1, 2);
		Assertions.assertEquals(2, t1.getY(), kEpsilon);
	}

	@Test
	public void testGetNorm() {
		Translation2d t1 = new Translation2d(3, 4);
		Assertions.assertEquals(5, t1.getNorm(), kEpsilon);
	}

	@Test
	public void testGetAngle() {
		Translation2d t1 = new Translation2d(1, 1);
		Assertions.assertEquals(new Rotation2d(Math.PI / 4), t1.getAngle());
	}

	@Test
	public void testRotateBy() {
		Translation2d t1 = new Translation2d(1, 0);
		Assertions.assertEquals(new Translation2d(0, 1), t1.rotateBy(new Rotation2d(Math.PI / 2)));
	}

	@Test
	public void testPlus() {
		Translation2d t1 = new Translation2d(1, 2);
		Translation2d t2 = new Translation2d(3, 4);
		Assertions.assertEquals(new Translation2d(4, 6), t1.plus(t2));
	}

	@Test
	public void testMinus() {
		Translation2d t1 = new Translation2d(1, 2);
		Translation2d t2 = new Translation2d(3, 4);
		Assertions.assertEquals(new Translation2d(-2, -2), t1.minus(t2));
	}

	@Test
	public void testUnaryMinus() {
		Translation2d t1 = new Translation2d(1, 2);
		Assertions.assertEquals(new Translation2d(-1, -2), t1.unaryMinus());
	}

	@Test
	public void testTimes() {
		Translation2d t1 = new Translation2d(1, 2);
		Assertions.assertEquals(new Translation2d(2, 4), t1.times(2));
	}

	@Test
	public void testDiv() {
		Translation2d t1 = new Translation2d(2, 4);
		Assertions.assertEquals(new Translation2d(1, 2), t1.div(2));
	}

	@Test
	public void testNearest() {
		Translation2d t1 = new Translation2d(1, 1);
		Translation2d t2 = new Translation2d(1, 1.5);
		Translation2d t3 = new Translation2d(3, 3);
		List<Translation2d> list = List.of(t1, t2, t3);

		Assertions.assertEquals(t1, t1.nearest(list));
	}

	@Test
	public void testToString() {
		Translation2d t1 = new Translation2d(1, 2);
		Assertions.assertEquals("Translation2d(X: 1.00, Y: 2.00)", t1.toString());
	}	
}
