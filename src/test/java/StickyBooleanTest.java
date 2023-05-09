import static org.junit.jupiter.api.Assertions.*;
import org.junit.jupiter.api.Test;

// The following code tests the concept of a sticky boolean, which is a boolean value that remains true once it has been set to true, regardless of future updates to false.

// The StickyBoolean class has three methods:

// update(boolean newVal) takes a boolean value and updates the mVal field to true if newVal is true, then returns mVal.
// get() returns the current value of mVal.
// reset() sets mVal to false.
// The StickyBooleanTest class has two test methods:

// testLatches() tests that StickyBoolean behaves as a sticky boolean by checking that mVal remains true after being set to true, even if it is later set to false.
// testReset() tests that calling reset() sets mVal back to false and that subsequent calls to update() return false.

// If you were to use the above code as a programming exercise as a teaching aid, your students could learn and practice the following programming concepts:

// Class and Object Creation: Students can learn how to create classes in Java and how to instantiate objects from those classes.

// Object-Oriented Programming (OOP) principles: Students can learn about OOP principles such as encapsulation, which is implemented in this code by the use of private instance variables and public methods to manipulate those variables.

// Boolean Variables and Operators: Students can learn about boolean data types in Java and how to use them in conditional statements and logical expressions.

// Unit Testing: Students can learn how to use the JUnit testing framework to write and run unit tests for their code.

// Test-Driven Development (TDD): Students can learn how to use TDD to design and implement code by writing tests first and then writing code to pass those tests.

// State Management: Students can learn how to manage state within an object and how to manipulate that state through public methods.

// Method Overriding: Students can learn how to override a method in a class, such as the toString() method, to provide custom behavior for that method.

// Overall, this exercise can provide students with a practical and hands-on way to learn and practice several important programming concepts in Java.
class StickyBoolean {
	boolean mVal = false;

	public boolean update(boolean newVal) {
		if (newVal) {
			mVal = true;
		}

		return mVal;
	}

	public boolean get() {
		return mVal;
	}

	public void reset() {
		mVal = false;
	}
}

public class StickyBooleanTest {
	@Test
	public void testLatches() {
		StickyBoolean sticky = new StickyBoolean();
		assertFalse(sticky.update(false));
		assertTrue(sticky.update(true));
		assertTrue(sticky.update(false));
		assertTrue(sticky.get());
	}

	@Test
	public void testReset() {
		StickyBoolean sticky = new StickyBoolean();
		
		assertTrue(sticky.update(true));
		sticky.reset();
		assertFalse(sticky.get());
		assertFalse(sticky.update(false));
	}
	
}
