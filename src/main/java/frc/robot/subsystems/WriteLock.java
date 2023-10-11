package frc.robot.subsystems;

/**
 * A write lock to prevent race conditions. This lock does not actually 
 * guarantee the memory safety of the contained object, each method call
 * requres that the programmer make a "promise" to follow its rules.
 */
public class WriteLock<T> {
	private T obj;
	private boolean locked;

	/**
	 * Creates a new lock around the given object. The given object mey not be used
	 * again after being passed in here except through the lock for the lock to be
	 * effective.
	 */
	public WriteLock(T obj) {
		this.obj = obj;
	}

	/**
	 * Blockes the current thread until the object is made available.
	 */
	public T lock() {
		while (locked == true) {}
		locked = true;
		return obj;
	}

	/**
	 * Unocks the object, it should not be used after this call.
	 */
	public void unlock() {
		locked = false;
	}

	/**
	 * Unocks the object, it should not be used after this call. Also sets the
	 * lock object to a new reference.
	 */
	public void unlock(T obj) {
		this.obj = obj;
	}
}
