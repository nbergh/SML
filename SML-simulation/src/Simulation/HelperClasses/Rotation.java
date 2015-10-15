package Simulation.HelperClasses;

public final class Rotation {
	
	public static double getX(double x, double y, double angle) {
		// Counterclockwise rotation
		return (x*Math.cos(angle) - y*Math.sin(angle));
	}
	
	public static double getY(double x, double y, double angle) {
		// Counterclockwise rotation
		return (x*Math.sin(angle) + y*Math.cos(angle));
	}
	
}
 