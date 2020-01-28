package frc.robot.resources;

public class Math {

    public static final double PI = 3.141592653589793238462643383279502884197;

    public static double clamp (double p, double mn, double mx){
		return java.lang.Math.max(mn, java.lang.Math.min(p,mx));
	}
	public static double max(double a, double b){
		return java.lang.Math.max(a, b);
	}
	public static double min(double a, double b){
		return java.lang.Math.min(a, b);
	}
	public static double abs(double a){
		return java.lang.Math.abs(a);
	}

	public static double deadZone(double input, double tolerance){
		if(abs(input) < tolerance){
			return 0;
		}
		return input;
	}
}
