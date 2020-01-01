package frc.robot.splines;

public class CubicSpline {

	 public double cubicX;
     public double squareX;
     public double x;
     public double indep;

     public CubicSpline(double cubic, double square, double lineal, double term)
     {
         cubicX = cubic;
         squareX = square;
         x = lineal;
         indep = term;
     }
	
}
