package frc.robot.splines;

public class SplineGenerator {

	public static CubicSpline GenerateSpline0(double x0, double x1, double x2, double y0, double y1, double y2)
    {
		double sigma1;

		double h0, h1; //diferencia de x
		double deltaY0, deltaY1; //diferencia de y

        // coeficientes del polinomio cubico del spline0, "a" es el coeficiente de x cubica, "b" el de x cuadrada, "c" el de x y "d" es el termino independiente
		double a0, b0, c0, d0;

        //Setea Deltas
        h0 = x1 - x0;
        h1 = x2 - x1;
        deltaY0 = y1 - y0;
        deltaY1 = y2 - y1;

        //Setea Sigma
        // La primera y segunda fraccion del numerador de la formula para Sigma1
		double fraccionA = deltaY1 / h1;
		double fraccionB = deltaY0 / h0;
        //Resta de fracciones del numerador de la formula de sigma1
		double restaFracciones = fraccionA - fraccionB;
        //suma de las h del denominador de la formula de sigma1
		double sumaH = h0 + h1;
        //Aplicacion de la formula de sigma1
        sigma1 = (6 * restaFracciones) / (2 * sumaH);


        // Todo esto se obtiene aplicando la formula del spline0, que interpola el punto 0 con el punto 1 (primer intervalo)
        a0 = sigma1 / (6 * h0);
        b0 = -((3 * sigma1 * x0) / (6 * h0));
        //Numerador del coeficiente de x de la formula para el spline0
		double numeradorC0 = (3 * sigma1 * x0 * x0) - (sigma1 * h0 * h0) - (6 * y0) + (6 * y1);
        c0 = numeradorC0 / (6 * h0);
		double numeradorD0 = (6 * y0 * x1) - (sigma1 * x0 * x0 * x0) + (sigma1 * h0 * h0 * x0) - (6 * y1 * x0);
        d0 = numeradorD0 / (6 * h0);

        return new CubicSpline(a0, b0, c0, d0);
    }
	
	public static CubicSpline GenerateSpline1(double x0, double x1, double x2, double y0, double y1, double y2)
	    {
			double sigma1;

			double h0, h1; //diferencia de x
			double deltaY0, deltaY1; //diferencia de y

	        // coeficientes del polinomio cubico del spline0, "a" es el coeficiente de x cubica, "b" el de x cuadrada, "c" el de x y "d" es el termino independiente
			double a1, b1, c1, d1;

	        //Setea Deltas
	        h0 = x1 - x0;
	        h1 = x2 - x1;
	        deltaY0 = y1 - y0;
	        deltaY1 = y2 - y1;

	        //Setea Sigma
	        // La primera y segunda fraccion del numerador de la formula para Sigma1
			double fraccionA = deltaY1 / h1;
			double fraccionB = deltaY0 / h0;
	        //Resta de fracciones del numerador de la formula de sigma1
			double restaFracciones = fraccionA - fraccionB;
	        //suma de las h del denominador de la formula de sigma1
			double sumaH = h0 + h1;
	        //Aplicacion de la formula de sigma1
	        sigma1 = (6 * restaFracciones) / (2 * sumaH);

	        //Aplica formula del spline 1
	        a1 = -(sigma1 / (6 * h1));
	        b1 = (3 * sigma1 * x2) / (6 * h1);
			double numeradorC1 = (sigma1 * h1 * h1) + (6 * y2) - (6 * y1) - (3 * sigma1 * x2 * x2);
	        c1 = numeradorC1 / (6 * h1);
			double numeradorD1 = (sigma1 * x2 * x2 * x2) - (sigma1 * h1 * h1 * x2) + (6 * y1 * x2) - (6 * y2 * x1);
	        d1 = numeradorD1 / (6 * h1);

	        return new CubicSpline(a1, b1, c1, d1);
	    }

	public static Derivative DerivateSpline(CubicSpline spline)
	    {
	        
	        // Coeficiente x cuadrada de la derivada del spline0
			double p = (3 * spline.cubicX);
	        //Coeficiente x de la derivada del spline0
			double q = (2 * spline.squareX);
	        //Termino independiente de la derivada del spline0
			double r = spline.x;

	        return new Derivative(p, q, r);
	    }

	public static double angleFromDerivate(Derivative derivative, double xPos) {
		double pendienteActual = (derivative.squareX * xPos * xPos) + (derivative.x * xPos) + derivative.indep;
		System.out.println("m: " + pendienteActual);
		return  (Math.atan(pendienteActual) * 360 / (2 * Math.PI));
	}


}
