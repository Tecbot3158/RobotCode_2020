package frc.robot.resources;

public class Math {

    public static final double PI = 3.141592653589793238462643383279502884197;


    public static double clamp(double p, double mn, double mx) {
        return java.lang.Math.max(mn, java.lang.Math.min(p, mx));
    }

    /**
     * @return returns mathematical a mod b
     */
    public static int module(int a, int b) {
        int mod = a % b;
        if (mod < 0) {
            mod += b;
        }
        return mod;
    }

    public static double max(double a, double b) {
        return java.lang.Math.max(a, b);
    }

    public static double min(double a, double b) {
        return java.lang.Math.min(a, b);
    }

    public static double abs(double a) {
        return java.lang.Math.abs(a);
    }

    public static double sin(double a) {
        return java.lang.Math.sin(a);
    }

    public static double cos(double a) {
        return java.lang.Math.cos(a);
    }

    public static double tan(double a) {
        return java.lang.Math.tan(a);
    }

    public static double atan(double a) {
        return java.lang.Math.atan(a);
    }

    public static double hypot(double a, double b) {
        return java.lang.Math.hypot(a, b);
    }

    public static double toRadians(double a) {
        return java.lang.Math.toRadians(a);
    }

    public static double toDegrees(double a) {
        return java.lang.Math.toDegrees(a);
    }

    public static double sqrt(double a) {
        return java.lang.Math.sqrt(a);
    }

    public static double pow(double a, double b) {
        return java.lang.Math.pow(a, b);
    }

    /**
     * @param r Red value in a range from 0-1
     * @param g Green value in a range from 0-1
     * @param b Blue value in a range from 0-1
     * @return Array with three values: Hue, Saturation, and Value
     */

    public static double[] RGBtoHSV(double r, double g, double b) {
        double[] hsv = {0f, 0f, 0f};
        double minRGB = Math.min(r, Math.min(g, b));
        double maxRGB = Math.max(r, Math.max(g, b));

        double d = (r == minRGB) ? g - b : ((b == minRGB) ? r - g : b - r);
        double h = (r == minRGB) ? 3 : ((b == minRGB) ? 1 : 5);
        hsv[0] = 60 * (h - d / (maxRGB - minRGB));
        hsv[1] = (maxRGB - minRGB) / maxRGB;
        hsv[2] = maxRGB;
        return hsv;
    }

    public static double deadZone(double input, double tolerance) {
        if (abs(input) < tolerance) {
            return 0;
        }
        return input;
    }

    public static double[] convertIntArrayToDoubleArray(int[] source) {
        double[] destinationArray = new double[source.length];
        for (int i = 0; i < source.length; i++) {
            destinationArray[i] = source[i];
        }
        return destinationArray;
    }
}