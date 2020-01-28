package frc.robot.resources;

public class TecbotSensors {

    public static Navx tecbotGyro;

    public static void initializeAllSensors(){

        tecbotGyro = new Navx();

    }

    public static void sensorsPeriodic(){
        tecbotGyro.run();
    }

    public static Navx getTecbotGyro(){
        return tecbotGyro;
    }

    public static double getYaw(){
        return tecbotGyro.getYaw();
    }

}