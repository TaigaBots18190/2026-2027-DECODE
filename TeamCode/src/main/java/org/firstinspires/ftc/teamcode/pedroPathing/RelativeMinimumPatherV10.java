package org.firstinspires.ftc.teamcode.pedroPathing;

class PollenDetection {
    public double tx;
    public double ty;
    public double ta;
    public PollenDetection(double tx, double ty, double ta) {
        this.tx = tx;
        this.ty = ty;
        this.ta = ta;
    }
}

class Coordinates {
    public double x;
    public double y;
    public Coordinates(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public static Coordinates midpointControlPoint(
            Coordinates start,
            Coordinates end,
            Coordinates point) {

        double ctrlX = 2 * point.x - (start.x + end.x) / 2.0;
        double ctrlY = 2 * point.y - (start.y + end.y) / 2.0;

        return new Coordinates(ctrlX, ctrlY);
    }
}