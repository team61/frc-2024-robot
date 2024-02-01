package frc.robot;

public class Vector2D {
    public static Vector2D zero = new Vector2D();
    public static Vector2D one = new Vector2D(1, 1);
    
    public double x, y;

    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2D() {
        this.x = 0;
        this.y = 0;
    }

    public static Vector2D add(Vector2D a, Vector2D b) {
        return new Vector2D(a.x + b.x, a.y + b.y);
    }

    public static Vector2D subtract(Vector2D a, Vector2D b) {
        return new Vector2D(a.x - b.x, a.y - b.y);
    }

    public static Vector2D scalarMultiply(Vector2D a, double b) {
        return new Vector2D(a.x * b, a.y * b);
    }

    public static Vector2D scalarDivide(Vector2D a, double b) {
        return new Vector2D(a.x / b, a.y / b);
    }

    public static Vector2D rotateByDegrees(Vector2D vector, double angle) {
        double magnitude = vector.magnitude();
        double theta = vector.theta() + angle;
        double rads = theta * Math.PI / 180;
        double x = Math.cos(rads) * magnitude;
        double y = Math.sin(rads) * magnitude;
        return new Vector2D(x, y);
    }

    public static Vector2D lerp(Vector2D a, Vector2D b, double t) {
        return new Vector2D(Utils.lerp(a.x, b.x, t), Utils.lerp(a.y, b.y, t));
    }

    public double magnitude() {
        return Math.sqrt(x * x + y * y);
    }

    public Vector2D normalized() {
        return scalarDivide(this, magnitude());
    }

    public double theta() {
        return Math.atan2(y, x) / Math.PI * 180;
    }

    public String toString() {
        return "(" + Utils.round(x) + ", " + Utils.round(y) + ")";
    }
}
