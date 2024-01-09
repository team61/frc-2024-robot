package frc.robot;

public class Vector2D {
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

    public static Vector2D scalarMultiply(Vector2D a, double b) {
        return new Vector2D(a.x * b, a.y * b);
    }

    public static Vector2D scalarDivide(Vector2D a, double b) {
        return new Vector2D(a.x / b, a.y / b);
    }

    public double magnitude() {
        return Math.sqrt(x * x + y * y);
    }

    public double theta() {
        return Math.atan2(y, x) / Math.PI * 180;
    }
}
