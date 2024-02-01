package frc.robot;

public class Vector3D {
    public static Vector3D zero = new Vector3D();
    public static Vector3D one = new Vector3D(1, 1, 1);
    
    public double x, y, z;

    public Vector3D(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    public Vector3D() {
        this.x = 0;
        this.y = 0;
        this.z = 0;
    }

    public static Vector3D add(Vector3D a, Vector3D b) {
        return new Vector3D(a.x + b.x, a.y + b.y, a.z + b.z);
    }

    public static Vector3D subtract(Vector3D a, Vector3D b) {
        return new Vector3D(a.x - b.x, a.y - b.y, a.z - b.z);
    }

    public static Vector3D scalarMultiply(Vector3D a, double b) {
        return new Vector3D(a.x * b, a.y * b, a.z * b);
    }

    public static Vector3D scalarDivide(Vector3D a, double b) {
        return new Vector3D(a.x / b, a.y / b, a.z / b);
    }

    public static Vector3D lerp(Vector3D a, Vector3D b, double t) {
        return new Vector3D(Utils.lerp(a.x, b.x, t), Utils.lerp(a.y, b.y, t), Utils.lerp(a.z, b.z, t));
    }

    public double magnitude() {
        return Math.sqrt(x * x + y * y + z * z);
    }

    public Vector3D normalized() {
        return scalarDivide(this, magnitude());
    }

    public String toString() {
        return "(" + Utils.round(x) + ", " + Utils.round(y) + ", " + Utils.round(z) + ")";
    }
}
