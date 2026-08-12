package application;

import application.Vector3D;


public class Vector3D {
    public double x;
    public double y;
    public double z;

    // Constructors
    public Vector3D() {
        this(0, 0, 0);
    }

    public Vector3D(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    //Setter
    public void set(double x, double y, double z) {
        this.x = x;
        this.y = y;
        this.z = z;
    }

    // ---------------- Vector Operations ----------------

    public Vector3D add(Vector3D other) {
        return new Vector3D(this.x + other.x,
                            this.y + other.y,
                            this.z + other.z);
    }

    public Vector3D subtract(Vector3D other) {
        return new Vector3D(this.x - other.x,
                            this.y - other.y,
                            this.z - other.z);
    }

    public Vector3D multiply(double scalar) {
        return new Vector3D(this.x * scalar,
                            this.y * scalar,
                            this.z * scalar);
    }

    public Vector3D divide(double scalar) {
        return new Vector3D(this.x / scalar,
                            this.y / scalar,
                            this.z / scalar);
    }

    public double dot(Vector3D other) {
        return this.x * other.x +
               this.y * other.y +
               this.z * other.z;
    }

    // Cross product 
    public Vector3D cross(Vector3D other) {
        return new Vector3D(
            this.y * other.z - this.z * other.y,
            this.z * other.x - this.x * other.z,
            this.x * other.y - this.y * other.x
        );
    }

    public double magnitude() {
        return Math.sqrt(x * x + y * y + z * z);
    }

    public Vector3D normalize() {
        double mag = magnitude();
        if (mag == 0) return new Vector3D(0, 0, 0);
        return divide(mag);
    }

   

    public void addInPlace(Vector3D other) {
        this.x += other.x;
        this.y += other.y;
        this.z += other.z;
    }

    public void multiplyInPlace(double scalar) {
        this.x *= scalar;
        this.y *= scalar;
        this.z *= scalar;
    }

    @Override
    public String toString() {
        return String.format("(%.3f, %.3f, %.3f)", x, y, z);
    }
}

