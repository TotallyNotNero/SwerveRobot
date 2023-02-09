package frc.robot.Utilities;

public class Vector3 {
    public double x,y,z;
    
    public Vector3() {
        x = 0;
        y = 0;
        z = 0;
    }

    public Vector3(double x, double y, double z){
        this.x = x;
        this.y = y;
        this.z = z;
    }

    /**
    * Add another vector to this one
    * @param rhs The vector on the right hand side of the equation
    */
    public static Vector3 add(Vector3 u, Vector3 v){
        return new Vector3(u.x + v.x, u.y + v.y, u.z + v.z);
    }
    /**
    * Subtract another vector from this one
    * @param rhs The vector on the right hand side of the equation
    */
    public static Vector3 sub(Vector3 u, Vector3 v){
        return new Vector3(u.x - v.x, u.y - v.y, u.z - v.z);
    }
    /**
    * Multiply this vector by a scalar
    * @param scale The scalar to multiply the vector by
    */
    public Vector3 scale(double scale){
        return new Vector3(x * scale, y * scale, z * scale);
    }
    /**
    * Divide this vector by a scalar
    * @param scale The scalar to divide the vector by
    */
    public Vector3 div(double scale){
        return new Vector3(x / scale, y / scale, z / scale);
    }
    /**
    * Calculates the magnitude of this vector
    */
    public double mag(){
        return Math.sqrt(x*x + y*y + z*z);
    }
    /**
    * Creates a unit vector with the same direction as this vector
    */
    public Vector3 unitVector(){
        // x/=mag();
        // y/= mag();
        double m = mag();
        return div(m);
    }

    //calculates the cross product of vectors u and v
    public static Vector3 cross(Vector3 u, Vector3 v){
        return new Vector3(u.y * v.z - u.z * v.y, u.z * v.x - u.x * v.z, u.x * v.y - u.y * v.x);
    }
    
    //calculates the dot product of vectors u and v
    public static double dot(Vector3 u, Vector3 v){
        return u.x * v.x + u.y * v.y + u.z * v.z;
    }

    //calculates the angle between vectors u and v in radians
    public static double atan(Vector3 u, Vector3 v){
        double denom = u.mag() * v.mag();
        return Math.acos(dot(u,v) / denom);
    }

    public double getX(){
        return x;
    }

    public double getY(){
        return y;
    }

    public double getZ(){
        return z;
    }
    
    @Override
    public String toString(){
        return "(" + x + ", " + y + ", " + z + ")";
    }
}