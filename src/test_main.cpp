#include "geometry/bezier/deCasteljau.h"


#include <iostream>


struct Vector3D
{
    Vector3D(float x, float y, float z)
        : x(x), y(y), z(z) {}

    Vector3D operator+(const Vector3D &other) const
    {
        return Vector3D(x + other.x, y + other.y, z + other.z);
    }

    Vector3D operator-(const Vector3D &other) const
    {
        return *this + other * -1;
    }

    Vector3D operator*(double scalar) const
    {
        return Vector3D(scalar*x, scalar*y, scalar*z);
    }

    float x, y, z;
};

Vector3D operator*(double scalar, const Vector3D &vec)
{
    return vec * scalar;
}

std::ostream &operator<<(std::ostream &stream, const Vector3D &vec)
{
    return stream << "(" << vec.x << ", " << vec.y << ", " << vec.z << ")";
}


int test_main()
{

    using namespace std;
    using namespace Geometry;

    vector<Vector3D> points = {
        Vector3D(0, 0, 0),
        Vector3D(1, 0, 0),
        Vector3D(1, 1, 0),
        Vector3D(1, 1, 1)
    };

    cout << "Original points:" << endl;
    cout << points[0] << endl
             << points[1] << endl
             << points[2] << endl
             << points[3] << endl;

    cout << "deCasteljau on original points with t = 0.0, 0.25, 0.7, 1.0" << endl;
    cout << Bezier::deCasteljau(points, 0) << endl;
    cout << Bezier::deCasteljau(points, .25) << endl;
    cout << Bezier::deCasteljau(points, .7) << endl;
    cout << Bezier::deCasteljau(points, 1) << endl;

    cout << "Blossoming test: the output must match the original points..." << endl;
    cout << Bezier::blossom(points, {0, 0, 0}) << endl; // Same as points[0]
    cout << Bezier::blossom(points, {0, 0, 1}) << endl; // Same as points[1]
    cout << Bezier::blossom(points, {0, 1, 1}) << endl; // Same as points[2]
    cout << Bezier::blossom(points, {1, 1, 1}) << endl; // Same as points[3]

    cout << "Attempting subdivision using blossoming..." << endl;
    vector<Vector3D> reparameterized = {
        Bezier::blossom(points, {0, 0, 0}),
        Bezier::blossom(points, {0, 0, 0.5}),
        Bezier::blossom(points, {0, 0.5, 0.5}),
        Bezier::blossom(points, {0.5, 0.5, 0.5})
    };

    cout << reparameterized[0] << endl
             << reparameterized[1] << endl
             << reparameterized[2] << endl
             << reparameterized[3] << endl;

    cout << "Attempting the same subdivision using subdivide()..." << endl;
    reparameterized = Bezier::subdivide(points, 0, 0.5);
    cout << reparameterized[0] << endl
             << reparameterized[1] << endl
             << reparameterized[2] << endl
             << reparameterized[3] << endl;

    cout << "deCasteljau with new curve at t = 0.0, 0.5, 1.4, 2.0" << endl;
    cout << Bezier::deCasteljau(reparameterized, 0.0) << endl;
    cout << Bezier::deCasteljau(reparameterized, 0.5) << endl;
    cout << Bezier::deCasteljau(reparameterized, 1.4) << endl;
    cout << Bezier::deCasteljau(reparameterized, 2.0) << endl;


    return 0;
}
