#include "EpsilonVector.h"

EpsilonVector::EpsilonVector(float x, float y)
: x(x),
y(y)
{
}



EpsilonVector operator*(EpsilonVector v,float s)
{
    return EpsilonVector(v.x * s, v.y * s);
}

EpsilonVector operator*(float s, EpsilonVector& v)
{
    return EpsilonVector(v.x * s, v.y * s);
}
EpsilonVector operator/(EpsilonVector v, float s)
{
    return EpsilonVector(v.x / s, v.y / s);
}
EpsilonVector EpsilonVector::operator+(EpsilonVector b)
{
    return EpsilonVector(x + b.x, y + b.y);
}
EpsilonVector& EpsilonVector::operator+=(EpsilonVector b)
{
    x += b.x;
    y += b.y;

    return *this;
}
EpsilonVector EpsilonVector::operator-(EpsilonVector b)
{
    return EpsilonVector(x - b.x, y - b.y);
}
EpsilonVector& EpsilonVector::operator-=(EpsilonVector b)
{
    x -= b.x;
    y -= b.y;

    return *this;
}
EpsilonVector operator-(EpsilonVector v)
{
    return EpsilonVector(-v.x, -v.y);
}
float EpsilonVector::LengthSquared()
{
    return x * x + y * y;
}

float EpsilonVector::Length()
{
    return sqrt(x * x + y * y);
}

EpsilonVector EpsilonVector::Normalized()
{
    float len = Length();
    return EpsilonVector(x / len, y / len);
}

float EpsilonVector::Dot(EpsilonVector a)
{
    return a.x * x + a.y * y;
}

float EpsilonVector::Cross(EpsilonVector a)
{
    return x * a.y - y * a.x;
}

