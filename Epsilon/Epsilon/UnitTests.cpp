#include"UnitTests.h"
void UnitTestVector()
{
    {
        EpsilonVector a(5, 4);
        EpsilonVector b(12, -37);
        EpsilonVector zero(0, 0);
        assert((a + b).x == 17 && (a + b).y == -33 && (a - b).x == -7 && (a - b).y == 41 && (a.Dot(b)) == -88 && (a.Cross(b)) == -233
            && !isnan(zero.Normalized().x) && !isnan(zero.Normalized().y));
    }

    {
        EpsilonVector v(1e-12f, -1e-12f);
        EpsilonVector n = v.Normalized();

        assert(!isnan(n.x));
        assert(!isnan(n.y));
    }

    {
        EpsilonVector v(1e8f, -1e8f);

        float len = v.Length();

        assert(isfinite(len));
    }

    {
        EpsilonVector a(1e6f, 1e6f);
        EpsilonVector b(-1e6f, 1e6f);

        float dot = a.Dot(b);

        assert(isfinite(dot));
    }

    {
        EpsilonVector v(5, 10);
        EpsilonVector r = v * 0.0f;

        assert(Collisions::NearlyEqual(r.x, 0));
        assert(Collisions::NearlyEqual(r.x, 0));
    }
}

void UnitTestAABB()
{
    {
        AABB a(EpsilonVector(0, 0), EpsilonVector(2, 2));
        AABB b(EpsilonVector(1, 1), EpsilonVector(3, 3));
        AABB c(EpsilonVector(0, 0), EpsilonVector(1, 1));
        AABB d(EpsilonVector(1, 0), EpsilonVector(2, 1));
        assert(Collisions::IntersectAABB(a, b) && Collisions::IntersectAABB(c, d));
    }

    {
        AABB a(EpsilonVector( 1,1 ), EpsilonVector( 1,1 ));
        AABB b(EpsilonVector( 0,0 ), EpsilonVector( 2,2 ));
        assert(Collisions::IntersectAABB(a, b));
    }

    {
        AABB a(EpsilonVector(0, 0), EpsilonVector(1, 1));
        AABB b(EpsilonVector(1, 1), EpsilonVector(2, 2));
        assert(Collisions::IntersectAABB(a, b));
    }

    {
        AABB a(EpsilonVector(0, 0), EpsilonVector(10, 10));
        AABB b(EpsilonVector(3, 3), EpsilonVector(5, 5));
        assert(Collisions::IntersectAABB(a, b)&&Collisions::ContainsAABB(a,b));
    }

    {
        AABB a(EpsilonVector(0, 0), EpsilonVector(100, 1e-6f));
        AABB b(EpsilonVector(50, -1), EpsilonVector(60, 1));
        assert(Collisions::IntersectAABB(a, b));
    }

    {
        AABB a(EpsilonVector(-10, -10), EpsilonVector(-1, -1));
        AABB b(EpsilonVector(-5, -5), EpsilonVector(5, 5));
        assert(Collisions::IntersectAABB(a, b));
    }

    {
        AABB a(EpsilonVector(2, 2), EpsilonVector(0, 0));
        AABB b(EpsilonVector(3, 3), EpsilonVector(1, 1));
        AABB c(EpsilonVector(1, 1), EpsilonVector(0, 0));
        AABB d(EpsilonVector(2, 1), EpsilonVector(1, 0));
        assert(Collisions::IntersectAABB(a, b) && Collisions::IntersectAABB(c, d));
    }
}

void UnitTestUpdateMovement() {
    {
        EpsilonBody bd = EpsilonBody::CreateBoxBody(EpsilonVector(0, 0), 0.7f, 0.5f, 4, 4, false, none);
        float dt = 1.0f;
        EpsilonVector gravity(0, 9.8f);
        bd.updateMovement(dt, gravity, 1);
        assert(Collisions::NearlyEqual(bd.position.y, 4.9f));
    }

    {
        EpsilonBody bd = EpsilonBody::CreateBoxBody(EpsilonVector(0, 0), 0.7f, 0.5f, 4, 4, false, none);
        float dt = 1e-8f;
        EpsilonVector gravity(0, 0);
        bd.linearVelocity.x = 10.f;
        bd.updateMovement(dt, gravity, 1);
        assert(isfinite(bd.position.x));
    }

    {
        EpsilonBody bd = EpsilonBody::CreateBoxBody(EpsilonVector(0, 0), 0.7f, 0.5f, 4, 4, false, none);
        float dt = 5.f;
        EpsilonVector gravity(0, 9.8f);
        bd.linearVelocity = EpsilonVector(10,10);
        bd.updateMovement(dt, gravity, 1);
        assert(isfinite(bd.position.x));
    }

    {
        EpsilonBody bd = EpsilonBody::CreateBoxBody(EpsilonVector(0, 0), 0.7f, 0.5f, 4, 4, false, none);
        float dt = 1.f;
        EpsilonVector gravity(0, 0);
        bd.linearVelocity = EpsilonVector(3, 4);
        bd.updateMovement(dt, gravity, 1);
        assert(Collisions::NearlyEqual(bd.position.x,3)&& Collisions::NearlyEqual(bd.position.y, 4));
    }

    {
        EpsilonBody bd = EpsilonBody::CreateBoxBody(EpsilonVector(0, 0), 0.7f, 0.5f, 4, 4, false, none);
        float dt = 0.f;
        EpsilonVector gravity(0, 9.8f);
        bd.linearVelocity = EpsilonVector(3, 4);
        bd.position = EpsilonVector(5, 5);
        bd.updateMovement(dt, gravity, 1);
        assert(Collisions::NearlyEqual(bd.position.x, 5) && Collisions::NearlyEqual(bd.position.y, 5));
    }
}