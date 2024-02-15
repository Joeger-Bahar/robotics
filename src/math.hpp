#pragma once

#include <iostream>
#include <random>
#include <numeric>
#include <vector>

struct Vector2
{
	Vector2() : x(0.0f), y(0.0f) {}
	Vector2(double x, double y) : x(x), y(y) {}

	void normalize()
	{
		double distance = sqrt(x * x + y * y);

		if (distance)
		{
			x /= distance;
			y /= distance;
		}
	}

	double dot(const Vector2& other) { return this->x * other.x + this->y * other.y; }

	int cross(const Vector2& other) { return (this->x * other.y) - (this->y * other.x); }

	Vector2 operator=(const Vector2& other)
	{
		return { other.x, other.y };
	}

	double length() { return sqrt(this->x * this->x + this->y * this->y); }

	friend std::ostream& operator<<(std::ostream& os, Vector2& v)
	{
		os << v.x << ", " << v.y << "\n";
		return os;
	}

	inline operator bool() const { return (this->x && this->y); }
	inline bool operator!() const { return (!this->x && !this->y); }

	inline bool operator<(const Vector2& other) const { return (this->x < other.x && this->y < other.y); }
	inline bool operator>(const Vector2& other) const { return (this->x > other.x && this->y > other.y); }
	inline bool operator<=(const Vector2& other) const { return (this->x <= other.x && this->y <= other.y); }
	inline bool operator>=(const Vector2& other) const { return (this->x >= other.x && this->y >= other.y); }
	inline bool operator==(const Vector2& other) const { return (this->x == other.x && this->y == other.y); }
	inline bool operator!=(const Vector2& other) const { return (this->x != other.x || this->y != other.y); }

	inline bool operator&&(const Vector2& other) const { return (this->x && this->y && other.x && other.y); }
	inline bool operator||(const Vector2& other) const { return (this->x || this->y || other.x || other.y); }

	inline void operator++() { ++x; ++y; }
	inline void operator++(int s) { ++x; ++y; }
	inline void operator--() { --x; --y; }
	inline void operator--(int s) { --x; --y; }
	inline void operator*=(double s) { x *= s; y *= s; }
	inline void operator/=(double s) { x /= s; y /= s; }
	inline void operator+=(double s) { x += s; y += s; }
	inline void operator-=(double s) { x -= s; y -= s; }
	inline void operator%=(double s) { x = std::fmod(x, s); y = std::fmod(y, s); }

	inline Vector2 operator-() const { return { -x, -y }; }
	inline Vector2 operator+(float a) { return { this->x + a, this->y + a }; }
	inline Vector2 operator-(float a) { return { this->x - a, this->y - a }; }
	inline Vector2 operator*(float a) { return { this->x * a, this->y * a }; }
	inline Vector2 operator/(float a) { return { this->x / a, this->y / a }; }

	union
	{
		double x;
		double w;
	};
	union
	{
		double y;
		double h;
	};
};

inline Vector2 operator+(Vector2 first, Vector2 second)
{
	return { first.x + second.x, first.y + second.y };
}

inline Vector2 operator-(Vector2 first, Vector2 second)
{
	return { first.x - second.x, first.y - second.y };
}

inline Vector2 operator*(Vector2 first, Vector2 second)
{
	return { first.x * second.x, first.y * second.y };
}

inline Vector2 operator/(Vector2 first, Vector2 second)
{
	return { first.x / second.x, first.y / second.y };
}

// Same as Vector2
struct Point
{
	Point() : x(0.0f), y(0.0f) {}
	Point(double x, double y, double dist, double ang) : x(x), y(y), distanceToNextPoint(dist), angleToNextPoint(ang) {}

	void normalize()
	{
		double distance = sqrt(x * x + y * y);

		if (distance)
		{
			x /= distance;
			y /= distance;
		}
	}

	double x, y, distanceToNextPoint, angleToNextPoint;
};

template <>
struct std::hash<Vector2>
{
	std::size_t operator()(const Vector2& vec) const
	{
		using std::size_t;

		return ((std::hash<int>()(vec.x)
			^ (std::hash<int>()(vec.y) << 1)) >> 1);
	}
};
namespace Math
{
	int Random(int lb, int hb)
	{
		return (rand() % (hb - lb) + 1) + lb;
	}
}
struct Rect
{
    Rect(int x, int y, int w, int h) : xy({x, y}), wh({w, h})
    {
        this->x = xy.x;
        this->y = xy.y;
        this->w = wh.w;
        this->h = wh.h;
    }
    Vector2 xy;
    Vector2 wh;
    int x, y;
    int w, h;
};