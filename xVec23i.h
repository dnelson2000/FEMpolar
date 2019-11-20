
#pragma once
#ifndef _VEC23i_H_
#define _VEC23i_H_


class Vec2i
{
public:
	Vec2i() {}
	Vec2i(const int x, const int y) : m_x(x), m_y(y) {}

	Vec2i(const Vec2i& v) : m_x(v.m_x), m_y(v.m_y) {}
	~Vec2i() {}

	int& operator[](int i) { return (&m_x)[i]; }
	const int& operator[](int i) const  { return (&m_x)[i]; }
	int* ptr() { return &m_x; }
	const int* ptr() const { return &m_x; }

private:
	int m_x;
	int m_y;
};



class Vec3i
{
public:
	Vec3i() {}
	Vec3i(const int x, const int y, const int z) : m_x(x), m_y(y), m_z(z) {}

	Vec3i(const Vec3i& v) : m_x(v.m_x), m_y(v.m_y), m_z(v.m_z) {}
	~Vec3i() {}

	bool operator==(const Vec3i &v) const { return (m_x==v.m_x)&&(m_y==v.m_y)&&(m_z==v.m_z); }

	int& operator[](int i) { return (&m_x)[i]; }
	const int& operator[](int i) const  { return (&m_x)[i]; }
	int* ptr() { return &m_x; }
	const int* ptr() const { return &m_x; }

	inline bool operator<(const Vec3i& v) const
	{
		return (m_x < v.m_x)
			|| (m_x == v.m_x && m_y < v.m_y)
			|| (m_x == v.m_x && m_y == v.m_y && m_z < v.m_z);
	}

	inline bool operator>(const Vec3i v) const
	{
		return m_x > v.m_x
			|| (m_x == v.m_x && m_y > v.m_y)
			|| (m_x == v.m_x && m_y == v.m_y && m_z > v.m_z);
	}

private:
	int m_x;
	int m_y;
	int m_z;
};


#endif

