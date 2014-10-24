#ifndef __Math_Vector_included_h_
#define __Math_Vector_included_h_

#define _USE_MATH_DEFINES
#include <math.h>
#include <iostream>

#define MV_POINT_ELEM_ZERO		1e-6

template <class SCALAR> class mvQuaternion;
template <class SCALAR> class mvMatrix44;

#define mvPoint4	mvQuaternion

template <class SCALAR>
class mvPoint2 {
public:
	SCALAR x, y;

	mvPoint2( )
		: x(0), y(0) { }

	mvPoint2(SCALAR _x, SCALAR _y)
		: x(_x), y(_y) { }

	// unary -
	mvPoint2<SCALAR> operator - ( ) const {
		return mvPoint2<SCALAR>(-x, -y);
	}

	// vector/vector arithmatic
	mvPoint2<SCALAR> operator + (mvPoint2<SCALAR> v) const {
		return mvPoint2<SCALAR>(x+v.x, y+v.y);
	}
	mvPoint2<SCALAR> operator - (mvPoint2<SCALAR> v) const {
		return mvPoint2<SCALAR>(x-v.x, y-v.y);
	}
	mvPoint2<SCALAR> operator += (mvPoint2<SCALAR> v) {
		return (*this) = (*this) + v;
	}
	mvPoint2<SCALAR> operator -= (mvPoint2<SCALAR> v) {
		return (*this) = (*this) - v;
	}
	SCALAR operator ^ (mvPoint2<SCALAR> v) const {	// dot product
		return (x*v.x) + (y*v.y);
	}
	SCALAR operator * (mvPoint2<SCALAR> v) const {	// cross product
		return (x*v.y) - (y*v.x);
	}

	// vector/scalar arithmatic
	mvPoint2<SCALAR> operator * (SCALAR s) const {
		return mvPoint2<SCALAR>(x*s, y*s);
	}
	mvPoint2<SCALAR> operator / (SCALAR s) const {
		return mvPoint2<SCALAR>(x/s, y/s);
	}
	mvPoint2<SCALAR> operator *= (SCALAR s) const {
		return (*this) = (*this)*s;
	}
	mvPoint2<SCALAR> operator /= (SCALAR s) const {
		return (*this) = (*this)/s;
	}	

	SCALAR getSqrSize( ) const {
		return x*x + y*y;
	}
	SCALAR getSize( ) const {
		return (SCALAR)sqrt(x*x + y*y);
	}
	bool isZero( ) const {
		return (x < MV_POINT_ELEM_ZERO && x > -MV_POINT_ELEM_ZERO &&
			y < MV_POINT_ELEM_ZERO && y > -MV_POINT_ELEM_ZERO);
	}

	mvPoint2<SCALAR> normalize( ) const {
		if (isZero( )) {
			return mvPoint2<SCALAR>( );
		} else {
			return (*this)/getSize( );
		}
	}

	// single precision
	// rotate vector using cosine angle
	mvPoint2<SCALAR> rotate(float c, float s) const {
		return mvPoint2<SCALAR>((SCALAR)(x*c - y*s), (SCALAR)(x*s + y*c));
	}
	mvPoint2<SCALAR> rotate(float theta) const {
		return rotate((float)cos(theta), (float)sin(theta));
	}

	// double precision
	mvPoint2<SCALAR> rotate(double c, double s) const {
		return mvPoint2<SCALAR>((SCALAR)(x*c - y*s), (SCALAR)(x*s + y*c));
	}
	mvPoint2<SCALAR> rotate(double theta) const {
		return rotate((double)cos(theta), (double)sin(theta));
	}

};

template <class SCALAR>
class mvPoint3 {
public:
	SCALAR x, y, z;

	mvPoint3( )
		: x(0), y(0), z(0) { }

	mvPoint3(SCALAR _x, SCALAR _y, SCALAR _z)
		: x(_x), y(_y), z(_z) { }

	mvPoint3(mvPoint2<SCALAR> p): x(p.x), y(p.y), z(0) { }
	mvPoint3(mvPoint2<SCALAR> p, float _z): x(p.x), y(p.y), z(_z) { }
	mvPoint3(mvPoint4<SCALAR> p);

	// unary -
	mvPoint3<SCALAR> operator - ( ) const {
		return mvPoint3<SCALAR>(-x, -y, -z);
	}

	// vector/vector arithmatic
	mvPoint3<SCALAR> operator + (mvPoint3<SCALAR> v) const {
		return mvPoint3<SCALAR>(x+v.x, y+v.y, z+v.z);
	}
	mvPoint3<SCALAR> operator - (mvPoint3<SCALAR> v) const {
		return mvPoint3<SCALAR>(x-v.x, y-v.y, z-v.z);
	}
	mvPoint3<SCALAR> operator += (mvPoint3<SCALAR> v) {
		return (*this) = (*this) + v;
	}
	mvPoint3<SCALAR> operator -= (mvPoint3<SCALAR> v) {
		return (*this) = (*this) - v;
	}
	SCALAR operator ^ (mvPoint3<SCALAR> v) const {	// dot product
		return (x*v.x) + (y*v.y) + (z*v.z);
	}
	mvPoint3<SCALAR> operator * (mvPoint3<SCALAR> v) const {	// cross product
		return mvPoint3<SCALAR>(	y*v.z - z*v.y,
									z*v.x - x*v.z,
									x*v.y - y*v.x 
								);
	}

	// vector/scalar arithmatic
	mvPoint3<SCALAR> operator * (SCALAR s) const {
		return mvPoint3<SCALAR>(x*s, y*s, z*s);
	}
	mvPoint3<SCALAR> operator / (SCALAR s) const {
		return mvPoint3<SCALAR>(x/s, y/s, z/s);
	}
	mvPoint3<SCALAR> operator *= (SCALAR s) const {
		return (*this) = (*this)*s;
	}
	mvPoint3<SCALAR> operator /= (SCALAR s) const {
		return (*this) = (*this)/s;
	}	


	SCALAR getSqrSize( ) const {
		return x*x + y*y + z*z;
	}
	SCALAR getSize( ) const {
		return (SCALAR)sqrt(x*x + y*y + z*z);
	}
	bool isZero( ) const {
		return (x < MV_POINT_ELEM_ZERO && x > -MV_POINT_ELEM_ZERO &&
			y < MV_POINT_ELEM_ZERO && y > -MV_POINT_ELEM_ZERO &&
			z < MV_POINT_ELEM_ZERO && z > -MV_POINT_ELEM_ZERO);
	}

	mvPoint3<SCALAR> normalize( ) const {
		if (isZero( )) {
			return mvPoint3<SCALAR>( );
		} else {
			return (*this)/getSize( );
		}
	}

	// rotate vector around an axis (normalized)
	mvPoint3<SCALAR> rotate(float theta, mvPoint3<SCALAR> v) const;

};

// Quaternion & Point4
template <class SCALAR>
class mvQuaternion {
public:
	SCALAR x, y, z, w;

	mvQuaternion( ):x(0), y(0), z(0), w(0) { }

	// create quaternion from rotation matrix
	mvQuaternion(mvMatrix44<SCALAR> m);

	// create quaternion from rotation scale and rotation axis
	mvQuaternion(SCALAR a, mvPoint3<SCALAR> n);
	// create quaternion
	mvQuaternion(mvPoint3<SCALAR> p, SCALAR _w)
		: x(p.x), y(p.y), z(p.z), w(_w) { }

	// create quaternion by finding a rotation from 2 normal vectors n0 to n1
	mvQuaternion(mvPoint3<SCALAR> n0, mvPoint3<SCALAR> n1);

	mvQuaternion(SCALAR _x, SCALAR _y, SCALAR _z, SCALAR _w)
		:x(_x), y(_y), z(_z), w(_w) { }

	// conjugate
	mvQuaternion<SCALAR> operator * ( ) const {
		return mvQuaternion<SCALAR>(-x, -y, -z, w);
	}

	mvQuaternion<SCALAR> operator * (mvQuaternion<SCALAR> q) const {
		mvPoint3<SCALAR> v1(x, y, z);
		mvPoint3<SCALAR> v2(q.x, q.y, q.z);
		return mvQuaternion<SCALAR>(v2*w + v1*q.w + v1*v2, w*q.w - (v1^v2));
	}

};

template <class SCALAR>
class mvMatrix44 {
public:
	SCALAR e[16];

	mvMatrix44( ) { }

	// create a uniform scaling matrix (s=1 for an identity matrix)
	mvMatrix44(SCALAR s);
	// create a translation matrix (t=0 vector for an identity matrix)
	mvMatrix44(mvPoint3<SCALAR> t);
	// create a rotation matrix from a quaternion
	mvMatrix44(mvQuaternion<SCALAR> q);

	inline SCALAR& operator [ ] (int i)
		{ return e[i]; }
	inline SCALAR& operator ( ) (int r, int c)
		{ return e[r + (c << 2)]; }

	mvMatrix44<SCALAR> operator * (mvMatrix44<SCALAR> m);
	mvPoint4<SCALAR> operator * (mvPoint4<SCALAR> p);
	mvPoint3<SCALAR> operator * (mvPoint3<SCALAR> n);

	mvMatrix44<SCALAR> transpose( );
};

// wrapper rotation matrix
template <class SCALAR>
class mvRotMatrix43 {
public:
	const SCALAR* e;
	mvRotMatrix43(const SCALAR* _e): e(_e) { }

	inline SCALAR operator [ ] (int i) const
		{ return e[i]; }
	inline SCALAR operator ( ) (int r, int c) const
		{ return e[r + (c << 2)]; }
	mvPoint3<SCALAR> operator * (mvPoint3<SCALAR> n) const;
};

typedef mvPoint2<int> mvPoint2i;
typedef mvPoint2<float> mvPoint2f;
typedef mvPoint2<double> mvPoint2d;

typedef mvPoint3<int> mvPoint3i;
typedef mvPoint3<float> mvPoint3f;
typedef mvPoint3<double> mvPoint3d;

typedef mvQuaternion<float> mvQuaternionf, mvPoint4f;
typedef mvQuaternion<double> mvQuaterniond, mvPoint4d;

typedef mvMatrix44<float> mvMatrix44f;
typedef mvMatrix44<double> mvMatrix44d;

typedef mvRotMatrix43<float> mvRotMatrix43f;
typedef mvRotMatrix43<double> mvRotMatrix43d;

// Point3 implementation ////////////////////////////////////////////////////////////////

// initialization from point4
template <class SCALAR>
mvPoint3<SCALAR>::mvPoint3(mvPoint4<SCALAR> p)
	: x(p.x), y(p.y), z(p.z) {
}

// rotation around an axis
template <class SCALAR>
mvPoint3<SCALAR> mvPoint3<SCALAR>::rotate(float theta, mvPoint3<SCALAR> v) const {
	mvQuaternion<SCALAR> qRot(theta, v);
	return mvPoint3<SCALAR>(qRot * mvQuaternion<SCALAR>(*this, 0) * (*qRot));
}

// Quaternion implementation ////////////////////////////////////////////////////////////

// instantiate a quaternion from rotation matrix
template <class SCALAR>
mvQuaternion<SCALAR>::mvQuaternion(mvMatrix44<SCALAR> m) {
	SCALAR T = m[0] + m[5] + m[10] + 1;
	if (T > 0) {
		SCALAR S=(SCALAR)(0.5/sqrt(T));
		w= 0.25/S;
		x=(m(2, 1) - m(1, 2)) * S;
		y=(m(0, 2) - m(2, 0)) * S;
		z=(m(1, 0) - m(0, 1)) * S;
	} else {
		float 
			SX=sqrt( 1.0 + m[0] - m[5] - m[10] ) * 2, 
			SY=sqrt( 1.0 + m[5] - m[0] - m[10] ) * 2, 
			SZ=sqrt( 1.0 + m[10] - m[0] - m[5] ) * 2;

		if (SX > SZ) {
			if (SX > SY) {	// SX greatest
				x=0.5/SX;
				y=(m(0, 1) + m(1, 0))/SX;
				z=(m(0, 2) + m(2, 0))/SX;
				w=(m(1, 2) + m(2, 1))/SX;
			} else {		// SY greatest
				x=(m(0, 1) + m(1, 0))/SY;
				y=0.5/SY;
				z=(m(1, 2) + m(2, 1))/SY;
				w=(m(0, 2) + m(2, 0))/SY;
			}
		} else {			// SZ greatest
			x=(m(0, 2) + m(2, 0))/SZ;
			y=(m(1, 2) + m(2, 1))/SZ;
			z=0.5/SZ;
			w=(m(0, 1) + m(1, 0))/SZ;
		}
	}
}

// instantiate a quaternion from rotation scale and point
template <class SCALAR>
mvQuaternion<SCALAR>::mvQuaternion(SCALAR a, mvPoint3<SCALAR> n) {
	SCALAR sin_a = (SCALAR)sin( a / 2 );
    x    = n.x * sin_a;
    y    = n.y * sin_a;
    z    = n.z * sin_a;
    w    = (SCALAR)cos( a / 2 );
}

// create quaternion by finding a rotation from nonzero vectors n0 to n1
template <class SCALAR>
mvQuaternion<SCALAR>::mvQuaternion(mvPoint3<SCALAR> n0, mvPoint3<SCALAR> n1) {
	SCALAR c = n0^n1;
	if (c < -1.f) c = -1.f;
	if (c > 1.f) c = 1.f;
	{
		SCALAR a = (SCALAR)acos(c);
		mvPoint3<SCALAR> n = (n0*n1).normalize( );

		SCALAR sin_a = (SCALAR)sin( a / 2 );
		x    = n.x * sin_a;
		y    = n.y * sin_a;
		z    = n.z * sin_a;
		w    = (SCALAR)cos( a / 2 );
	}
}


// RotMatrix43 implementation //////////////////////////////////////////////////////////////

template <class SCALAR>
mvPoint3<SCALAR> mvRotMatrix43<SCALAR>::operator * (mvPoint3<SCALAR> n) const {
	return mvPoint3f( 
			operator( )(0, 0)*n.x + operator( )(0, 1)*n.y + operator( )(0, 2)*n.z,
			operator( )(1, 0)*n.x + operator( )(1, 1)*n.y + operator( )(1, 2)*n.z,
			operator( )(2, 0)*n.x + operator( )(2, 1)*n.y + operator( )(2, 2)*n.z
		);
}

// Matrix44 implementation //////////////////////////////////////////////////////////////

// create a uniform scaling matrix
template <class SCALAR>
mvMatrix44<SCALAR>::mvMatrix44(SCALAR s) {
	for (int r = 0; r < 4; r++)
		for (int c = 0; c < 4; c++)
			operator ( ) (r, c) = ((r==c)? s : 0);
}

// create a translation matrix
template <class SCALAR>
mvMatrix44<SCALAR>::mvMatrix44(mvPoint3<SCALAR> t) {
	*this = mvMatrix44<SCALAR>(1);
	operator( )(0, 3) = t.x;
	operator( )(1, 3) = t.y;
	operator( )(2, 3) = t.z;
}

// instantiation from quaternion
template <class SCALAR>
mvMatrix44<SCALAR>::mvMatrix44(mvQuaternion<SCALAR> q) {
	SCALAR 
		xx      = q.x * q.x,
		xy      = q.x * q.y,
		xz      = q.x * q.z,
		xw      = q.x * q.w,
		yy      = q.y * q.y,
		yz      = q.y * q.z,
		yw      = q.y * q.w,
		zz      = q.z * q.z,
		zw      = q.z * q.w;

	operator( )(0,0) = 1 - 2 * ( yy + zz );
    operator( )(0,1) =     2 * ( xy - zw );
    operator( )(0,2) =     2 * ( xz + yw );

    operator( )(1,0) =     2 * ( xy + zw );
    operator( )(1,1) = 1 - 2 * ( xx + zz );
    operator( )(1,2) =     2 * ( yz - xw );

    operator( )(2,0) =     2 * ( xz - yw );
    operator( )(2,1) =     2 * ( yz + xw );
    operator( )(2,2) = 1 - 2 * ( xx + yy );

    operator( )(0,3) = operator( )(1,3) = 
	operator( )(2,3) = operator( )(3,0) = 
	operator( )(3,1) = operator( )(3,2) = 0;

    operator( )(3,3) = 1;
}

// matrix multiplication
template <class SCALAR>
mvMatrix44<SCALAR> mvMatrix44<SCALAR>::operator * (mvMatrix44<SCALAR> m) {
	mvMatrix44<SCALAR> result;
	for (int r = 0; r < 4; r++)
		for (int c = 0; c < 4; c++)
				result(r, c) = 
					operator( )(r, 0) * m(0, c) +
					operator( )(r, 1) * m(1, c) +
					operator( )(r, 2) * m(2, c) +
					operator( )(r, 3) * m(3, c);
	return result;
}

// homogeneous transformation
template <class SCALAR>
mvPoint4<SCALAR> mvMatrix44<SCALAR>::operator * (mvPoint4<SCALAR> p) {
	return mvPoint4f( 
			operator( )(0, 0)*p.x + operator( )(0, 1)*p.y + 
			operator( )(0, 2)*p.z + operator( )(0, 3)*p.w,

			operator( )(1, 0)*p.x + operator( )(1, 1)*p.y + 
			operator( )(1, 2)*p.z + operator( )(1, 3)*p.w,

			operator( )(2, 0)*p.x + operator( )(2, 1)*p.y + 
			operator( )(2, 2)*p.z + operator( )(2, 3)*p.w,

			operator( )(3, 0)*p.x + operator( )(3, 1)*p.y + 
			operator( )(3, 2)*p.z + operator( )(3, 3)*p.w
		);
}

// homogeneous transformation (w component = 0)
template <class SCALAR>
mvPoint3<SCALAR> mvMatrix44<SCALAR>::operator * (mvPoint3<SCALAR> n) {
	return mvPoint3<SCALAR>( 
			operator( )(0, 0)*n.x + operator( )(0, 1)*n.y + operator( )(0, 2)*n.z,
			operator( )(1, 0)*n.x + operator( )(1, 1)*n.y + operator( )(1, 2)*n.z,
			operator( )(2, 0)*n.x + operator( )(2, 1)*n.y + operator( )(2, 2)*n.z
		);
}

// matrix transposition
template <class SCALAR>
mvMatrix44<SCALAR> mvMatrix44<SCALAR>::transpose( ) {
	mvMatrix44<SCALAR> m;
	for (int r = 0; r < 4; r++)
		for (int c = 0; c < 4; c++)
			m(c, r) = operator ( ) (r, c);
	return m;
}


#endif