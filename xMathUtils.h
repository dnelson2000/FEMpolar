
#pragma once
#ifndef _MATH_UTILS_H_
#define _MATH_UTILS_H_

namespace FEM {}

#ifndef max
#define max(a,b) (((a) > (b)) ? (a) : (b))
#endif
#ifndef min
#define min(a,b) (((a) < (b)) ? (a) : (b))
#endif

inline float max3( float v0, float v1, float v2 ) {
    return max(max(v0,v1),v2);
}

#include "shapesUtils/StlUtils.h"
#include "shapesUtils/Vec3.h"
#include "shapesUtils/VecN.h"

using namespace FEM;

#include <vector>
#include <math.h>

#define M_PI 3.14159265358979323846

namespace xMath {

class Vector3
{
    float mX;
    float mY;
    float mZ;

public:
    // Default constructor; does no initialization
    // 
    inline Vector3( ) { };

    // Copy a 3-D vector
    // 
    inline Vector3( const Vector3 & vec );

    // Construct a 3-D vector from x, y, and z elements
    // 
    inline Vector3( float x, float y, float z );

    // Set all elements of a 3-D vector to the same scalar value
    // 
    explicit inline Vector3( float scalar );

    // Assign one 3-D vector to another
    // 
    inline Vector3 & operator =( const Vector3 & vec );

    // Set an x, y, or z element of a 3-D vector by index
    // 
    inline Vector3 & setElem( int idx, float value );

    // Get an x, y, or z element of a 3-D vector by index
    // 
    inline float getElem( int idx ) const;

    // Subscripting operator to set or get an element
    // 
    inline float & operator []( int idx );

    // Subscripting operator to get an element
    // 
    inline float operator []( int idx ) const;

    // Add two 3-D vectors
    // 
    inline const Vector3 operator +( const Vector3 & vec ) const;

    // Subtract a 3-D vector from another 3-D vector
    // 
    inline const Vector3 operator -( const Vector3 & vec ) const;

    // Multiply a 3-D vector by a scalar
    // 
    inline const Vector3 operator *( float scalar ) const;

    // Divide a 3-D vector by a scalar
    // 
    inline const Vector3 operator /( float scalar ) const;

    // Perform compound assignment and addition with a 3-D vector
    // 
    inline Vector3 & operator +=( const Vector3 & vec );

    // Perform compound assignment and subtraction by a 3-D vector
    // 
    inline Vector3 & operator -=( const Vector3 & vec );

    // Perform compound assignment and multiplication by a scalar
    // 
    inline Vector3 & operator *=( float scalar );

    // Perform compound assignment and division by a scalar
    // 
    inline Vector3 & operator /=( float scalar );

    // Negate all elements of a 3-D vector
    // 
    inline const Vector3 operator -( ) const;

    // Construct x axis
    // 
    static inline const Vector3 xAxis( );

    // Construct y axis
    // 
    static inline const Vector3 yAxis( );

    // Construct z axis
    // 
    static inline const Vector3 zAxis( );

};

// Multiply a 3-D vector by a scalar
// 
inline const Vector3 operator *( float scalar, const Vector3 & vec );

// Compute the dot product of two 3-D vectors
// 
inline float dot( const Vector3 & vec0, const Vector3 & vec1 );

// Compute the square of the length of a 3-D vector
// 
inline float lengthSqr( const Vector3 & vec );

// Compute the length of a 3-D vector
// 
inline float length( const Vector3 & vec );



inline float & Vector3::operator []( int idx )
{
    return *(&mX + idx);
}

inline float Vector3::operator []( int idx ) const
{
    return *(&mX + idx);
}

inline const Vector3 Vector3::operator +( const Vector3 & vec ) const
{
    return Vector3(
        ( mX + vec.mX ),
        ( mY + vec.mY ),
        ( mZ + vec.mZ )
    );
}

inline const Vector3 Vector3::operator -( const Vector3 & vec ) const
{
    return Vector3(
        ( mX - vec.mX ),
        ( mY - vec.mY ),
        ( mZ - vec.mZ )
    );
}


inline const Vector3 Vector3::operator *( float scalar ) const
{
    return Vector3(
        ( mX * scalar ),
        ( mY * scalar ),
        ( mZ * scalar )
    );
}

inline Vector3 & Vector3::operator +=( const Vector3 & vec )
{
    *this = *this + vec;
    return *this;
}

inline Vector3 & Vector3::operator -=( const Vector3 & vec )
{
    *this = *this - vec;
    return *this;
}

inline Vector3 & Vector3::operator *=( float scalar )
{
    *this = *this * scalar;
    return *this;
}

inline const Vector3 Vector3::operator /( float scalar ) const
{
    return Vector3(
        ( mX / scalar ),
        ( mY / scalar ),
        ( mZ / scalar )
    );
}

inline Vector3 & Vector3::operator /=( float scalar )
{
    *this = *this / scalar;
    return *this;
}

inline const Vector3 Vector3::operator -( ) const
{
    return Vector3(
        -mX,
        -mY,
        -mZ
    );
}

inline const Vector3 operator *( float scalar, const Vector3 & vec )
{
    return vec * scalar;
}


class Vector4
{
    float mX;
    float mY;
    float mZ;
    float mW;

public:
    // Default constructor; does no initialization
    // 
    inline Vector4( ) { };

    // Copy a 4-D vector
    // 
    inline Vector4( const Vector4 & vec );

    // Construct a 4-D vector from x, y, z, and w elements
    // 
    inline Vector4( float x, float y, float z, float w );

    // Construct a 4-D vector from a 3-D vector and a scalar
    // 
    inline Vector4( const Vector3 & xyz, float w );

    // Copy x, y, and z from a 3-D vector into a 4-D vector, and set w to 0
    // 
    explicit inline Vector4( const Vector3 & vec );

    // Set all elements of a 4-D vector to the same scalar value
    // 
    explicit inline Vector4( float scalar );

    // Assign one 4-D vector to another
    // 
    inline Vector4 & operator =( const Vector4 & vec );

    // Set the x, y, and z elements of a 4-D vector
    // NOTE: 
    // This function does not change the w element.
    // 
    inline Vector4 & setXYZ( const Vector3 & vec );

    // Get the x, y, and z elements of a 4-D vector
    // 
    inline const Vector3 getXYZ( ) const;

    // Set the x element of a 4-D vector
    // 
    inline Vector4 & setX( float x );

    // Set the y element of a 4-D vector
    // 
    inline Vector4 & setY( float y );

    // Set the z element of a 4-D vector
    // 
    inline Vector4 & setZ( float z );

    // Set the w element of a 4-D vector
    // 
    inline Vector4 & setW( float w );

    // Get the x element of a 4-D vector
    // 
    inline float getX( ) const;

    // Get the y element of a 4-D vector
    // 
    inline float getY( ) const;

    // Get the z element of a 4-D vector
    // 
    inline float getZ( ) const;

    // Get the w element of a 4-D vector
    // 
    inline float getW( ) const;

    // Set an x, y, z, or w element of a 4-D vector by index
    // 
    inline Vector4 & setElem( int idx, float value );

    // Get an x, y, z, or w element of a 4-D vector by index
    // 
    inline float getElem( int idx ) const;

    // Subscripting operator to set or get an element
    // 
    inline float & operator []( int idx );

    // Subscripting operator to get an element
    // 
    inline float operator []( int idx ) const;

    // Add two 4-D vectors
    // 
    inline const Vector4 operator +( const Vector4 & vec ) const;

    // Subtract a 4-D vector from another 4-D vector
    // 
    inline const Vector4 operator -( const Vector4 & vec ) const;

    // Multiply a 4-D vector by a scalar
    // 
    inline const Vector4 operator *( float scalar ) const;

    // Divide a 4-D vector by a scalar
    // 
    inline const Vector4 operator /( float scalar ) const;

    // Perform compound assignment and addition with a 4-D vector
    // 
    inline Vector4 & operator +=( const Vector4 & vec );

    // Perform compound assignment and subtraction by a 4-D vector
    // 
    inline Vector4 & operator -=( const Vector4 & vec );

    // Perform compound assignment and multiplication by a scalar
    // 
    inline Vector4 & operator *=( float scalar );

    // Perform compound assignment and division by a scalar
    // 
    inline Vector4 & operator /=( float scalar );

    // Negate all elements of a 4-D vector
    // 
    inline const Vector4 operator -( ) const;

    // Construct x axis
    // 
    static inline const Vector4 xAxis( );

    // Construct y axis
    // 
    static inline const Vector4 yAxis( );

    // Construct z axis
    // 
    static inline const Vector4 zAxis( );

    // Construct w axis
    // 
    static inline const Vector4 wAxis( );

};



class xMatrix3
{
    Vector3 mCol0;
    Vector3 mCol1;
    Vector3 mCol2;

public:
    // Default constructor; does no initialization
    // 
    inline xMatrix3( ) { };

    // Copy a 3x3 xMatrix
    // 
    inline xMatrix3( const xMatrix3 & mat );

    // Construct a 3x3 xMatrix containing the specified columns
    // 
    inline xMatrix3( const Vector3 & col0, const Vector3 & col1, const Vector3 & col2 );
    
    // Set all elements of a 3x3 xMatrix to the same scalar value
    // 
    explicit inline xMatrix3( float scalar );

    // Assign one 3x3 xMatrix to another
    // 
    inline xMatrix3 & operator =( const xMatrix3 & mat );

    // Set column 0 of a 3x3 xMatrix
    // 
    inline xMatrix3 & setCol0( const Vector3 & col0 );

    // Set column 1 of a 3x3 xMatrix
    // 
    inline xMatrix3 & setCol1( const Vector3 & col1 );

    // Set column 2 of a 3x3 xMatrix
    // 
    inline xMatrix3 & setCol2( const Vector3 & col2 );

    // Get column 0 of a 3x3 xMatrix
    // 
    inline const Vector3 getCol0( ) const;

    // Get column 1 of a 3x3 xMatrix
    // 
    inline const Vector3 getCol1( ) const;

    // Get column 2 of a 3x3 xMatrix
    // 
    inline const Vector3 getCol2( ) const;

    // Set the column of a 3x3 xMatrix referred to by the specified index
    // 
    inline xMatrix3 & setCol( int col, const Vector3 & vec );

    // Set the row of a 3x3 xMatrix referred to by the specified index
    // 
    inline xMatrix3 & setRow( int row, const Vector3 & vec );

    // Get the column of a 3x3 xMatrix referred to by the specified index
    // 
    inline const Vector3 getCol( int col ) const {
        return *(&mCol0 + col);
    }

    // Get the row of a 3x3 xMatrix referred to by the specified index
    // 
    inline const Vector3 getRow( int row ) const;

    // Subscripting operator to set or get a column
    // 
    inline Vector3 & operator []( int col );

    // Subscripting operator to get a column
    // 
    inline const Vector3 operator []( int col ) const;

    // Set the element of a 3x3 xMatrix referred to by column and row indices
    // 
    inline xMatrix3 & setElem( int col, int row, float val );

    // Get the element of a 3x3 xMatrix referred to by column and row indices
    // 
    inline float getElem( int col, int row ) const
    {
        return this->getCol( col )[row];
    }

    // Add two 3x3 matrices
    // 
    inline const xMatrix3 operator +( const xMatrix3 & mat ) const;

    // Subtract a 3x3 xMatrix from another 3x3 xMatrix
    // 
    inline const xMatrix3 operator -( const xMatrix3 & mat ) const;

    // Negate all elements of a 3x3 xMatrix
    // 
    inline const xMatrix3 operator -( ) const;

    // Multiply a 3x3 xMatrix by a scalar
    // 
    inline const xMatrix3 operator *( float scalar ) const;

    // Multiply a 3x3 xMatrix by a 3-D vector
    // 
    inline const Vector3 operator *( const Vector3 & vec ) const;

    // Multiply two 3x3 matrices
    // 
    inline const xMatrix3 operator *( const xMatrix3 & mat ) const;

    // Perform compound assignment and addition with a 3x3 xMatrix
    // 
    inline xMatrix3 & operator +=( const xMatrix3 & mat );

    // Perform compound assignment and subtraction by a 3x3 xMatrix
    // 
    inline xMatrix3 & operator -=( const xMatrix3 & mat );

    // Perform compound assignment and multiplication by a scalar
    // 
    inline xMatrix3 & operator *=( float scalar );

    // Perform compound assignment and multiplication by a 3x3 xMatrix
    // 
    inline xMatrix3 & operator *=( const xMatrix3 & mat );

    // Construct an identity 3x3 xMatrix
    // 
    static inline const xMatrix3 identity( ) {
        xMatrix3 tmp;
        tmp.mCol0 = Vector3(1,0,0);
        tmp.mCol1 = Vector3(0,1,0);
        tmp.mCol2 = Vector3(0,0,1);
        return tmp;
    }

    // Construct a 3x3 xMatrix to rotate around the x axis
    // 
    static inline const xMatrix3 rotationX( float radians );

    // Construct a 3x3 xMatrix to rotate around the y axis
    // 
    static inline const xMatrix3 rotationY( float radians );

    // Construct a 3x3 xMatrix to rotate around the z axis
    // 
    static inline const xMatrix3 rotationZ( float radians );

    // Construct a 3x3 xMatrix to rotate around the x, y, and z axes
    // 
    static inline const xMatrix3 rotationZYX( const Vector3 & radiansXYZ );

    // Construct a 3x3 xMatrix to rotate around a unit-length 3-D vector
    // 
    static inline const xMatrix3 rotation( float radians, const Vector3 & unitVec );
    
    // Construct a 3x3 xMatrix to perform scaling
    // 
    static inline const xMatrix3 scale( const Vector3 & scaleVec );

};

// Multiply a 3x3 xMatrix by a scalar
// 
inline const xMatrix3 operator *( float scalar, const xMatrix3 & mat );

// Append (post-multiply) a scale transformation to a 3x3 xMatrix
// NOTE: 
// Faster than creating and multiplying a scale transformation xMatrix.
// 
inline const xMatrix3 appendScale( const xMatrix3 & mat, const Vector3 & scaleVec );

// Prepend (pre-multiply) a scale transformation to a 3x3 xMatrix
// NOTE: 
// Faster than creating and multiplying a scale transformation xMatrix.
// 
inline const xMatrix3 prependScale( const Vector3 & scaleVec, const xMatrix3 & mat );

// Multiply two 3x3 matrices per element
// 
inline const xMatrix3 mulPerElem( const xMatrix3 & mat0, const xMatrix3 & mat1 );

// Compute the absolute value of a 3x3 xMatrix per element
// 
inline const xMatrix3 absPerElem( const xMatrix3 & mat );

// Transpose of a 3x3 xMatrix
// 
inline const xMatrix3 transpose( const xMatrix3 & mat );

// Compute the inverse of a 3x3 xMatrix
// NOTE: 
// Result is unpredictable when the determinant of mat is equal to or near 0.
// 
inline const xMatrix3 inverse( const xMatrix3 & mat );


// Determinant of a 3x3 xMatrix
// 
inline float determinant( const xMatrix3 & mat );

// Conditionally select between two 3x3 matrices
// 
inline const xMatrix3 select( const xMatrix3 & mat0, const xMatrix3 & mat1, bool select1 );


// A 4x4 xMatrix in array-of-structures format
//
class xMatrix4
{
    Vector4 mCol0;
    Vector4 mCol1;
    Vector4 mCol2;
    Vector4 mCol3;

public:
    // Default constructor; does no initialization
    // 
    inline xMatrix4( ) { };

    // Copy a 4x4 xMatrix
    // 
    inline xMatrix4( const xMatrix4 & mat );

    // Construct a 4x4 xMatrix containing the specified columns
    // 
    inline xMatrix4( const Vector4 & col0, const Vector4 & col1, const Vector4 & col2, const Vector4 & col3 )
    {
        mCol0 = col0;
        mCol1 = col1;
        mCol2 = col2;
        mCol3 = col3;
    }
    
    // Construct a 4x4 xMatrix from a 3x3 xMatrix and a 3-D vector
    // 
    inline xMatrix4( const xMatrix3 & mat, const Vector3 & translateVec );

    // Set all elements of a 4x4 xMatrix to the same scalar value
    // 
    explicit inline xMatrix4( float scalar );

    // Assign one 4x4 xMatrix to another
    // 
    inline xMatrix4 & operator =( const xMatrix4 & mat );

    // Set the upper-left 3x3 subxMatrix
    // NOTE: 
    // This function does not change the bottom row elements.
    // 
    inline xMatrix4 & setUpper3x3( const xMatrix3 & mat3 );

    // Get the upper-left 3x3 subxMatrix of a 4x4 xMatrix
    // 
    inline const xMatrix3 getUpper3x3( ) const;

    // Set translation component
    // NOTE: 
    // This function does not change the bottom row elements.
    // 
    inline xMatrix4 & setTranslation( const Vector3 & translateVec );

    // Get the translation component of a 4x4 xMatrix
    // 
    inline const Vector3 getTranslation( ) const;

    // Set column 0 of a 4x4 xMatrix
    // 
    inline xMatrix4 & setCol0( const Vector4 & col0 );

    // Set column 1 of a 4x4 xMatrix
    // 
    inline xMatrix4 & setCol1( const Vector4 & col1 );

    // Set column 2 of a 4x4 xMatrix
    // 
    inline xMatrix4 & setCol2( const Vector4 & col2 );

    // Set column 3 of a 4x4 xMatrix
    // 
    inline xMatrix4 & setCol3( const Vector4 & col3 );

    // Get column 0 of a 4x4 xMatrix
    // 
    inline const Vector4 getCol0( ) const;

    // Get column 1 of a 4x4 xMatrix
    // 
    inline const Vector4 getCol1( ) const;

    // Get column 2 of a 4x4 xMatrix
    // 
    inline const Vector4 getCol2( ) const;

    // Get column 3 of a 4x4 xMatrix
    // 
    inline const Vector4 getCol3( ) const;

    // Set the column of a 4x4 xMatrix referred to by the specified index
    // 
    inline xMatrix4 & setCol( int col, const Vector4 & vec );

    // Set the row of a 4x4 xMatrix referred to by the specified index
    // 
    inline xMatrix4 & setRow( int row, const Vector4 & vec );

    // Get the column of a 4x4 xMatrix referred to by the specified index
    // 
    inline const Vector4 getCol( int col ) const {
        return *(&mCol0 + col);
    }

    // Get the row of a 4x4 xMatrix referred to by the specified index
    // 
    inline const Vector4 getRow( int row ) const;

    // Subscripting operator to set or get a column
    // 
    inline Vector4 & operator []( int col );

    // Subscripting operator to get a column
    // 
    inline const Vector4 operator []( int col ) const;

    // Set the element of a 4x4 xMatrix referred to by column and row indices
    // 
    inline xMatrix4 & setElem( int col, int row, float val );

    // Get the element of a 4x4 xMatrix referred to by column and row indices
    // 
    inline float getElem( int col, int row ) const
    {
        return this->getCol( col )[row];
    }

    // Add two 4x4 matrices
    // 
    inline const xMatrix4 operator +( const xMatrix4 & mat ) const;

    // Subtract a 4x4 xMatrix from another 4x4 xMatrix
    // 
    inline const xMatrix4 operator -( const xMatrix4 & mat ) const;

    // Negate all elements of a 4x4 xMatrix
    // 
    inline const xMatrix4 operator -( ) const;

    // Multiply a 4x4 xMatrix by a scalar
    // 
    inline const xMatrix4 operator *( float scalar ) const;

    // Multiply a 4x4 xMatrix by a 4-D vector
    // 
    inline const Vector4 operator *( const Vector4 & vec ) const;

    // Multiply a 4x4 xMatrix by a 3-D vector
    // 
    inline const Vector4 operator *( const Vector3 & vec ) const;
    
    // Multiply two 4x4 matrices
    // 
    inline const xMatrix4 operator *( const xMatrix4 & mat ) const;
    
    // Perform compound assignment and addition with a 4x4 xMatrix
    // 
    inline xMatrix4 & operator +=( const xMatrix4 & mat );

    // Perform compound assignment and subtraction by a 4x4 xMatrix
    // 
    inline xMatrix4 & operator -=( const xMatrix4 & mat );

    // Perform compound assignment and multiplication by a scalar
    // 
    inline xMatrix4 & operator *=( float scalar );

    // Perform compound assignment and multiplication by a 4x4 xMatrix
    // 
    inline xMatrix4 & operator *=( const xMatrix4 & mat );
    
    // Construct an identity 4x4 xMatrix
    // 
    static inline const xMatrix4 identity( );

    // Construct a 4x4 xMatrix to rotate around the x axis
    // 
    static inline const xMatrix4 rotationX( float radians );

    // Construct a 4x4 xMatrix to rotate around the y axis
    // 
    static inline const xMatrix4 rotationY( float radians );

    // Construct a 4x4 xMatrix to rotate around the z axis
    // 
    static inline const xMatrix4 rotationZ( float radians );

    // Construct a 4x4 xMatrix to rotate around the x, y, and z axes
    // 
    static inline const xMatrix4 rotationZYX( const Vector3 & radiansXYZ );

    // Construct a 4x4 xMatrix to rotate around a unit-length 3-D vector
    // 
    static inline const xMatrix4 rotation( float radians, const Vector3 & unitVec );

    // Construct a 4x4 xMatrix to perform scaling
    // 
    static inline const xMatrix4 scale( const Vector3 & scaleVec );

    // Construct a 4x4 xMatrix to perform translation
    // 
    static inline const xMatrix4 translation( const Vector3 & translateVec );


};

inline const Vector3 cross( const Vector3 & vec0, const Vector3 & vec1 )
{
    return Vector3(
        ( ( vec0[1] * vec1[2] ) - ( vec0[2] * vec1[1] ) ),
        ( ( vec0[2] * vec1[0] ) - ( vec0[0] * vec1[2] ) ),
        ( ( vec0[0] * vec1[1] ) - ( vec0[1] * vec1[0] ) )
    );
}

inline const xMatrix3 outer( const Vector3 & tfrm0, const Vector3 & tfrm1 )
{
    return xMatrix3(
        ( tfrm0 * tfrm1[0] ),
        ( tfrm0 * tfrm1[1] ),
        ( tfrm0 * tfrm1[2] )
    );
}

inline const Vector3 xMatrix3::getCol0( ) const
{
    return mCol0;
}

inline const Vector3 xMatrix3::getCol1( ) const
{
    return mCol1;
}

inline const Vector3 xMatrix3::getCol2( ) const
{
    return mCol2;
}


inline const xMatrix3 inverse( const xMatrix3 & mat )
{
    Vector3 tmp0, tmp1, tmp2;
    float detinv;
    tmp0 = cross( mat.getCol1(), mat.getCol2() );
    tmp1 = cross( mat.getCol2(), mat.getCol0() );
    tmp2 = cross( mat.getCol0(), mat.getCol1() );
    detinv = ( 1.0f / xMath::dot( mat.getCol2(), tmp2 ) );
    return xMatrix3(
        Vector3( ( tmp0[0] * detinv ), ( tmp1[0] * detinv ), ( tmp2[0] * detinv ) ),
        Vector3( ( tmp0[1] * detinv ), ( tmp1[1] * detinv ), ( tmp2[1] * detinv ) ),
        Vector3( ( tmp0[2] * detinv ), ( tmp1[2] * detinv ), ( tmp2[2] * detinv ) )
    );
}

inline xMatrix3 & xMatrix3::operator *=( const xMatrix3 & mat )
{
    *this = *this * mat;
    return *this;
}

inline xMatrix3 & xMatrix3::operator -=( const xMatrix3 & mat )
{
    *this = *this - mat;
    return *this;
}


inline const xMatrix3 transpose( const xMatrix3 & mat )
{
    return xMatrix3(
        Vector3( mat.getCol0()[0], mat.getCol1()[0], mat.getCol2()[0] ),
        Vector3( mat.getCol0()[1], mat.getCol1()[1], mat.getCol2()[1] ),
        Vector3( mat.getCol0()[2], mat.getCol1()[2], mat.getCol2()[2] )
    );
}



inline const Vector3 xMatrix3::operator *( const Vector3 & vec ) const
{
    return Vector3(
        ( ( ( mCol0[0] * vec[0] ) + ( mCol1[0] * vec[1] ) ) + ( mCol2[0] * vec[2] ) ),
        ( ( ( mCol0[1] * vec[0] ) + ( mCol1[1] * vec[1] ) ) + ( mCol2[1] * vec[2] ) ),
        ( ( ( mCol0[2] * vec[0] ) + ( mCol1[2] * vec[1] ) ) + ( mCol2[2] * vec[2] ) )
    );
}



inline const xMatrix3 xMatrix3::operator *( const xMatrix3 & mat ) const
{
    return xMatrix3(
        ( *this * mat.mCol0 ),
        ( *this * mat.mCol1 ),
        ( *this * mat.mCol2 )
    );
}


inline const xMatrix3 xMatrix3::operator -( const xMatrix3 & mat ) const
{
    return xMatrix3(
        ( mCol0 - mat.mCol0 ),
        ( mCol1 - mat.mCol1 ),
        ( mCol2 - mat.mCol2 )
    );
}
inline xMatrix3 & xMatrix3::operator =( const xMatrix3 & mat )
{
    mCol0 = mat.mCol0;
    mCol1 = mat.mCol1;
    mCol2 = mat.mCol2;
    return *this;
}


inline xMatrix3 & xMatrix3::setCol( int col, const Vector3 & vec )
{
    *(&mCol0 + col) = vec;
    return *this;
}

inline xMatrix3 & xMatrix3::setRow( int row, const Vector3 & vec )
{
    mCol0[ row]= vec[ 0 ];
    mCol1[ row]= vec[ 1 ];
    mCol2[ row]= vec[ 2 ];
    return *this;
}

inline xMatrix4 & xMatrix4::setCol( int col, const Vector4 & vec )
{
    *(&mCol0 + col) = vec;
    return *this;
}

inline xMatrix4 & xMatrix4::setRow( int row, const Vector4 & vec )
{
    mCol0[ row]= vec[ 0 ];
    mCol1[ row]= vec[ 1 ];
    mCol2[ row]= vec[ 2 ];
    mCol3[ row]= vec[ 3 ];
    return *this;
}

inline xMatrix3::xMatrix3( const xMatrix3 & mat )
{
    mCol0 = mat.mCol0;
    mCol1 = mat.mCol1;
    mCol2 = mat.mCol2;
}

inline xMatrix3::xMatrix3( float scalar )
{
    mCol0 = Vector3( scalar,scalar,scalar );
    mCol1 = Vector3( scalar,scalar,scalar );
    mCol2 = Vector3( scalar,scalar,scalar );
}
inline xMatrix3::xMatrix3( const Vector3 & _col0, const Vector3 & _col1, const Vector3 & _col2 )
{
    mCol0 = _col0;
    mCol1 = _col1;
    mCol2 = _col2;
}

inline xMatrix3 & xMatrix3::operator +=( const xMatrix3 & mat )
{
    *this = *this + mat;
    return *this;
}


inline Vector3 & xMatrix3::operator []( int col )
{
    return *(&mCol0 + col);
}

inline const Vector3 xMatrix3::operator []( int col ) const
{
    return *(&mCol0 + col);
}



inline const xMatrix3 xMatrix3::operator *( float scalar ) const
{
    return xMatrix3(
        ( mCol0 * scalar ),
        ( mCol1 * scalar ),
        ( mCol2 * scalar )
    );
}

inline xMatrix3 & xMatrix3::operator *=( float scalar )
{
    *this = *this * scalar;
    return *this;

}

inline const Vector4 xMatrix4::getCol0( ) const
{
    return mCol0;
}

inline const Vector4 xMatrix4::getCol1( ) const
{
    return mCol1;
}

inline const Vector4 xMatrix4::getCol2( ) const
{
    return mCol2;
}

inline const Vector4 xMatrix4::getCol3( ) const
{
    return mCol3;
}

inline float determinant( const xMatrix3 & mat )
{
    return dot( mat.getCol2(), cross( mat.getCol0(), mat.getCol1() ) );
}


inline const xMatrix3 operator *( float scalar, const xMatrix3 & mat )
{
    return mat * scalar;
}


inline xMatrix4::xMatrix4( const xMatrix4 & mat )
{
    mCol0 = mat.mCol0;
    mCol1 = mat.mCol1;
    mCol2 = mat.mCol2;
    mCol3 = mat.mCol3;
}

inline const xMatrix3 xMatrix3::operator +( const xMatrix3 & mat ) const
{
    return xMatrix3(
        ( mCol0 + mat.mCol0 ),
        ( mCol1 + mat.mCol1 ),
        ( mCol2 + mat.mCol2 )
    );
}

inline const xMatrix3 xMatrix3::operator -( ) const
{
    return xMatrix3(
        ( Vector3(-mCol0[0],-mCol0[1],-mCol0[2]) ),
        ( Vector3(-mCol1[0],-mCol1[1],-mCol1[2]) ),
        ( Vector3(-mCol2[0],-mCol2[1],-mCol2[2]) )
    );
}

inline const xMatrix4 xMatrix4::operator *( float scalar ) const
{
    return xMatrix4(
        ( mCol0 * scalar ),
        ( mCol1 * scalar ),
        ( mCol2 * scalar ),
        ( mCol3 * scalar )
    );
}

inline const Vector3 normalize( const Vector3 & vec )
{
    float lenSqr, lenInv;
    lenSqr = lengthSqr( vec );
    lenInv = ( 1.0f / sqrtf( lenSqr ) );
    return Vector3(
        ( vec[0] * lenInv ),
        ( vec[1] * lenInv ),
        ( vec[2] * lenInv )
    );
}

inline Vector3::Vector3( const Vector3 & vec )
{
    mX = vec.mX;
    mY = vec.mY;
    mZ = vec.mZ;
}

inline Vector3 & Vector3::operator =( const Vector3 & vec )
{
    mX = vec.mX;
    mY = vec.mY;
    mZ = vec.mZ;
    return *this;
}

inline Vector3::Vector3( float _x, float _y, float _z )
{
    mX = _x;
    mY = _y;
    mZ = _z;
}

inline Vector3::Vector3( float scalar )
{
    mX = scalar;
    mY = scalar;
    mZ = scalar;
}


inline float dot( const Vector3 & vec0, const Vector3 & vec1 )
{
    float result;
    result = ( vec0[0] * vec1[0] );
    result = ( result + ( vec0[1] * vec1[1] ) );
    result = ( result + ( vec0[2] * vec1[2] ) );
    return result;
}

inline float lengthSqr( const Vector3 & vec )
{
    float result;
    result = ( vec[0] * vec[0] );
    result = ( result + ( vec[1] * vec[1] ) );
    result = ( result + ( vec[2] * vec[2] ) );
    return result;
}

inline float length( const Vector3 & vec )
{
    return sqrtf( lengthSqr( vec ) );
}


inline float & Vector4::operator []( int idx )
{
    return *(&mX + idx);
}

inline float Vector4::operator []( int idx ) const
{
    return *(&mX + idx);
}

inline Vector4::Vector4( const Vector4 & vec )
{
    mX = vec.mX;
    mY = vec.mY;
    mZ = vec.mZ;
    mW = vec.mW;
}

inline Vector4::Vector4( float _x, float _y, float _z, float _w )
{
    mX = _x;
    mY = _y;
    mZ = _z;
    mW = _w;
}

inline const Vector4 Vector4::operator *( float scalar ) const
{
    return Vector4(
        ( mX * scalar ),
        ( mY * scalar ),
        ( mZ * scalar ),
        ( mW * scalar )
    );
}

inline Vector4 & Vector4::operator =( const Vector4 & vec )
{
    mX = vec.mX;
    mY = vec.mY;
    mZ = vec.mZ;
    mW = vec.mW;
    return *this;
}

inline const xMatrix4 inverse( const xMatrix4 & mat )
{
    Vector4 res0, res1, res2, res3;
    float mA, mB, mC, mD, mE, mF, mG, mH, mI, mJ, mK, mL, mM, mN, mO, mP, tmp0, tmp1, tmp2, tmp3, tmp4, tmp5, detInv;
    mA = mat.getCol0()[0];
    mB = mat.getCol0()[1];
    mC = mat.getCol0()[2];
    mD = mat.getCol0()[3];
    mE = mat.getCol1()[0];
    mF = mat.getCol1()[1];
    mG = mat.getCol1()[2];
    mH = mat.getCol1()[3];
    mI = mat.getCol2()[0];
    mJ = mat.getCol2()[1];
    mK = mat.getCol2()[2];
    mL = mat.getCol2()[3];
    mM = mat.getCol3()[0];
    mN = mat.getCol3()[1];
    mO = mat.getCol3()[2];
    mP = mat.getCol3()[3];
    tmp0 = ( ( mK * mD ) - ( mC * mL ) );
    tmp1 = ( ( mO * mH ) - ( mG * mP ) );
    tmp2 = ( ( mB * mK ) - ( mJ * mC ) );
    tmp3 = ( ( mF * mO ) - ( mN * mG ) );
    tmp4 = ( ( mJ * mD ) - ( mB * mL ) );
    tmp5 = ( ( mN * mH ) - ( mF * mP ) );
    res0[0] = ( ( ( mJ * tmp1 ) - ( mL * tmp3 ) ) - ( mK * tmp5 ) );
    res0[1] = ( ( ( mN * tmp0 ) - ( mP * tmp2 ) ) - ( mO * tmp4 ) );
    res0[2] = ( ( ( mD * tmp3 ) + ( mC * tmp5 ) ) - ( mB * tmp1 ) );
    res0[3] = ( ( ( mH * tmp2 ) + ( mG * tmp4 ) ) - ( mF * tmp0 ) );
    detInv = ( 1.0f / ( ( ( ( mA * res0[0] ) + ( mE * res0[1] ) ) + ( mI * res0[2] ) ) + ( mM * res0[3] ) ) );
    res1[0] = ( mI * tmp1 );
    res1[1] = ( mM * tmp0 );
    res1[2] = ( mA * tmp1 );
    res1[3] = ( mE * tmp0 );
    res3[0] = ( mI * tmp3 );
    res3[1] = ( mM * tmp2 );
    res3[2] = ( mA * tmp3 );
    res3[3] = ( mE * tmp2 );
    res2[0] = ( mI * tmp5 );
    res2[1] = ( mM * tmp4 );
    res2[2] = ( mA * tmp5 );
    res2[3] = ( mE * tmp4 );
    tmp0 = ( ( mI * mB ) - ( mA * mJ ) );
    tmp1 = ( ( mM * mF ) - ( mE * mN ) );
    tmp2 = ( ( mI * mD ) - ( mA * mL ) );
    tmp3 = ( ( mM * mH ) - ( mE * mP ) );
    tmp4 = ( ( mI * mC ) - ( mA * mK ) );
    tmp5 = ( ( mM * mG ) - ( mE * mO ) );
    res2[0] = ( ( ( mL * tmp1 ) - ( mJ * tmp3 ) ) + res2[0] );
    res2[1] = ( ( ( mP * tmp0 ) - ( mN * tmp2 ) ) + res2[1] );
    res2[2] = ( ( ( mB * tmp3 ) - ( mD * tmp1 ) ) - res2[2] );
    res2[3] = ( ( ( mF * tmp2 ) - ( mH * tmp0 ) ) - res2[3] );
    res3[0] = ( ( ( mJ * tmp5 ) - ( mK * tmp1 ) ) + res3[0] );
    res3[1] = ( ( ( mN * tmp4 ) - ( mO * tmp0 ) ) + res3[1] );
    res3[2] = ( ( ( mC * tmp1 ) - ( mB * tmp5 ) ) - res3[2] );
    res3[3] = ( ( ( mG * tmp0 ) - ( mF * tmp4 ) ) - res3[3] );
    res1[0] = ( ( ( mK * tmp3 ) - ( mL * tmp5 ) ) - res1[0] );
    res1[1] = ( ( ( mO * tmp2 ) - ( mP * tmp4 ) ) - res1[1] );
    res1[2] = ( ( ( mD * tmp5 ) - ( mC * tmp3 ) ) + res1[2] );
    res1[3] = ( ( ( mH * tmp4 ) - ( mG * tmp2 ) ) + res1[3] );
    return xMatrix4(
        ( res0 * detInv ),
        ( res1 * detInv ),
        ( res2 * detInv ),
        ( res3 * detInv )
    );
}


xMath::xMatrix3 makexMatrix3( Vector3 &v0, Vector3 &v1, Vector3 &v2 );
xMath::xMatrix3 makexMatrix3( float a, float b, float c, float d, float e, float f, float g, float h, float i );
xMath::xMatrix4 makexMatrix4( float a, float b, float c, float d, float e, float f, float g, float h, float i, float j, float k, float l, float m, float n, float o, float p );
xMath::xMatrix3 makeDiagonal( Vector3 &diag );

inline float xMatrixNorm1(xMath::xMatrix3 &m)
{
	Vector3 col0 = Vector3( m.getCol0()[0], m.getCol0()[1],m.getCol0()[2]);
	Vector3 col1 = Vector3( m.getCol1()[0], m.getCol1()[1],m.getCol1()[2]);
	Vector3 col2 = Vector3( m.getCol2()[0], m.getCol2()[1],m.getCol2()[2]);
	float v0 = std::abs(col0[0]) + std::abs(col0[1]) + std::abs(col0[2]);
	float v1 = std::abs(col1[0]) + std::abs(col1[1]) + std::abs(col1[2]);
	float v2 = std::abs(col2[0]) + std::abs(col2[1]) + std::abs(col2[2]);
	return max3(v0,v1,v2);
}


inline float xMatrixNormF2(xMath::xMatrix3 &m)
{	
	Vector3 col0 = Vector3( m.getCol0()[0], m.getCol0()[1],m.getCol0()[2]);
	Vector3 col1 = Vector3( m.getCol1()[0], m.getCol1()[1],m.getCol1()[2]);
	Vector3 col2 = Vector3( m.getCol2()[0], m.getCol2()[1],m.getCol2()[2]);
	return(col0[0]*col0[0] + col0[1]*col0[1] + col0[2]*col0[2]
		+ col1[0]*col1[0] + col1[1]*col1[1] + col1[2]*col1[2]
		+ col2[0]*col2[0] + col2[1]*col2[1] + col2[2]*col2[2]);
}


inline float xMatrixNormInf(xMath::xMatrix3 &m)
{
	Vector3 col0 = Vector3( m.getCol0()[0], m.getCol0()[1],m.getCol0()[2]);
	Vector3 col1 = Vector3( m.getCol1()[0], m.getCol1()[1],m.getCol1()[2]);
	Vector3 col2 = Vector3( m.getCol2()[0], m.getCol2()[1],m.getCol2()[2]);
	float v0 = std::abs(col0[0]) + std::abs(col1[0]) + std::abs(col2[0]);
	float v1 = std::abs(col0[1]) + std::abs(col1[1]) + std::abs(col2[1]);
	float v2 = std::abs(col0[2]) + std::abs(col1[2]) + std::abs(col2[2]);
    return max3(v0,v1,v2);
}

inline xMath::xMatrix3 cofactor(xMath::xMatrix3 &m) 
{
	xMath::xMatrix3 res = makexMatrix3(+ m.getElem(1,1)*m.getElem(2,2) - m.getElem(2,1)*m.getElem(1,2),
					- m.getElem(0,1)*m.getElem(2,2) + m.getElem(2,1)*m.getElem(0,2),
					+ m.getElem(0,1)*m.getElem(1,2) - m.getElem(1,1)*m.getElem(0,2),
					- m.getElem(1,0)*m.getElem(2,2) + m.getElem(2,0)*m.getElem(1,2),
					+ m.getElem(0,0)*m.getElem(2,2) - m.getElem(2,0)*m.getElem(0,2),
					- m.getElem(0,0)*m.getElem(1,2) + m.getElem(1,0)*m.getElem(0,2),
					+ m.getElem(1,0)*m.getElem(2,1) - m.getElem(2,0)*m.getElem(1,1),
					- m.getElem(0,0)*m.getElem(2,1) + m.getElem(2,0)*m.getElem(0,1),
					+ m.getElem(0,0)*m.getElem(1,1) - m.getElem(1,0)*m.getElem(0,1));
	return res;
}

}

#endif // _MATH_UTILS_H_
