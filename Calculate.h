// ------------------------------------------------------------
// Calculate
// Author: alwei
// ------------------------------------------------------------
#ifndef Calculate_h_
#define Calculate_h_

#ifndef Environment_h_
  #include <cassert>
  #include <cmath>
  #include <iostream>
#endif

namespace ae {

const double PI = 3.14159265f;
const double PI2 = PI * 2.0f;
const double PI_OVER2 = PI / 2.0f;
const double ONEOVER_PI = 1.0f / PI;
const double ONEOVER2_PI = 1.0f / PI2;

// ラジアンからデグリーへ変換
inline double rad2deg(float rad) {
  return rad * (180.0 / PI);
}

// デグリーからラジアンへ変換
inline double deg2rad(float deg) {
  return deg * (PI / 180.0);
}

// 適切に2πの倍数を加える事で角度を-π…πの範囲にラップする
inline double wrapPi(float theta) {
  theta += PI;
  theta -= std::floor(theta * ONEOVER2_PI) * PI2;
  theta -= PI;
  return theta;
}

// ある角度のsinとcos両方を計算する
inline void calcSinCos(float& sin, float& cos, float theta) {
  sin = std::sin(theta);
  cos = std::cos(theta);
}

// 2次元ベクトル
class Vector2 {
public:
  union {
    struct { float x, y; };
    float v[2];
  };

  Vector2() : x(0), y(0) {}
  Vector2(float x_, float y_) : x(x_), y(y_) {}
  Vector2(const Vector2& v) : x(v.x), y(v.y) {}
  explicit Vector2(float t) : x(t), y(t) {}

  float& operator [](size_t i) { return v[i]; }
  const float& operator [](size_t i) const { return v[i]; }
  void operator =(const Vector2& v) { x = v.x; y = v.y; }
  void operator =(float t) { x = t; y = t; }

  bool operator ==(const Vector2& v) const { return x == v.x && y == v.y; }
  bool operator !=(const Vector2& v) const { return x != v.x || y != v.y; }
  bool operator >(const Vector2& v) const { return norm() > v.norm() ? true : false; }
  bool operator <(const Vector2& v) const { return norm() < v.norm() ? true : false; }
  bool operator >=(const Vector2& v) const { return norm() >= v.norm() ? true : false; }
  bool operator <=(const Vector2& v) const { return norm() <= v.norm() ? true : false; }

  Vector2 operator +(float t) const { return Vector2(x + t, y + t); }
  Vector2 operator -(float t) const { return Vector2(x - t, y - t); }
  Vector2 operator *(float t) const { return Vector2(x * t, y * t); }
  Vector2 operator /(float t) const { return Vector2(x / t, y / t); }
  void operator +=(float t) { x += t; y += t; }
  void operator -=(float t) { x -= t; y -= t; }
  void operator *=(float t) { x *= t; y *= t; }
  void operator /=(float t) { x /= t; y /= t; }

  Vector2 operator +(const Vector2& v) const { return Vector2(x + v.x, y + v.y); }
  Vector2 operator -(const Vector2& v) const { return Vector2(x - v.x, y - v.y); }
  Vector2 operator *(const Vector2& v) const { return Vector2(x * v.x, y * v.y); }
  Vector2 operator /(const Vector2& v) const { return Vector2(x / v.x, y / v.y); }
  void operator +=(const Vector2& v) { x += v.x; y += v.y; }
  void operator -=(const Vector2& v) { x -= v.x; y -= v.y; }
  void operator *=(const Vector2& v) { x *= v.x; y *= v.y; }
  void operator /=(const Vector2& v) { x /= v.x; y /= v.y; }

  float square() const { return (x * x) + (y * y); }
  float norm() const { return std::sqrt((x * x) + (y * y)); }

  Vector2& normalize() {
    float n = norm();
    if (n != 0) { *this /= n; }
    return *this;
  }

  float dot(const Vector2& v) { return (x * v.x) + (y * v.y); }
  static float dot(const Vector2& v1, const Vector2& v2) 
    { return (v1.x * v2.x) + (v1.y * v2.y); }

  float distance(const Vector2& v) {
    Vector2 t((x - v.x), (y - v.y));
    return std::sqrt((t.x * t.x) + (t.y * t.y));
  }
  static float distance(const Vector2& v1, const Vector2& v2) {
    Vector2 t((v1.x - v2.x), (v1.y - v2.y));
    return std::sqrt((t.x * t.x) + (t.y * t.y));
  }

  void out() {
    std::cout << '(' << x << ", " << y << ')' << std::endl;
  }
};

// 3次元ベクトル
class Vector3 {
public:
  union {
    struct { float x, y, z; };
    float v[3];
  };

  Vector3() : x(0), y(0), z(0) {}
  Vector3(float x_, float y_, float z_) : x(x_), y(y_), z(z_) {}
  Vector3(const Vector3& v) : x(v.x), y(v.y), z(v.z) {}
  explicit Vector3(float t) : x(t), y(t), z(t) {}

  float& operator [](size_t i) { return v[i]; }
  const float& operator [](size_t i) const { return v[i]; }
  void operator =(const Vector3& v) { x = v.x; y = v.y; z = v.z; }
  void operator =(float t) { x = t; y = t; z = t; }

  bool operator ==(const Vector3& v) const { return x == v.x && y == v.y && z == v.z; }
  bool operator !=(const Vector3& v) const { return x != v.x || y != v.y || z != v.z; }
  bool operator >(const Vector3& v) const { return norm() > v.norm() ? true : false; }
  bool operator <(const Vector3& v) const { return norm() < v.norm() ? true : false; }
  bool operator >=(const Vector3& v) const { return norm() >= v.norm() ? true : false; }
  bool operator <=(const Vector3& v) const { return norm() <= v.norm() ? true : false; }

  Vector3 operator +(float t) const { return Vector3(x + t, y + t, z + t); }
  Vector3 operator -(float t) const { return Vector3(x - t, y - t, z - t); }
  Vector3 operator *(float t) const { return Vector3(x * t, y * t, z * t); }
  Vector3 operator /(float t) const { return Vector3(x / t, y / t, z / t); }
  void operator +=(float t) { x += t; y += t; z += t; }
  void operator -=(float t) { x -= t; y -= t; z -= t; }
  void operator *=(float t) { x *= t; y *= t; z *= t; }
  void operator /=(float t) { x /= t; y /= t; z /= t; }

  Vector3 operator +(const Vector3& v) const { return Vector3(x + v.x, y + v.y, z + v.z); }
  Vector3 operator -(const Vector3& v) const { return Vector3(x - v.x, y - v.y, z - v.z); }
  Vector3 operator *(const Vector3& v) const { return Vector3(x * v.x, y * v.y, z * v.z); }
  Vector3 operator /(const Vector3& v) const { return Vector3(x / v.x, y / v.y, z / v.z); }
  void operator +=(const Vector3& v) { x += v.x; y += v.y; z += v.z; }
  void operator -=(const Vector3& v) { x -= v.x; y -= v.y; z -= v.z; }
  void operator *=(const Vector3& v) { x *= v.x; y *= v.y; z *= v.z; }
  void operator /=(const Vector3& v) { x /= v.x; y /= v.y; z /= v.z; }

  float square() const { return (x * x) + (y * y) + (z * z); }
  float norm() const { return std::sqrt((x * x) + (y * y) + (z * z)); }

  Vector3& normalize() {
    float n = norm();
    if (n != 0) { *this /= n; }
    return *this;
  }

  float dot(const Vector3& v) { return (x * v.x) + (y * v.y) + (z * v.z); }
  static float dot(const Vector3& v1, const Vector3& v2) 
    { return (v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z); }

  Vector3 cross(const Vector3& v) {
    return Vector3(
        (y * v.z) - (z * v.y),
        (z * v.x) - (x * v.z),
        (x * v.y) - (y * v.x));
  }
  static Vector3 cross(const Vector3& v1, const Vector3& v2) {
    return Vector3(
        (v1.y * v2.z) - (v1.z * v2.y),
        (v1.z * v2.x) - (v1.x * v2.z),
        (v1.x * v2.y) - (v1.y * v2.x));
  }

  float distance(const Vector3& v) {
    Vector3 t((x - v.x), (y - v.y), (z - v.z));
    return std::sqrt((t.x * t.x) + (t.y * t.y) + (t.z * t.z));
  }
  static float distance(const Vector3& v1, const Vector3& v2) {
    Vector3 t((v1.x - v2.x), (v1.y - v2.y), (v1.z - v2.z));
    return std::sqrt((t.x * t.x) + (t.y * t.y) + (t.z * t.z));
  }

  void out() {
    std::cout << '(' << x << ", " << y << ", " << z << ')' << std::endl;
  }
};

// 4次元ベクトル
class Vector4 {
public:
  union {
    struct { float x, y, z, w; };
    float v[4];
  };

  Vector4() : x(0), y(0), z(0), w(0) {}
  Vector4(float x_, float y_, float z_, float w_) : x(x_), y(y_), z(z_),w(w_) {}
  Vector4(const Vector4& v) : x(v.x), y(v.y), z(v.z), w(v.w) {}
  explicit Vector4(float t) : x(t), y(t), z(t), w(t) {}

  float& operator [](size_t i) { return v[i]; }
  const float& operator [](size_t i) const { return v[i]; }
  void operator =(const Vector4& v) { x = v.x; y = v.y; z = v.z; w = v.w; }
  void operator =(float t) { x = t; y = t; z = t; }

  bool operator ==(const Vector4& v) const { return x == v.x && y == v.y && z == v.z && w == v.w; }
  bool operator !=(const Vector4& v) const { return x != v.x || y != v.y || z != v.z || w != v.w; }
  bool operator >(const Vector4& v) const { return norm() > v.norm() ? true : false; }
  bool operator <(const Vector4& v) const { return norm() < v.norm() ? true : false; }
  bool operator >=(const Vector4& v) const { return norm() >= v.norm() ? true : false; }
  bool operator <=(const Vector4& v) const { return norm() <= v.norm() ? true : false; }

  Vector4 operator +(float t) const { return Vector4(x + t, y + t, z + t, w + t); }
  Vector4 operator -(float t) const { return Vector4(x - t, y - t, z - t, w - t); }
  Vector4 operator *(float t) const { return Vector4(x * t, y * t, z * t, w * t); }
  Vector4 operator /(float t) const { return Vector4(x / t, y / t, z / t, w / t); }
  void operator +=(float t) { x += t; y += t; z += t; w += t; }
  void operator -=(float t) { x -= t; y -= t; z -= t; w -= t; }
  void operator *=(float t) { x *= t; y *= t; z *= t; w *= t; }
  void operator /=(float t) { x /= t; y /= t; z /= t; w /= t; }

  Vector4 operator +(const Vector4& v) const { return Vector4(x + v.x, y + v.y, z + v.z, w + v.w); }
  Vector4 operator -(const Vector4& v) const { return Vector4(x - v.x, y - v.y, z - v.z, w - v.w); }
  Vector4 operator *(const Vector4& v) const { return Vector4(x * v.x, y * v.y, z * v.z, w * v.w); }
  Vector4 operator /(const Vector4& v) const { return Vector4(x / v.x, y / v.y, z / v.z, w / v.w); }
  void operator +=(const Vector4& v) { x += v.x; y += v.y; z += v.z; w += v.w; }
  void operator -=(const Vector4& v) { x -= v.x; y -= v.y; z -= v.z; w -= v.w; }
  void operator *=(const Vector4& v) { x *= v.x; y *= v.y; z *= v.z; w *= v.w; }
  void operator /=(const Vector4& v) { x /= v.x; y /= v.y; z /= v.z; w /= v.w; }

  float square() const { return (x * x) + (y * y) + (z * z) + (w * w); }
  float norm() const { return std::sqrt((x * x) + (y * y) + (z * z)) + (w * w); }

  Vector4& normalize() {
    float n = norm();
    if (n != 0) { *this /= n; }
    return *this;
  }

  float dot(const Vector4& v) { return (x * v.x) + (y * v.y) + (z * v.z) * (w * v.w); }
  static float dot(const Vector4& v1, const Vector4& v2) 
    { return (v1.x * v2.x) + (v1.y * v2.y) + (v1.z * v2.z) * (v1.w * v2.w); }

  float distance(const Vector4& v) {
    Vector4 t((x - v.x), (y - v.y), (z - v.z), (w - v.w));
    return std::sqrt((t.x * t.x) + (t.y * t.y) + (t.z * t.z) + (t.w * t.w));
  }
  static float distance(const Vector4& v1, const Vector4& v2) {
    Vector4 t((v1.x - v2.x), (v1.y - v2.y), (v1.z - v2.z), (v1.w - v2.w));
    return std::sqrt((t.x * t.x) + (t.y * t.y) + (t.z * t.z) + (t.w * t.w));
  }

  void out() {
    std::cout << '(' << x << ", " << y << ", " << z << ", " << w << ')' << std::endl;
  }
};

class Quaternion;
class RotMatrix;
class Matrix;

// オイラーアングル
class EulerAngle {
public:
  float heading, pitch, bank;

  EulerAngle() {}
  EulerAngle(float h, float p, float b) : heading(h), pitch(p), bank(b) {}

  void identity() { heading = pitch = bank = 0.0f; }
  void canonize();

  void fromObjToInrQtn(const Quaternion& q);
  void fromInrToObjQtn(const Quaternion& q);
  void fromObjToWrdMat(const Matrix& m);
  void fromWrdToObjMat(const Matrix& m);
  void fromRotMat(const RotMatrix& m);
};

// クォータニオン
class Quaternion {
public:
  float w, x, y, z;

  void identity() { w = 1.0f; x = y = z = 0.0f; }
  void normalize();

  void setRotateX(float theta);
  void setRotateY(float theta);
  void setRotateZ(float theta);
  void setRotateAxis(const Vector3& axis, float theta);

  void setRotateObjToInr(const EulerAngle& orient);
  void setRotateInrToObj(const EulerAngle& orient);

  float getRotateAngle() const;
  Vector3 getRotateAxis() const;

  Quaternion operator *(const Quaternion& a) const;
  Quaternion& operator *=(const Quaternion& a);
  static float dot(const Quaternion& a, const Quaternion& b);

  static Quaternion slerp(const Quaternion& q1, const Quaternion& q2, float t);
  static Quaternion conjugate(const Quaternion& q);
  static Quaternion pow(const Quaternion& q, float exponent);
};

// 回転作業行列
class RotMatrix {
public:
  float m11, m12, m13;
  float m21, m22, m23;
  float m31, m32, m33;

  void identity() {
    m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
    m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
    m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
  }

  void setup(const EulerAngle& orient);
  void fromInrToObjQtn(const Quaternion& q);
  void fromObjToInrQtn(const Quaternion& q);
  Vector3 rotInrToObj(const Vector3& v) const;
  Vector3 rotObjToInr(const Vector3& v) const;
};

// 行列
class Matrix {
public:
  enum Axis {
    AXIS_X,
    AXIS_Y,
    AXIS_Z,
  };

  // この行列は4行3列
  float m11, m12, m13;
  float m21, m22, m23;
  float m31, m32, m33;
  float tx, ty, tz;

  void identity() {
    m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
    m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
    m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
    tx  = 0.0f; ty  = 0.0f; tz  = 0.0f;
  }

  void zeroTranslation();
  void setTranslation(const Vector3& d);
  void setupTranslation(const Vector3& d);

  void setupLocalToParent(const Vector3& pos, const EulerAngle& orient);
  void setupLocalToParent(const Vector3& pos, const RotMatrix& orient);
  void setupParentToLocal(const Vector3& pos, const EulerAngle& orient);
  void setupParentToLocal(const Vector3& pos, const RotMatrix& orient);

  void setupRotate(Axis axis, float theta);
  void setupRotate(const Vector3& axis, float theta);

  void fromQuaternion(const Quaternion& q);

  void setupScale(const Vector3& s);
  void setupScaleAxis(const Vector3& axis, float k);
  void setupShear(Axis axis, float s, float t);
  void setupProjection(const Vector3& n);
  void setupReflect(Axis axis, float k = 0.0f);
  void setupReflect(const Vector3& n);

  static float determinant(const Matrix& m);
  static Matrix inverse(const Matrix& m);
  static Vector3 getTraslation(const Matrix& m);
  static Vector3 getPosFromParentToLoaclMat(const Matrix& m);
  static Vector3 getPosFromLocalToParentMat(const Matrix& m);
};

} // namespace ae

#endif // Calculate_h_

