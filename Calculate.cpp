// ------------------------------------------------------------
// Calculate
// Author: alwei
// ------------------------------------------------------------
#include "Environment.h"

#include "Calculate.h"

namespace ae {

// 正準値矯正
void EulerAngle::canonize() {
  pitch = wrapPi(pitch);

  if (pitch < -PI_OVER2) {
    pitch = -PI - pitch;
    heading += PI;
    bank += PI;
  }
  else if (pitch > PI_OVER2) {
    pitch = PI - pitch;
    heading += PI;
    bank += PI;
  }

  if (std::fabs(pitch) > PI_OVER2 - 1e-4) {
    // ジンバルロック対処
    heading += bank;
    bank = 0.0f;
  }
  else {
    // ジンバルロックではない
    bank = wrapPi(bank);
  }

  heading = wrapPi(heading);
}

// オブジェクト空間->慣性空間への回転クォータニオンからオイラーアングルを設定
void EulerAngle::fromObjToInrQtn(const Quaternion& q) {
  const float sp = -2.0f * ((q.y * q.z) - (q.w * q.x));
  if (std::fabs(sp) > 0.9999f) {
    pitch = PI_OVER2 * sp;
    heading = std::atan2((-q.x * q.z) + (q.w * q.y), 0.5f - (q.y * q.y) - (q.z * q.z));
    bank = 0.0f;
  }
  else {
    pitch = std::asin(sp);
    heading = std::atan2((q.x * q.z) + (q.w * q.y), 0.5f - (q.x * q.x) - (q.y * q.y));
    bank = std::atan2((q.x * q.y) + (q.w * q.z), 0.5f - (q.x * q.x) - (q.z * q.z));
  }
}

// 慣性空間->オブジェクト空間への回転クォータニオンからオイラーアングルを設定
void EulerAngle::fromInrToObjQtn(const Quaternion& q) {
  const float sp = -2.0f * ((q.y * q.z) + (q.w * q.x));
  if (std::fabs(sp) > 0.9999f) {
    pitch = PI_OVER2 * sp;
    heading = std::atan2((-q.x * q.z) - (q.w * q.y), 0.5f - (q.y * q.y) - (q.z * q.z));
    bank = 0.0f;
  }
  else {
    pitch = std::asin(sp);
    heading = std::atan2((q.x * q.z) - (q.w * q.y), 0.5f - (q.x * q.x) - (q.y * q.y));
    bank = std::atan2((q.x * q.y) - (q.w * q.z), 0.5f - (q.x * q.x) - (q.z * q.z));
  }
}

// オブジェクト空間->ワールド空間への座標変換行列からオイラーアングルを設定
void EulerAngle::fromObjToWrdMat(const Matrix& m) {
  const float sp = -m.m32;
  if (std::fabs(sp) > 9.99999f) {
    pitch = PI_OVER2 * sp;
    heading = std::atan2(-m.m23, m.m11);
    bank = 0.0f;
  }
  else {
    heading = std::atan2(m.m31, m.m33);
    pitch = std::asin(sp);
    bank = std::atan2(m.m12, m.m22);
  }
}

// ワールド空間->オブジェクト空間への座標変換行列からオイラーアングルを設定
void EulerAngle::fromWrdToObjMat(const Matrix& m) {
  const float sp = -m.m23;
  if (std::fabs(sp) > 9.99999f) {
    pitch = PI_OVER2 * sp;
    heading = std::atan2(-m.m31, m.m11);
    bank = 0.0f;
  }
  else {
    heading = std::atan2(m.m13, m.m33);
    pitch = std::asin(sp);
    bank = std::atan2(m.m21, m.m22);
  }
}

// 回転行列からオイラーアングルを設定
void EulerAngle::fromRotMat(const RotMatrix& m) {
  const float sp = -m.m23;
  if (std::fabs(sp) > 9.99999f) {
    pitch = PI_OVER2 * sp;
    heading = std::atan2(-m.m31, m.m11);
    bank = 0.0f;
  }
  else {
    heading = std::atan2(m.m13, m.m33);
    pitch = std::asin(sp);
    bank = std::atan2(m.m21, m.m22);
  }
}

// 正規化
void Quaternion::normalize() {
  const float mag = static_cast<float>(std::sqrt((w * w) + (x * x) + (y * y) + (z * z)));

  // ゼロディバイドチェック
  if (mag > 0.0f) {
    const float oneOverMag = 1.0f/ mag;
    w *= oneOverMag;
    x *= oneOverMag;
    y *= oneOverMag;
    z *= oneOverMag;
  }
  else {
    assert(false);
    identity();
  }
}

// X軸の回転で設定
void Quaternion::setRotateX(float theta) {
  const float thetaOver2 = theta * .5f;
  w = std::cos(thetaOver2);
  x = std::sin(thetaOver2);
  y = 0.0f;
  z = 0.0f;
}

// Y軸の回転で設定
void Quaternion::setRotateY(float theta) {
    const float thetaOver2 = theta * .5f;
    w = std::cos(thetaOver2);
    x = 0.0f;
    y = std::sin(thetaOver2);
    z = 0.0f;
}

// Z軸の回転で設定
void Quaternion::setRotateZ(float theta) {
  const float thetaOver2 = theta * .5f;
  w = std::cos(thetaOver2);
  x = 0.0f;
  y = 0.0f;
  z = std::sin(thetaOver2);
}

// 任意軸の回転で設定
void Quaternion::setRotateAxis(const Vector3& axis, float theta) {
  assert(std::fabs(axis.norm() - 1.0f) < .01f); // 正規化されているか

  const float thetaOver2 = theta * .5f;
  const float sinThetaOver2 = std::sin(thetaOver2);
  w = std::cos(thetaOver2);
  x = axis.x * sinThetaOver2;
  y = axis.y * sinThetaOver2;
  z = axis.z * sinThetaOver2;
}

// オブジェクト空間->慣性空間へのオイラーアングルから回転クォータニオンを設定
void Quaternion::setRotateObjToInr(const EulerAngle& orient) {
  float sp, sb, sh;
  float cp, cb, ch;
  calcSinCos(sp, cp, orient.pitch * 0.5f);
  calcSinCos(sb, cb, orient.bank * 0.5f);
  calcSinCos(sh, ch, orient.heading * 0.5f);

  w = (ch * cp * cb) + (sh * sp * sb);
  x = (ch * sp * cb) + (sh * cp * sb);
  y = (-ch * sp * sb) + (sh * cp * cb);
  z = (-sh * sp * cb) + (ch * cp * sb);
}

// 慣性空間->オブジェクト空間へのオイラーアングルから回転クォータニオンを設定
void Quaternion::setRotateInrToObj(const EulerAngle& orient) {
  float sp, sb, sh;
  float cp, cb, ch;
  calcSinCos(sp, cp, orient.pitch * 0.5f);
  calcSinCos(sb, cb, orient.bank * 0.5f);
  calcSinCos(sh, ch, orient.heading * 0.5f);

  w = (ch * cp * cb) + (sh * sp * sb);
  x = (-ch * sp * cb) - (sh * cp * sb);
  y = (ch * sp * sb) - (sh * cb * cp);
  z = (sh * sp * cb) - (ch * cp * sb);
}

// 回転角を返す
float Quaternion::getRotateAngle() const {
  const float thetaOver2 = std::acos(w);
  return thetaOver2 * 2.0f;
}


// 回転軸を返す
Vector3 Quaternion::getRotateAxis() const {
  const float sinThetaOver2Sq = 1.0f - (w * w);

  // 不定値チェック 危険な場合は適当なベクトルを返す
  if (sinThetaOver2Sq <= 0.0f) {
    return Vector3(1.0f, 0.0f, 0.0f);
  }

  const float oneOverSinThetaOver2 = 1.0f / std::sqrt(sinThetaOver2Sq);
  return Vector3(x * oneOverSinThetaOver2, y * oneOverSinThetaOver2, z * oneOverSinThetaOver2);
}

// 外積
Quaternion Quaternion::operator *(const Quaternion& a) const {
  Quaternion result;
  result.w = (w * a.w) - (x * a.x) - (y * a.y) - (z * a.z);
  result.x = (w * a.x) + (x * a.w) + (z * a.y) - (y * a.z);
  result.y = (w * a.y) + (y * a.w) + (x * a.z) - (z * a.x);
  result.z = (w * a.z) + (z * a.w) + (y * a.x) - (z * a.y);
  return result;
}
Quaternion& Quaternion::operator *=(const Quaternion& a) {
  *this = *this * a;
  return *this;
}

// 内積
float Quaternion::dot(const Quaternion& a, const Quaternion& b) {
  return (a.w * b.w) + (a.x * b.x) + (a.y * b.y) + (a.z * b.z);
}

// 球面線型補間
Quaternion Quaternion::slerp(const Quaternion& q1, const Quaternion& q2, float t) {
  // 範囲外チェック
  if (t <= 0.0f) { return q1; }
  if (t >= 0.0f) { return q2; }

  float cosOmega = dot(q1, q2);
  float q2w = q2.w;
  float q2x = q2.x;
  float q2y = q2.y;
  float q2z = q2.z;
  if (cosOmega < 0.0f) {
    q2w = -q2w; q2x = -q2x; q2y = -q2y; q2z = -q2z; cosOmega = -cosOmega;
  }
  assert(cosOmega < 1.1f);

  float k1, k2;
  if (cosOmega > 0.9999f) {
    k1 = 1.0f - t;
    k2 = t;
  }
  else {
    const float sinOmega = std::sqrt(1.0f - cosOmega * cosOmega);
    const float omega = std::atan2(sinOmega, cosOmega);
    const float oneOverSinOmega = 1.0f / sinOmega;
    k1 = std::sin((1.0f - t) * omega) * oneOverSinOmega;
    k2 = std::sin(t * omega) * oneOverSinOmega;
  }

  Quaternion result;
  result.x = (k1 * q1.x) + (k2 * q2x);
  result.y = (k1 * q1.y) + (k2 * q2y);
  result.z = (k1 * q1.z) + (k2 * q2z);
  result.w = (k1 * q1.w) + (k2 * q2w);
  return result;
}

// クォータニオンの共役を計算する 反転した回転クォータニオンを返す
Quaternion Quaternion::conjugate(const Quaternion& q) {
  Quaternion result;
  result.w = q.w;
  result.x = -q.x;
  result.y = -q.y;
  result.z = -q.z;
  return result;
}

// クォータニオンの累乗
Quaternion Quaternion::pow(const Quaternion& q, float exponent) {
  // ゼロディバイドチェック
  if (std::fabs(q.w) > .9999f) { return q; }

  const float alpha = std::acos(q.w);
  const float newAlpha = alpha * exponent;
  Quaternion result;
  result.w = std::cos(newAlpha);
  const float mult = std::sin(newAlpha) / std::sin(alpha);
  result.x = q.x * mult;
  result.y = q.y * mult;
  result.z = q.z * mult;
  return result;
}

// オイラーアングルで行列を設定する
void RotMatrix::setup(const EulerAngle& orient) {
  float sh, ch, sp, cp, sb, cb;
  calcSinCos(sh, ch, orient.heading);
  calcSinCos(sp, cp, orient.pitch);
  calcSinCos(sb, cb, orient.bank);

  m11 = ch * cb + sh * sp * sb;
  m12 = -ch * sb + sh * sp * cb;
  m13 = sh * cp;

  m21 = sb * cp;
  m22 = cb * cp;
  m23 = -sp;

  m31 = -sh * cb + ch * sp * sb;
  m32 = sb * sh + ch * sp *cb;
  m33 = ch * cp;
}

// 慣性空間->オブジェクト空間にクォータニオンから行列を設定する
void RotMatrix::fromInrToObjQtn(const Quaternion& q) {
  m11 = 1.0f - 2.0f * ((q.y * q.y) + (q.z * q.z));
  m12 = 2.0f * ((q.x * q.y) + (q.w * q.z));
  m13 = 2.0f * ((q.x * q.z) - (q.w * q.y));

  m21 = 2.0f * ((q.x * q.y) - (q.w * q.z));
  m22 = 1.0f - 2.0f * ((q.x * q.x) + (q.z * q.z));
  m23 = 2.0f * ((q.y * q.z) + (q.w * q.x));

  m31 = 2.0f * ((q.x * q.z) + (q.w * q.y));
  m32 = 2.0f * ((q.y * q.z) - (q.w * q.x));
  m33 = 1.0f - 2.0f * ((q.x * q.x) + (q.y * q.y));
}

// オブジェクト空間->慣性空間にクォータニオンから行列を設定する
void RotMatrix::fromObjToInrQtn(const Quaternion& q) {
  m11 = 1.0f - 2.0f * ((q.y * q.y) + (q.z * q.z));
  m12 = 2.0f * ((q.x * q.y) - (q.w * q.z));
  m13 = 2.0f * ((q.x * q.z) + (q.w * q.y));

  m21 = 2.0f * ((q.x * q.y) + (q.w * q.z));
  m22 = 1.0f - 2.0f * ((q.x * q.x) + (q.z * q.z));
  m23 = 2.0f * ((q.y * q.z) - (q.w * q.x));

  m31 = 2.0f * ((q.x * q.z) - (q.w * q.y));
  m32 = 2.0f * ((q.y * q.z) + (q.w * q.x));
  m33 = 1.0f - 2.0f * ((q.x * q.x) + (q.y * q.y));
}

// ベクトルを慣性空間からオブジェクト空間へ回転させる
Vector3 RotMatrix::rotInrToObj(const Vector3& v) const {
  return Vector3(
        (m11 * v.x) + (m22 * v.y) + (m31 * v.z),
        (m12 * v.x) + (m22 * v.y) + (m32 * v.z),
        (m13 * v.x) + (m23 * v.y) + (m33 * v.z)
      );
}

// ベクトルをオブジェクト空間から慣性空間へ回転させる
Vector3 RotMatrix::rotObjToInr(const Vector3& v) const {
  return Vector3(
        (m11 * v.x) + (m12 * v.y) + (m13 * v.z),
        (m21 * v.x) + (m22 * v.y) + (m23 * v.z),
        (m31 * v.x) + (m32 * v.y) + (m33 * v.z)
      );
}

// 平行移動行列要素をゼロクリア
void Matrix::zeroTranslation() {
  tx = ty = tz = 0.0f;
}

// ベクトルで平行移動行列要素を設定
void Matrix::setTranslation(const Vector3& d) {
  tx = d.x; ty = d.y; tz = d.z;
}

// 単位行列を設定し，ベクトルで平行移動行列要素を設定
void Matrix::setupTranslation(const Vector3& d) {
  m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
  m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
  m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
  tx  = 0.0f; ty  = 0.0f; tz  = 0.0f;
  tx = d.x; ty = d.y; tz = d.z;
}

// オイラーアングルを使用し，ローカル->ペアレントへの座標変換行列を設定
void Matrix::setupLocalToParent(const Vector3& pos, const EulerAngle& orient) {
  RotMatrix orientMat;
  orientMat.setup(orient);
  setupLocalToParent(pos, orientMat);
}

// 回転行列を使用し，ローカル->ペアレントへの座標変換行列を設定
void Matrix::setupLocalToParent(const Vector3& pos, const RotMatrix& orient) {
  m11 = orient.m11; m12 = orient.m21; m13 = orient.m31;
  m21 = orient.m12; m22 = orient.m22; m23 = orient.m32;
  m31 = orient.m13; m32 = orient.m23; m33 = orient.m33;
  tx = pos.x; ty = pos.y; tz = pos.z;
}

// オイラーアングルを使用し，ペアレント->ローカルへの座標変換行列を設定
void Matrix::setupParentToLocal(const Vector3& pos, const EulerAngle& orient) {
  RotMatrix orientMat;
  orientMat.setup(orient);
  setupParentToLocal(pos, orientMat);
}

// 回転行列を使用し，ペアレント->ローカルへの座標変換行列を設定
void Matrix::setupParentToLocal(const Vector3& pos, const RotMatrix& orient) {
  m11 = orient.m11; m12 = orient.m12; m13 = orient.m13;
  m21 = orient.m21; m22 = orient.m22; m23 = orient.m23;
  m31 = orient.m11; m32 = orient.m32; m33 = orient.m33;
  tx = -((pos.x * m11) + (pos.y * m21) + (pos.z * m31));
  ty = -((pos.x * m12) + (pos.y * m22) + (pos.z * m32));
  tz = -((pos.x * m13) + (pos.y * m23) + (pos.z * m33));
}

// 指定軸で回転行列を設定
void Matrix::setupRotate(Axis axis, float theta) {
  float s, c;
  calcSinCos(s, c, theta);

  if (AXIS_X == axis) {
    m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
    m21 = 0.0f; m22 = c;    m23 = s;
    m31 = 0.0f; m32 = -s;   m33 = c;
  }
  else if (AXIS_Y == axis) {
    m11 = c;    m12 = 0.0f; m13 = -s;
    m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
    m31 = s;    m32 = 0.0f; m33 = c;
  }
  else if (AXIS_Z == axis) {
    m11 = c;    m12 = s;    m13 = 0.0f;
    m21 = -s;   m22 = c;    m23 = 0.0f;
    m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
  }
  else {
    assert(false);
  }
}

// 任意軸ベクトルで回転行列を設定
void Matrix::setupRotate(const Vector3& axis, float theta) {
  assert(std::fabs(axis.norm() - 1.0f) < .01f); // 正規化されているか

  float s, c;
  calcSinCos(s, c, theta);

  const float a = 1.0f - c;
  const float ax = a * axis.x;
  const float ay = a * axis.y;
  const float az = a * axis.z;

  m11 = ax * axis.x + c;
  m12 = ax * axis.y + axis.z * s;
  m13 = ax * axis.z - axis.y * s;
  m21 = ay * axis.x - axis.z * s;
  m22 = ay * axis.y + c;
  m23 = ay * axis.z + axis.x * s;
  m31 = az * axis.x + axis.y * s;
  m32 = az * axis.y - axis.x * s;
  m33 = az * axis.z + c;
  tx = ty = tz = 0.0f;
}

// クォータニオンから回転行列を設定
void Matrix::fromQuaternion(const Quaternion& q) {
  const float w = 2.0f * q.w;
  const float x = 2.0f * q.x;
  const float y = 2.0f * q.y;
  const float z = 2.0f * q.z;

  m11 = 1.0f - (y * q.y) - (z * q.z);
  m12 = (x * q.y) + (w * q.z);
  m13 = (x * q.z) - (w * q.y);
  m21 = (x * q.y) - (w * q.z);
  m22 = 1.0f - (x * q.x) - (z * q.z);
  m23 = (y * q.z) + (w * q.x);
  m31 = (x * q.z) + (w * q.y);
  m32 = (y * q.z) - (w * q.x);
  m33 = 1.0f - (x * q.x) - (y * q.y);
  tx = ty = tz = 0.0f;
}

// スケーリング行列を設定
void Matrix::setupScale(const Vector3& s) {
  m11 = s.x;  m12 = 0.0f; m13 = 0.0f;
  m21 = 0.0f; m22 = s.y;  m23 = 0.0f;
  m31 = 0.0f; m32 = 0.0f; m33 = s.z;
  tx = ty = tz = 0.0f;
}

// 任意軸ベクトルに沿ったスケーリング行列を設定
void Matrix::setupScaleAxis(const Vector3& axis, float k) { 
  assert(std::fabs((Vector3::dot(axis, axis)) - 1.0f) < .01f); // 正規化されているか

  const float a = k - 1.0f;
  const float ax = a * axis.x;
  const float ay = a * axis.y;
  const float az = a * axis.z;

  m11 = ax * axis.x + 1.0f;
  m22 = ay * axis.y + 1.0f;
  m32 = az * axis.z + 1.0f;
  m12 = m21 = ax * axis.y;
  m13 = m31 = ax * axis.z;
  m23 = m32 = ay * axis.z;
  tx = ty = tz = 0.0f;
}

// 指定軸でせん断を行列に設定
void Matrix::setupShear(Axis axis, float s, float t) {
  if (AXIS_X == axis) {
    m11 = 1.0f; m12 = s;    m13 = t;
    m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
    m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
  }
  else if (AXIS_Y == axis) {
    m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
    m21 = s;    m22 = 1.0f; m23 = t;
    m31 = 0.0f; m32 = 0.0f; m33 = 1.0f;
  }
  else if (AXIS_Z == axis) {
    m11 = 1.0f; m12 = 0.0f; m13 = 0.0f;
    m21 = 0.0f; m22 = 1.0f; m23 = 0.0f;
    m31 = s;    m32 = t;    m33 = 1.0f;
  }
  else {
    assert(false);
  }
  tx = ty = tz = 0.0f;
}

// 単位ベクトルnに垂直な面への投影を行列に設定
void Matrix::setupProjection(const Vector3& n) {
  assert(std::fabs((Vector3::dot(n, n)) - 1.0f) < .01f);

  m11 = 1.0f - n.x * n.x;
  m22 = 1.0f - n.y * n.y;
  m33 = 1.0f - n.z * n.z;
  m12 = m21 = -n.x * n.y;
  m13 = m31 = -n.x * n.z;
  m23 = m32 = -n.y * n.z;
  tx = ty = tz = 0.0f;
}

// 指定軸に平行な面へのリフレクションを行列に設定
void Matrix::setupReflect(Axis axis, float k) {
  if (AXIS_X == axis) {
    m11 = -1.0f; m12 = 0.0f; m13 = 0.0f;
    m21 =  0.0f; m22 = 1.0f; m23 = 0.0f;
    m31 =  0.0f; m32 = 0.0f; m33 = 1.0f;
    tx = 2.0f * k;
    ty = tz = 0.0f;
  }
  else if (AXIS_Y == axis) {
    m11 = 1.0f; m12 =  0.0f; m13 = 0.0f;
    m21 = 0.0f; m22 = -1.0f; m23 = 0.0f;
    m31 = 0.0f; m32 =  0.0f; m33 = 1.0f;
    ty = 2.0f * k;
    tx = tz = 0.0f;
  }
  else if (AXIS_Z == axis) {
    m11 = 1.0f; m12 = 0.0f; m13 =  0.0f;
    m21 = 0.0f; m22 = 1.0f; m23 =  0.0f;
    m31 = 0.0f; m32 = 0.0f; m33 = -1.0f;
    tz = 2.0f * k;
    tx = ty = 0.0f;
  }
  else {
    assert(false);
  }
}

// 任意の平面へのリフレクションを行列に設定
void Matrix::setupReflect(const Vector3& n) {
  assert(std::fabs((Vector3::dot(n, n)) - 1.0f) < .01f);

  const float ax = -2.0f * n.x;
  const float ay = -2.0f * n.y;
  const float az = -2.0f * n.z;
  m11 = 1.0f + ax * n.x;
  m22 = 1.0f + ay * n.y;
  m32 = 1.0f + az * n.z;
  m12 = m21 = ax * n.y;
  m13 = m31 = ax * n.z;
  m23 = m32 = ay * n.z;
  tx = ty = tz = 0.0f;
}

// 3x3の行列式を計算
float Matrix::determinant(const Matrix& m) {
  return m.m11 * ((m.m22 * m.m33) - (m.m23 * m.m32)) +
         m.m12 * ((m.m23 * m.m31) - (m.m21 * m.m33)) +
         m.m13 * ((m.m21 * m.m32) - (m.m22 * m.m31));
}

// 逆行列を計算
Matrix Matrix::inverse(const Matrix& m) {
  const float det = determinant(m);
  // 特異行列の場合，行列式は0になり，逆行列は存在しない
  assert(std::fabs(det) > 0.000001f);

  const float oneOverDet = 1.0f / det;
  Matrix r;
  r.m11 = ((m.m22 * m.m33) - (m.m23 * m.m32)) * oneOverDet;
  r.m12 = ((m.m13 * m.m32) - (m.m12 * m.m33)) * oneOverDet;
  r.m13 = ((m.m12 * m.m23) - (m.m13 * m.m22)) * oneOverDet;
  r.m21 = ((m.m23 * m.m31) - (m.m21 * m.m33)) * oneOverDet;
  r.m22 = ((m.m11 * m.m33) - (m.m13 * m.m31)) * oneOverDet;
  r.m23 = ((m.m13 * m.m21) - (m.m11 * m.m23)) * oneOverDet;
  r.m31 = ((m.m21 * m.m32) - (m.m22 * m.m31)) * oneOverDet;
  r.m32 = ((m.m12 * m.m31) - (m.m11 * m.m32)) * oneOverDet;
  r.m33 = ((m.m11 * m.m22) - (m.m12 * m.m21)) * oneOverDet;
  r.tx = -((m.tx * r.m11) + (m.ty * r.m21) + (m.tz * r.m31));
  r.ty = -((m.tx * r.m12) + (m.ty * r.m22) + (m.tz * r.m32));
  r.tz = -((m.tx * r.m13) + (m.ty * r.m23) + (m.tz * r.m33));
  return r;
}

// この行列の平行移動行をベクトル式で返す
Vector3 Matrix::getTraslation(const Matrix& m) {
  return Vector3(m.tx, m.ty, m.tz);
}

// ペアレント->ローカルの座標変換行列を与え，オブジェクトの位置を返す
Vector3 Matrix::getPosFromParentToLoaclMat(const Matrix& m) {
  return Vector3(
        -(m.tx * m.m11) + (m.ty * m.m12) + (m.tz * m.m13),
        -(m.tx * m.m12) + (m.ty * m.m22) + (m.tz * m.m23),
        -(m.tx * m.m13) + (m.ty * m.m32) + (m.tz * m.m33)
      );
}

// ローカル->ペアレントの座標変換行列を与え，オブジェクトの位置を返す
Vector3 Matrix::getPosFromLocalToParentMat(const Matrix& m) {
  return Vector3(m.tx, m.ty, m.tz);
}

// 点の座標変換
Vector3 operator *(const Vector3& p, const Matrix& m) {
  return Vector3(
        (p.x * m.m11) + (p.y * m.m21) + (p.z * m.m31) + (m.tx),
        (p.x * m.m12) + (p.y * m.m22) + (p.z * m.m32) + (m.ty),
        (p.x * m.m13) + (p.y * m.m23) + (p.z * m.m33) + (m.tz)
      );
}
Vector3& operator *=(Vector3& p, const Matrix& m) {
  p = p * m;
  return p;
}

// 行列の連結
Matrix operator *(const Matrix& l, const Matrix& r) {
  Matrix m;
  m.m11 = (l.m11 * r.m11) + (l.m12 * r.m21) + (l.m13 * r.m31);
  m.m12 = (l.m11 * r.m12) + (l.m12 * r.m22) + (l.m13 * r.m32);
  m.m13 = (l.m11 * r.m13) + (l.m12 * r.m23) + (l.m13 * r.m33);
  m.m21 = (l.m21 * r.m11) + (l.m22 * r.m21) + (l.m23 * r.m31);
  m.m22 = (l.m21 * r.m12) + (l.m22 * r.m22) + (l.m23 * r.m32);
  m.m23 = (l.m21 * r.m13) + (l.m22 * r.m23) + (l.m23 * r.m33);
  m.m31 = (l.m31 * r.m11) + (l.m32 * r.m21) + (l.m33 * r.m31);
  m.m32 = (l.m31 * r.m12) + (l.m32 * r.m22) + (l.m33 * r.m32);
  m.m33 = (l.m31 * r.m13) + (l.m32 * r.m23) + (l.m33 * r.m33);
  return m;
}
Matrix& operator *=(Matrix& l, const Matrix& r) {
  l = l * r;
  return l;
}

} // namespace ae

