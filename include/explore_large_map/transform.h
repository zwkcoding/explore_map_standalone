

#ifndef CARTOGRAPHER_TRANSFORM_TRANSFORM_H_
#define CARTOGRAPHER_TRANSFORM_TRANSFORM_H_

#include <cmath>

#include "Eigen/Core"
#include "Eigen/Geometry"
#include "math.h"
#include "rigid_transform.h"

namespace cartographer {
namespace transform {
  
  
  
  enum GridZone
  {
    UTM_ZONE_AUTO = 0,
    UTM_ZONE_1=1,  UTM_ZONE_2=2,    UTM_ZONE_3=3,    UTM_ZONE_4=4,    UTM_ZONE_5=5,
    UTM_ZONE_6=6,    UTM_ZONE_7=7,    UTM_ZONE_8=8,    UTM_ZONE_9=9,    UTM_ZONE_10=10,
    UTM_ZONE_11=11,   UTM_ZONE_12=12,   UTM_ZONE_13=13,   UTM_ZONE_14=14,   UTM_ZONE_15=15,
    UTM_ZONE_16=16,   UTM_ZONE_17=17,   UTM_ZONE_18=18,   UTM_ZONE_19=19,   UTM_ZONE_20=20,
    UTM_ZONE_21=21,   UTM_ZONE_22=22,   UTM_ZONE_23=23,   UTM_ZONE_24=24,   UTM_ZONE_25=25,
    UTM_ZONE_26=26,   UTM_ZONE_27=27,   UTM_ZONE_28=28,   UTM_ZONE_29=29,   UTM_ZONE_30=30,
    UTM_ZONE_31=31,   UTM_ZONE_32=32,   UTM_ZONE_33=33,   UTM_ZONE_34=34,   UTM_ZONE_35=35,
    UTM_ZONE_36=36,   UTM_ZONE_37=37,   UTM_ZONE_38=38,   UTM_ZONE_39=39,   UTM_ZONE_40=40,
    UTM_ZONE_41=41,   UTM_ZONE_42=42,   UTM_ZONE_43=43,   UTM_ZONE_44=44,   UTM_ZONE_45=45,
    UTM_ZONE_46=46,   UTM_ZONE_47=47,   UTM_ZONE_48=48,   UTM_ZONE_49=49,   UTM_ZONE_50=50,
    UTM_ZONE_51=51,   UTM_ZONE_52=52,   UTM_ZONE_53=53,   UTM_ZONE_54=54,   UTM_ZONE_55=55,
    UTM_ZONE_56=56,   UTM_ZONE_57=57,   UTM_ZONE_58=58,   UTM_ZONE_59=59,   UTM_ZONE_60=60,
    UPS_NORTH, UPS_SOUTH, 
    GRID_AUTO
  };

enum Hemisphere
  {
    HEMI_AUTO = 0,  HEMI_NORTH, HEMI_SOUTH
  };
  struct point{
    double x;
    double y;
  };
struct state_struct{
 point position;
};

#if !defined(__cplusplus)
typedef enum GridZone GridZone;
typedef enum Hemisphere Hemisphere;
#endif

/* FORWARD AND BACK TM/PS PROJECTIONS FOR A SPHERE */

void geographic_to_tm_sphere(double R, double k0, 
			     double lon_mer, double FN, double FE,
			     double lat_rad, double lon_rad,
			     double* N, double* E);

void tm_to_geographic_sphere(double R, double k0, 
			     double lon_mer, double FN, double FE,
			     double N, double E,
			     double* lat_rad, double* lon_rad);

void geographic_to_ps_sphere(double R, double k0, 
			     Hemisphere hemi, double FN, double FE,
			     double lat_rad, double lon_rad,
			     double* N, double* E);

void ps_to_geographic_sphere(double R, double k0, 
			     Hemisphere hemi, double FN, double FE,
			     double N, double E,
			     double* lat_rad, double* lon_rad);

/* FORWARD AND BACK TM/PS PROJECTIONS FOR AN ELLIPSOID */

#ifndef TM_TO_GEOGRAPHIC_TOLERANCE_M
#define TM_TO_GEOGRAPHIC_TOLERANCE_M 0.001
#endif

void geographic_to_tm(double a, double e2, double k0, 
		      double lon_mer, double FN, double FE,
		      double lat_rad, double lon_rad,
		      double* N, double* E);

void tm_to_geographic(double a, double e2, double k0, 
		      double lon_mer, double FN, double FE,
		      double N, double E,
		      double* lat_rad, double* lon_rad);

void geographic_to_ps(double a, double e2, double k0, 
		      Hemisphere hemi, double FN, double FE,
		      double lat_rad, double lon_rad,
		      double* N, double* E);

void ps_to_geographic(double a, double e2, double k0, 
		      Hemisphere hemi, double FN, double FE,
		      double N, double E,
		      double* lat_rad, double* lon_rad);
  
  /* FORWARD AND BACK PROJECTIONS FOR AN ELLIPSOID ONTO THE UTM/UPS GRID */

int geographic_to_grid(double a, double e2,
		       double lat_rad, double lon_rad, 
		       GridZone* zone, Hemisphere* hemi, double* N, double* E);

int grid_to_geographic(double a, double e2,		       
		       GridZone zone, Hemisphere hemi, double N, double E,
		       double* lat_rad, double* lon_rad);

// Returns the non-negative rotation angle in radians of the 3D transformation
// 'transform'.
template <typename FloatType>
FloatType GetAngle(const Rigid3<FloatType>& transform) {
  return FloatType(2) * std::atan2(transform.rotation().vec().norm(),
                                   std::abs(transform.rotation().w()));
}

// Returns the yaw component in radians of the given 3D 'rotation'. Assuming
// 'rotation' is composed of three rotations around X, then Y, then Z, returns
// the angle of the Z rotation.
template <typename T>
T GetYaw(const Eigen::Quaternion<T>& rotation) {
  const Eigen::Matrix<T, 3, 1> direction =
      rotation * Eigen::Matrix<T, 3, 1>::UnitX();
  return atan2(direction.y(), direction.x());
}

// Returns the yaw component in radians of the given 3D transformation
// 'transform'.
template <typename T>
T GetYaw(const Rigid3<T>& transform) {
  return GetYaw(transform.rotation());
}

// Returns an angle-axis vector (a vector with the length of the rotation angle
// pointing to the direction of the rotation axis) representing the same
// rotation as the given 'quaternion'.
template <typename T>
Eigen::Matrix<T, 3, 1> RotationQuaternionToAngleAxisVector(
    const Eigen::Quaternion<T>& quaternion) {
  Eigen::Quaternion<T> normalized_quaternion = quaternion.normalized();
  // We choose the quaternion with positive 'w', i.e., the one with a smaller
  // angle that represents this orientation.
  if (normalized_quaternion.w() < 0.) {
    // Multiply by -1. http://eigen.tuxfamily.org/bz/show_bug.cgi?id=560
    normalized_quaternion.w() *= T(-1.);
    normalized_quaternion.x() *= T(-1.);
    normalized_quaternion.y() *= T(-1.);
    normalized_quaternion.z() *= T(-1.);
  }
  // We convert the normalized_quaternion into a vector along the rotation axis
  // with length of the rotation angle.
  const T angle = T(2.) * atan2(normalized_quaternion.vec().norm(),
                                normalized_quaternion.w());
  constexpr double kCutoffAngle = 1e-7;  // We linearize below this angle.
  const T scale = angle < kCutoffAngle ? T(2.) : angle / sin(angle / T(2.));
  return Eigen::Matrix<T, 3, 1>(scale * normalized_quaternion.x(),
                                scale * normalized_quaternion.y(),
                                scale * normalized_quaternion.z());
}

// Returns a quaternion representing the same rotation as the given 'angle_axis'
// vector.
template <typename T>
Eigen::Quaternion<T> AngleAxisVectorToRotationQuaternion(
    const Eigen::Matrix<T, 3, 1>& angle_axis) {
  T scale = T(0.5);
  T w = T(1.);
  constexpr double kCutoffAngle = 1e-8;  // We linearize below this angle.
  if (angle_axis.squaredNorm() > kCutoffAngle) {
    const T norm = angle_axis.norm();
    scale = sin(norm / 2.) / norm;
    w = cos(norm / 2.);
  }
  const Eigen::Matrix<T, 3, 1> quaternion_xyz = scale * angle_axis;
  return Eigen::Quaternion<T>(w, quaternion_xyz.x(), quaternion_xyz.y(),
                              quaternion_xyz.z());
}

// Projects 'transform' onto the XY plane.
template <typename T>
Rigid2<T> Project2D(const Rigid3<T>& transform) {
  return Rigid2<T>(transform.translation().template head<2>(),
                   GetYaw(transform));
}

// Embeds 'transform' into 3D space in the XY plane.
template <typename T>
Rigid3<T> Embed3D(const Rigid2<T>& transform) {
  return Rigid3<T>(
      {transform.translation().x(), transform.translation().y(), T(0)},
      Eigen::AngleAxis<T>(transform.rotation().angle(),
                          Eigen::Matrix<T, 3, 1>::UnitZ()));
}


}  // namespace transform
}  // namespace cartographer

#endif  // CARTOGRAPHER_TRANSFORM_TRANSFORM_H_
