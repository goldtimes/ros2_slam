#include <Eigen/Core>
#include <Eigen/Dense>
#include "sophus/se2.hpp"
#include "sophus/se3.hpp"
namespace slam {
using Vec2i = Eigen::Vector2i;
using Vec3i = Eigen::Vector3i;
using Vec3b = Eigen::Matrix<char, 3, 1>;
using V1D = Eigen::Matrix<double, 1, 1>;

using Vec2d = Eigen::Vector2d;
using Vec2f = Eigen::Vector2f;
using V3D = Eigen::Vector3d;
using Vec3f = Eigen::Vector3f;
using Vec4d = Eigen::Vector4d;
using Vec4f = Eigen::Vector4f;
using Vec5d = Eigen::Matrix<double, 5, 1>;
using Vec5f = Eigen::Matrix<float, 5, 1>;
using Vec6d = Eigen::Matrix<double, 6, 1>;
using Vec6f = Eigen::Matrix<float, 6, 1>;
using Vec9d = Eigen::Matrix<double, 9, 1>;
using V12D = Eigen::Matrix<double, 12, 1>;
using Vec15d = Eigen::Matrix<double, 15, 1>;
using Vec18d = Eigen::Matrix<double, 18, 1>;
using V21D = Eigen::Matrix<double, 21, 1>;
using V33D = Eigen::Matrix<double, 33, 1>;
using V34D = Eigen::Matrix<double, 34, 1>;

using Mat1d = Eigen::Matrix<double, 1, 1>;
using Mat2d = Eigen::Matrix<double, 2, 2>;
using Mat23d = Eigen::Matrix<double, 2, 3>;
using Mat32d = Eigen::Matrix<double, 3, 2>;
using M3D = Eigen::Matrix3d;
using Mat3f = Eigen::Matrix3f;
using Mat4d = Eigen::Matrix4d;
using Mat4f = Eigen::Matrix4f;
using Mat5d = Eigen::Matrix<double, 5, 5>;
using Mat5f = Eigen::Matrix<float, 5, 5>;
using Mat6d = Eigen::Matrix<double, 6, 6>;
using Mat6f = Eigen::Matrix<float, 6, 6>;
using Mat9d = Eigen::Matrix<double, 9, 9>;
using Mat96d = Eigen::Matrix<double, 9, 6>;
using M12D = Eigen::Matrix<double, 12, 12>;
using Mat15d = Eigen::Matrix<double, 15, 15>;
using Mat18d = Eigen::Matrix<double, 18, 18>;
using M21D = Eigen::Matrix<double, 21, 21>;
using M33D = Eigen::Matrix<double, 33, 33>;

using VecXd = Eigen::Matrix<double, -1, 1>;
using MatXd = Eigen::Matrix<double, -1, -1>;
using MatX18d = Eigen::Matrix<double, -1, 18>;

using Quatd = Eigen::Quaterniond;
using Quatf = Eigen::Quaternionf;

using Vector21d = Eigen::Matrix<double, 21, 1>;
using Vector12d = Eigen::Matrix<double, 12, 1>;
using Matrix21d = Eigen::Matrix<double, 21, 21>;
using Matrix12d = Eigen::Matrix<double, 12, 12>;
using Matrix21x12d = Eigen::Matrix<double, 21, 12>;
using Matrix23x12d = Eigen::Matrix<double, 23, 12>;
using Matrix33x12d = Eigen::Matrix<double, 33, 12>;

using Matrix23d = Eigen::Matrix<double, 23, 23>;
using Vector23d = Eigen::Matrix<double, 23, 1>;
using Vector24d = Eigen::Matrix<double, 24, 1>;
using Matrix3x2d = Eigen::Matrix<double, 3, 2>;
using Matrix2x3d = Eigen::Matrix<double, 2, 3>;

// const M3D Eye3d = M3D::Identity();
// const Mat3f Eye3f = Mat3f::Identity();
// const V3D Zero3d(0, 0, 0);
// const Vec3f Zero3f(0, 0, 0);

// pose represented as sophus structs
using SE2 = Sophus::SE2d;
using SE2f = Sophus::SE2f;
using SO2 = Sophus::SO2d;
using SE3 = Sophus::SE3d;
using SE3f = Sophus::SE3f;
using SO3 = Sophus::SO3d;
}  // namespace slam