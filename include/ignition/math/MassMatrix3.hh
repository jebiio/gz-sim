/*
 * Copyright (C) 2015 Open Source Robotics Foundation
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
*/
#ifndef _IGNITION_MASSMATRIX3_HH_
#define _IGNITION_MASSMATRIX3_HH_

#include <algorithm>
#include <string>
#include <vector>

#include "ignition/math/Quaternion.hh"
#include "ignition/math/Vector3.hh"
#include "ignition/math/Matrix3.hh"

namespace ignition
{
  namespace math
  {
    /// \class MassMatrix3 MassMatrix3.hh ignition/math/MassMatrix3.hh
    /// \brief A class for inertial information about a rigid body
    /// consisting of the scalar mass and a 3x3 symmetric moment
    /// of inertia matrix stored as two Vector3's.
    template<typename T>
    class MassMatrix3
    {
      /// \brief Default Constructor
      public: MassMatrix3() : mass(1), Ixxyyzz(1, 1, 1), Ixyxzyz(0, 0, 0)
      {}

      /// \brief Constructor.
      /// \param[in] _mass Mass value in kg if using metric.
      /// \param[in] _Ixxyyzz Diagonal moments of inertia.
      /// \param[in] _Ixyxzyz Off-diagonal moments of inertia
      public: MassMatrix3(const T &_mass,
                          const Vector3<T> &_ixxyyzz,
                          const Vector3<T> &_ixyxzyz )
      : mass(_mass), Ixxyyzz(_ixxyyzz), Ixyxzyz(_ixyxzyz)
      {}

      /// \brief Copy constructor.
      /// \param[in] _massMatrix MassMatrix3 element to copy
      public: MassMatrix3(const MassMatrix3<T> &_m)
      : mass(_m.Mass()), Ixxyyzz(_m.DiagonalMoments()),
        Ixyxzyz(_m.OffDiagonalMoments())
      {}

      /// \brief Destructor.
      public: virtual ~MassMatrix3() {}

      /// \brief Set the mass.
      /// \param[in] _m New mass value.
      /// \return True if the mass was set successfully.
      public: bool Mass(const T &_m)
      {
        // Should we only accept positive values?
        this->mass = _m;
        return true;
      }

      /// \brief Get the mass
      /// \return The mass value
      public: T Mass() const
      {
        return this->mass;
      }

      /// \brief Set the moment of inertia matrix.
      /// \param[in] _ixx X second moment of inertia (MOI) about x axis.
      /// \param[in] _iyy Y second moment of inertia about y axis.
      /// \param[in] _izz Z second moment of inertia about z axis.
      /// \param[in] _ixy XY inertia.
      /// \param[in] _ixz XZ inertia.
      /// \param[in] _iyz YZ inertia.
      /// \return True if the inertia matrix was set successfully.
      public: bool InertiaMatrix(const T &_ixx, const T &_iyy, const T &_izz,
                                 const T &_ixy, const T &_ixz, const T &_iyz)
      {
        // Should we validate the values?
        // matrix must be positive definite
        this->Ixxyyzz.Set(_ixx, _iyy, _izz);
        this->Ixyxzyz.Set(_ixy, _ixz, _iyz);
        return true;
      }

      /// \brief Get the diagonal moments of inertia (Ixx, Iyy, Izz).
      /// \return The diagonal moments.
      public: Vector3<T> DiagonalMoments() const
      {
        return this->Ixxyyzz;
      }

      /// \brief Get the off-diagonal moments of inertia (Ixy, Ixz, Iyz).
      /// \return The off-diagonal moments of inertia.
      public: Vector3<T> OffDiagonalMoments() const
      {
        return this->Ixyxzyz;
      }

      /// \brief Set the diagonal moments of inertia (Ixx, Iyy, Izz).
      /// \param[in] _ixxyyzz diagonal moments of inertia
      /// \return True if the moments were set successfully.
      public: bool DiagonalMoments(const Vector3<T> &_ixxyyzz)
      {
        // Should we validate?
        this->Ixxyyzz = _ixxyyzz;
        return true;
      }

      /// \brief Set the off-diagonal moments of inertia (Ixy, Ixz, Iyz).
      /// \param[in] _ixyxzyz off-diagonal moments of inertia
      /// \return True if the moments were set successfully.
      public: bool OffDiagonalMoments(const Vector3<T> &_ixyxzyz)
      {
        // Should we validate?
        this->Ixyxzyz = _ixyxzyz;
        return true;
      }

      /// \brief Get IXX
      /// \return IXX value
      public: T IXX() const
      {
        return this->Ixxyyzz[0];
      }

      /// \brief Get IYY
      /// \return IYY value
      public: T IYY() const
      {
        return this->Ixxyyzz[1];
      }

      /// \brief Get IZZ
      /// \return IZZ value
      public: T IZZ() const
      {
        return this->Ixxyyzz[2];
      }

      /// \brief Get IXY
      /// \return IXY value
      public: T IXY() const
      {
        return this->Ixyxzyz[0];
      }

      /// \brief Get IXZ
      /// \return IXZ value
      public: T IXZ() const
      {
        return this->Ixyxzyz[1];
      }

      /// \brief Get IYZ
      /// \return IYZ value
      public: T IYZ() const
      {
        return this->Ixyxzyz[2];
      }

      /// \brief Set IXX
      /// \param[in] _v IXX value
      /// \return True if the value was set successfully.
      public: bool IXX(const T &_v)
      {
        // Should we validate?
        this->Ixxyyzz.X(_v);
        return true;
      }

      /// \brief Set IYY
      /// \param[in] _v IYY value
      /// \return True if the value was set successfully.
      public: bool IYY(const T &_v)
      {
        // Should we validate?
        this->Ixxyyzz.Y(_v);
        return true;
      }

      /// \brief Set IZZ
      /// \param[in] _v IZZ value
      /// \return True if the value was set successfully.
      public: bool IZZ(const T &_v)
      {
        // Should we validate?
        this->Ixxyyzz.Z(_v);
        return true;
      }

      /// \brief Set IXY
      /// \param[in] _v IXY value
      /// \return True if the value was set successfully.
      public: bool IXY(const T &_v)
      {
        // Should we validate?
        this->Ixyxzyz.X(_v);
        return true;
      }

      /// \brief Set IXZ
      /// \param[in] _v IXZ value
      /// \return True if the value was set successfully.
      public: bool IXZ(const T &_v)
      {
        // Should we validate?
        this->Ixyxzyz.Y(_v);
        return true;
      }

      /// \brief Set IYZ
      /// \param[in] _v IYZ value
      /// \return True if the value was set successfully.
      public: bool IYZ(const T &_v)
      {
        // Should we validate?
        this->Ixyxzyz.Z(_v);
        return true;
      }

      /// \brief returns Moments of Inertia as a Matrix3
      /// \return Moments of Inertia as a Matrix3
      public: Matrix3<T> MOI() const
      {
        return Matrix3<T>(
          this->Ixxyyzz[0], this->Ixyxzyz[0], this->Ixyxzyz[1],
          this->Ixyxzyz[0], this->Ixxyyzz[1], this->Ixyxzyz[2],
          this->Ixyxzyz[1], this->Ixyxzyz[2], this->Ixxyyzz[2]);
      }

      /// \brief Sets Moments of Inertia (MOI) from a Matrix3.
      /// Symmetric component of input matrix is used by averaging
      /// off-axis terms.
      /// \param[in] Moments of Inertia as a Matrix3
      /// \return True if the inertia matrix was set successfully.
      public: bool MOI(const Matrix3<T> &_moi)
      {
        // Should we validate?
        this->Ixxyyzz.Set(_moi(0, 0), _moi(1, 1), _moi(2, 2));
        this->Ixyxzyz.Set(
          0.5*(_moi(0, 1) + _moi(1, 0)),
          0.5*(_moi(0, 2) + _moi(2, 0)),
          0.5*(_moi(1, 2) + _moi(2, 1)));
        return true;
      }

      /// \brief Equal operator.
      /// \param[in] _massMatrix MassMatrix3 to copy.
      /// \return Reference to this object.
      public: MassMatrix3 &operator=(const MassMatrix3<T> &_massMatrix)
      {
        this->mass = _massMatrix.Mass();
        this->Ixxyyzz = _massMatrix.DiagonalMoments();
        this->Ixyxzyz = _massMatrix.OffDiagonalMoments();

        return *this;
      }

      /// \brief Equality comparison operator.
      /// \param[in] _m MassMatrix3 to copy.
      /// \return true if each component is equal within a default tolerance,
      /// false otherwise
      public: bool operator==(const MassMatrix3<T> &_m) const
      {
        return equal<T>(this->mass, _m.Mass()) &&
               (this->Ixxyyzz == _m.DiagonalMoments()) &&
               (this->Ixyxzyz == _m.OffDiagonalMoments());
      }

      /// \brief Inequality test operator
      /// \param[in] _m MassMatrix3<T> to test
      /// \return True if not equal (using the default tolerance of 1e-6)
      public: bool operator!=(const MassMatrix3<T> &_m) const
      {
        return !(*this == _m);
      }

      /// \brief Verify that inertia values are positive definite
      /// \return True if mass is positive and moment of inertia matrix
      /// is positive definite.
      public: bool IsPositive() const
      {
        // Check if mass and determinants of all upper left submatrices
        // of moment of inertia matrix are positive
        return (this->mass > 0) &&
               (this->IXX() > 0) &&
               (this->IXX()*this->IYY() - this->IXY()*this->IXY() > 0) &&
               (this->MOI().Determinant() > 0);
      }

      /// \brief Verify that inertia values are positive definite
      /// and satisfy the triangle inequality.
      /// \return True if IsPositive and moment of inertia satisfies
      /// the triangle inequality.
      public: bool IsValid() const
      {
        Vector3<T> moments = this->PrincipalMoments();
        return this->IsPositive() &&
          moments[0] + moments[1] > moments[2] &&
          moments[1] + moments[2] > moments[0] &&
          moments[2] + moments[0] > moments[1];
      }

      /// \brief Compute principal moments of inertia,
      /// which are the eigenvalues of the moment of inertia matrix.
      /// \return Principal moments of inertia, sorted from
      /// smallest to largest.
      public: Vector3<T> PrincipalMoments() const
      {
        if ((this->IXY() == 0) && (this->IXZ() == 0) && (this->IYZ() == 0))
        {
          // Matrix is already diagonalized,
          // return sorted copy of Ixxyyzz
          std::vector<T> moments(3, 0);
          moments[0] = Ixxyyzz[0];
          moments[1] = Ixxyyzz[1];
          moments[2] = Ixxyyzz[2];
          std::sort(moments.begin(), moments.end());
          return Vector3<T>(moments[0], moments[1], moments[2]);
        }

        // Algorithm based on http://arxiv.org/abs/1306.6291v4
        // A Method for Fast Diagonalization of a 2x2 or 3x3 Real Symmetric
        // Matrix, by Maarten Kronenburg
        Vector3<T> Id(this->Ixxyyzz);
        Vector3<T> Ip(this->Ixyxzyz);
        // b = Ixx + Iyy + Izz
        T b = Id.Sum();
        // c = Ixx*Iyy - Ixy^2  +  Ixx*Izz - Ixz^2  +  Iyy*Izz - Iyz^2
        T c = Id[0]*Id[1] - std::pow(Ip[0], 2)
            + Id[0]*Id[2] - std::pow(Ip[1], 2)
            + Id[1]*Id[2] - std::pow(Ip[2], 2);
        // d = Ixx*Iyz^2 + Iyy*Ixz^2 + Izz*Ixy^2 - Ixx*Iyy*Izz - 2*Ixy*Ixz*Iyz
        T d = Id[0]*std::pow(Ip[2], 2)
            + Id[1]*std::pow(Ip[1], 2)
            + Id[2]*std::pow(Ip[0], 2)
            - Id[0]*Id[1]*Id[2]
            - 2*Ip[0]*Ip[1]*Ip[2];
        // p = b^2 - 3c
        T p = std::pow(b, 2) - 3*c;

        // p can also be expressed as a sum of squares (see eq 4.7)
        // so it must be non-negative (p >= 0)
        // Also, if p is zero (or close enough):
        //  then the diagonal terms must be close to zero
        //  and the three roots are equal
        if (p < 1e-18)
          return b / 3.0 * Vector3<T>::One;

        // q = 2b^3 - 9bc - 27d
        T q = 2*std::pow(b, 3) - 9*b*c - 27*d;

        // delta = acos(q / (2 * p^(1.5)))
        T delta = acos(0.5 * q / (p * sqrt(p)));

        std::vector<T> moments(3, 0);
        moments[0] = (b + 2*sqrt(p) * cos(delta / 3.0)) / 3.0;
        moments[1] = (b + 2*sqrt(p) * cos((delta + 2*M_PI)/3.0)) / 3.0;
        moments[2] = (b + 2*sqrt(p) * cos((delta - 2*M_PI)/3.0)) / 3.0;
        std::sort(moments.begin(), moments.end());
        return Vector3<T>(moments[0], moments[1], moments[2]);
      }

      /// \brief Compute rotational offset of principal axes.
      /// \return Quaternion representing rotational offset of principal axes.
      public: Quaternion<T> PrincipalAxesOffset() const
      {
        Vector3<T> moments = this->PrincipalMoments();
        if (moments == this->DiagonalMoments())
        {
          // matrix is already aligned with principal axes
          // return identity rotation
          return Quaternion<T>();
        }

        // Check if Diagonal Moments are all equal
        // Will this code ever be called?
        // I think the previous block will catch this
        Vector3<T> momentsDiff = Vector3<T>(
          moments[0] - moments[1],
          moments[1] - moments[2],
          moments[2] - moments[0]);
        if (momentsDiff == Vector3d::Zero)
        {
          // Diagonal Moments are all equal
          // eigenvectors for identity matrix
          // return identity rotation
          return Quaternion<T>();
        }

        // // Check if two moments are equal.
        // // The moments vector is already sorted,
        // // so just check adjacent values.
        // {
        //   // index of unequal moment
        //   int unequalMoment = -1;
        //   if (equal<T>(momentsDiff[0], 0)
        //     unequalMoment = 2;
        //   else if (equal<T>(momentsDiff[1], 0)
        //     unequalMoment = 0;

        //   if (unequalMoment >= 0)
        //   {
        //     // moments[1] is equal to one of the other moments
        //     // it is not equal to moments[unequalMoment]


        //     return Quaternion<T>()
        //   }
        // }

        return Quaternion<T>(0.5, 0.5, 0.5, 0.5);
      }

      /// \brief Get dimensions and rotation offset of uniform box
      /// with equivalent mass and moment of inertia.
      /// To compute this, the Matrix3 is diagonalized.
      /// The eigenvalues on the diagonal and the rotation offset
      /// of the principal axes are returned.
      /// \param[in] _size Dimensions of box aligned with principal axes.
      /// \param[in] _rot Rotational offset of principal axes.
      /// \return True if box properties were computed successfully.
      public: bool EquivalentBox(Vector3<T> &_size, Quaternion<T> &_rot) const
      {
        return true;
      }

      /// \brief Mass the object. Default is 1.0.
      private: T mass;

      /// \brief Principal moments of inertia. Default is (1.0 1.0 1.0)
      /// These Moments of Inertia are specified in the local frame.
      /// Where Ixxyyzz.x is Ixx, Ixxyyzz.y is Iyy and Ixxyyzz.z is Izz.
      private: Vector3<T> Ixxyyzz;

      /// \brief Product moments of inertia. Default is (0.0 0.0 0.0)
      /// These MOI off-diagonals are specified in the local frame.
      /// Where Ixyxzyz.x is Ixy, Ixyxzyz.y is Ixz and Ixyxzyz.z is Iyz.
      private: Vector3<T> Ixyxzyz;
    };

    typedef MassMatrix3<double> MassMatrix3d;
    typedef MassMatrix3<float> MassMatrix3f;
  }
}
#endif
