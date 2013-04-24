#ifndef RATSMATRIX_H
#define RATSMATRIX_H
#include <hrpUtil/Eigen3d.h>
#include <iostream>

namespace rats
{
  inline bool eps_eq(const double a, const double b, const double eps = 0.001)
  {
    return fabs((a)-(b)) <= eps;
  };
  hrp::Vector3 matrix_log(const hrp::Matrix33& m);
  void rotation_matrix(hrp::Matrix33& rm, const double theta, const hrp::Vector3& axis);
  inline hrp::Matrix33 rotation_matrix(const double theta, const hrp::Vector3& axis) {
    hrp::Matrix33 ret;
    rotation_matrix(ret, theta, axis);
    return ret;
  }

  void print_vector(std::ostream& strm, const hrp::Vector3& vec, const bool use_newline = true);
  void print_matrix(std::ostream& strm, const hrp::Matrix33& mat, const bool use_newline = true);
  // matrix product using quaternion normalization
  void rotm3times (hrp::Matrix33& m12, const hrp::Matrix33& m1, const hrp::Matrix33& m2);
  void difference_rotation(hrp::Vector3& ret_dif_rot, const hrp::Matrix33& self_rot, const hrp::Matrix33& target_rot);

  // /* implementation of template functions */
  // void print_vector(std::ostream& strm, const hrp::Vector3& v, const bool use_newline = true) {
  //   print_vector(strm, v.data(), use_newline);
  // }

  // void print_matrix(std::ostream& strm, const hrp::Matrix33& m, const bool use_newline = true) {
  //   print_matrix(strm, m.data(), 3, 3, use_newline);
  // }

  struct coordinates {
    hrp::Vector3 pos;
    hrp::Matrix33 rot;
    coordinates() : pos(hrp::Vector3::Zero()), rot(hrp::Matrix33::Identity()) {};
    coordinates(const hrp::Vector3& p, const hrp::Matrix33& r) : pos(p), rot(r) {};
    coordinates(const hrp::Vector3& p) : pos(p), rot(hrp::Matrix33::Identity()) {};
    coordinates(const hrp::Matrix33& r) : pos(hrp::Vector3::Zero()), rot(r) {};
    coordinates(const coordinates& c) : pos(c.pos), rot(c.rot) {};
    virtual ~coordinates() {
    }
    coordinates& operator=(const coordinates& c) {
      if (this != &c) {
        pos = c.pos;
        rot = c.rot;
      }
      return *this;
    }
    //void print_eus_coordinates(std::ostream& strm, const bool use_newline = true) const; /* for euslisp format print function */
    void print_eus_coordinates(std::ostream& strm, const bool use_newline = true) const
    {
      strm << "#s(coordinates pos "; print_vector(strm, hrp::Vector3(1e3*pos), false);
      strm << " rot "; print_matrix(strm, rot, false); strm << ")";
      if (use_newline) strm << std::endl;
    }
    void translate(const hrp::Vector3& v, const std::string& wrt = ":local") {
      if (wrt == ":local") {
        pos += rot * v;
      } else if(wrt == ":world") {
        pos += v;
      } else {
        std::cerr << "**** invalid wrt! ****" << std::endl;
      }
    }
    void rotate_vector(hrp::Vector3& ret, const hrp::Vector3 &v) const {
      ret = rot * v;
    }
    void rotate_with_matrix (const hrp::Matrix33& mat, const std::string& wrt = ":local") {
      hrp::Matrix33 rot_org(rot);
      if (wrt == ":local") {		  
        //			alias(rot) = rot * mat;
        rotm3times(rot, rot_org, mat);
      } else if(wrt == ":world") {
        //			alias(rot) = mat * rot;
        rotm3times(rot, mat, rot_org);
      } else {
        std::cerr << "**** invalid wrt! ****" << std::endl;
      }
    }
    void rotate (const double theta, const hrp::Vector3& axis, const std::string& wrt = ":local") {
      rotate_with_matrix(rotation_matrix(theta, axis), wrt);
    }
    void difference_position(hrp::Vector3& dif_pos, const coordinates& c) const {
      dif_pos =  c.pos - pos;
    }
    /* void difference_rotation(hrp::Vector3& dif_rot, const coordinates& c) const { */
    /*   dif_rot = rot * matrix_log(rot.transpose() * c.rot); */
    /* } */
    void difference(hrp::Vector3& dif_pos, hrp::Vector3& dif_rot, const coordinates& c) const {
      difference_position(dif_pos, c);
      difference_rotation(dif_rot, rot, c.rot);
    }
    //abc
    void inverse_transformation(coordinates& inv) const {
      inv.rot = rot.transpose();
      inv.pos = inv.rot*(-1 * pos);
    }
    void transformation(coordinates& tc, coordinates c, const std::string& wrt = ":local") const {
      tc = *this;
      inverse_transformation(tc);
      if (wrt == ":local") {
        tc.transform(c);
      } else if(wrt == ":world") {
        c.transform(tc);
        tc = c;
      } else {
        std::cerr << "**** invalid wrt! ****" << std::endl;
      }
    }
    void transform(const coordinates& c, const std::string& wrt = ":local") {
      if (wrt == ":local") {
        pos += rot * c.pos;
        // alias(rot) = rot * c.rot;
        hrp::Matrix33 rot_org(rot);
        rotm3times(rot, rot_org, c.rot);
      } else if (wrt == ":world") {
        hrp::Vector3 p(c.pos);
        hrp::Matrix33 r(c.rot);      
        p += r * pos; 
        rotm3times(r, c.rot, rot);      
        pos = p;
        rot = r;
      } else {
        std::cerr << "**** invalid wrt! ****" << std::endl;
      }
    }
  };

  void outer_product_matrix(hrp::Matrix33 &ret, const hrp::Vector3 &v);
  void matrix_exponent(hrp::Matrix33& mexp, const hrp::Vector3& omega, double p);
  void mid_coords(coordinates& mid_coords, const double p, const coordinates& c1, const coordinates& c2);
};
#endif /* RATSMATRIX_H */
