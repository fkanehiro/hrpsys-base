/* -*- coding:utf-8-unix mode:c++ -*- */
#ifndef PREVIEW_H_
#define PREVIEW_H_
#include <iostream>
#include <queue>
#include <deque>
#include <hrpUtil/Eigen3d.h>
#include "hrpsys/util/Hrpsys.h"

namespace rats
{
  static const double DEFAULT_GRAVITATIONAL_ACCELERATION = 9.80665; // [m/s^2]

  template <std::size_t dim>
  struct riccati_equation
  {
    Eigen::Matrix<double, dim, dim> A;
    Eigen::Matrix<double, dim, 1> b;
    Eigen::Matrix<double, 1, dim> c;
    Eigen::Matrix<double, dim, dim> P;
    Eigen::Matrix<double, 1, dim> K;
    Eigen::Matrix<double, dim, dim> A_minus_bKt; /* for buffer */
    double Q, R;
    double R_btPb_inv; /* for buffer */

    riccati_equation() : A(Eigen::Matrix<double, dim, dim>::Zero()), b(Eigen::Matrix<double, dim, 1>::Zero()), c(Eigen::Matrix<double, 1, dim>::Zero()),
                         P(Eigen::Matrix<double, dim, dim>::Zero()), K(Eigen::Matrix<double, 1, dim>::Zero()),
                         A_minus_bKt(Eigen::Matrix<double, dim, dim>::Zero()), Q(0), R(0), R_btPb_inv(0) {};
    riccati_equation(const Eigen::Matrix<double, dim, dim>& _A, const Eigen::Matrix<double, dim, 1>& _b,
                     const Eigen::Matrix<double, 1, dim>& _c, const double _Q, const double _R)
      : A(_A), b(_b), c(_c), P(Eigen::Matrix<double, dim, dim>::Zero()), K(Eigen::Matrix<double, 1, dim>::Zero()), A_minus_bKt(Eigen::Matrix<double, dim, dim>::Zero()), Q(_Q), R(_R), R_btPb_inv(0) {};
    virtual ~riccati_equation() {};
    bool solve() {
      Eigen::Matrix<double, dim, dim> prev_P;
      for (int i = 0; i < 10000; i++) {
        R_btPb_inv = (1.0 / (R + (b.transpose() * P * b)(0,0)));
        Eigen::Matrix<double, dim, dim> tmp_pa(P * A);
        K = R_btPb_inv * b.transpose() * tmp_pa;
        prev_P = A.transpose() * tmp_pa + c.transpose() * Q * c - A.transpose() * P * b * K;
        if ((abs((P - prev_P).array()) < 5.0e-10).all()) {
          A_minus_bKt = (A - b * K).transpose();
          return true;
        }
        P = prev_P;
      }
      return false;
    }
  };

  template <std::size_t dim>
  class preview_control_base
  {
  protected:
    riccati_equation<dim> riccati;
    Eigen::Matrix<double, 3, 3> tcA;
    Eigen::Matrix<double, 3, 1> tcb;
    Eigen::Matrix<double, 1, 3> tcc;
    Eigen::Matrix<double, 3, 2> x_k;
    Eigen::Matrix<double, 1, 2> u_k;
    hrp::dvector f;
    std::deque<Eigen::Matrix<double, 2, 1> > p;
    std::deque<double> pz;
    std::deque< std::vector<hrp::Vector3> > qdata;
    double zmp_z, cog_z;
    size_t delay, ending_count;
    virtual void calc_f() = 0;
    virtual void calc_u() = 0;
    virtual void calc_x_k() = 0;
    void init_riccati(const Eigen::Matrix<double, dim, dim>& A,
                      const Eigen::Matrix<double, dim, 1>& b,
                      const Eigen::Matrix<double, 1, dim>& c,
                      const double q = 1.0, const double r = 1.0e-6)
    {
      riccati = riccati_equation<dim>(A, b, c, q, r);
      riccati.solve();
      calc_f();
    };
    /* inhibit copy constructor and copy insertion not by implementing */
    preview_control_base (const preview_control_base& _p);
    preview_control_base &operator=(const preview_control_base &_p);
  public:
    /* dt = [s], zc = [mm], d = [s] */
    preview_control_base(const double dt, const double zc,
                         const hrp::Vector3& init_xk, const double _gravitational_acceleration, const double d = 1.6)
      : riccati(), x_k(Eigen::Matrix<double, 3, 2>::Zero()), u_k(Eigen::Matrix<double, 1, 2>::Zero()), p(), pz(), qdata(),
        zmp_z(0), cog_z(zc), delay(static_cast<size_t>(round(d / dt))), ending_count(1+delay)
    {
      tcA << 1, dt, 0.5 * dt * dt,
        0, 1,  dt,
        0, 0,  1;
      tcb << 1 / 6.0 * dt * dt * dt,
        0.5 * dt * dt,
        dt;
      tcc << 1.0, 0.0, -zc / _gravitational_acceleration;
      x_k(0,0) = init_xk(0);
      x_k(0,1) = init_xk(1);
    };
    virtual ~preview_control_base()
    {
      p.clear();
      pz.clear();
      qdata.clear();
    };
    virtual void update_x_k(const hrp::Vector3& pr, const std::vector<hrp::Vector3>& qdata);
    virtual void update_x_k()
    {
      hrp::Vector3 pr;
      pr(0) = p.back()(0);
      pr(1) = p.back()(1);
      pr(2) = pz.back();
      update_x_k(pr, qdata.back());
      ending_count--;
    };
    // void update_zc(double zc);
    size_t get_delay () { return delay; };
    double get_preview_f (const size_t idx) { return f(idx); };
    void get_refcog (double* ret)
    {
      ret[0] = x_k(0,0);
      ret[1] = x_k(0,1);
      ret[2] = cog_z;
    };
    void get_refcog_vel (double* ret)
    {
      ret[0] = x_k(1,0);
      ret[1] = x_k(1,1);
      ret[2] = 0;
    };
    void get_refcog_acc (double* ret)
    {
      ret[0] = x_k(2,0);
      ret[1] = x_k(2,1);
      ret[2] = 0;
    };
    void get_cart_zmp (double* ret)
    {
      Eigen::Matrix<double, 1, 2> _p(tcc * x_k);
      ret[0] = _p(0, 0);
      ret[1] = _p(0, 1);
      ret[2] = pz.front();
    };
    void get_current_refzmp (double* ret)
    {
      ret[0] = p.front()(0);
      ret[1] = p.front()(1);
      ret[2] = pz.front();
    };
    void get_current_qdata (std::vector<hrp::Vector3>& _qdata)
    {
        _qdata = qdata.front();
    };
    bool is_doing () { return p.size() >= 1 + delay; };
    bool is_end () { return ending_count <= 0 ; };
    void remove_preview_queue(const size_t remain_length)
    {
      size_t num = p.size() - remain_length;
      for (size_t i = 0; i < num; i++) {
        p.pop_back();
        pz.pop_back();
        qdata.pop_back();
      }
    };
    void remove_preview_queue() // Remove all queue
    {
        p.clear();
        pz.clear();
        qdata.clear();
    };
    void set_preview_queue(const hrp::Vector3& pr, const std::vector<hrp::Vector3>& q, const size_t idx)
    {
      Eigen::Matrix<double, 2, 1> tmpv;
      tmpv(0,0) = pr(0);
      tmpv(1,0) = pr(1);
      p[idx] = tmpv;
      pz[idx] = pr(2);
      qdata[idx] = q;
    };
    size_t get_preview_queue_size()
    {
      return p.size();
    };
    void print_all_queue ()
    {
      std::cerr << "(list ";
      for (size_t i = 0; i < p.size(); i++) {
        std::cerr << "#f(" << p[i](0) << " " << p[i](1) << ") ";
      }
      std::cerr << ")" << std::endl;
    }
  };

  class preview_control : public preview_control_base<3>
  {
  private:
    void calc_f();
    void calc_u();
    void calc_x_k();
  public:
    preview_control(const double dt, const double zc,
                    const hrp::Vector3& init_xk, const double _gravitational_acceleration = DEFAULT_GRAVITATIONAL_ACCELERATION, const double q = 1.0,
                    const double r = 1.0e-6, const double d = 1.6)
        : preview_control_base<3>(dt, zc, init_xk, _gravitational_acceleration, d)
    {
      init_riccati(tcA, tcb, tcc, q, r);
    };
    virtual ~preview_control() {};
  };

  class extended_preview_control : public preview_control_base<4>
  {
  private:
    Eigen::Matrix<double, 4, 2> x_k_e;
    void calc_f();
    void calc_u();
    void calc_x_k();
  public:
    extended_preview_control(const double dt, const double zc,
                             const hrp::Vector3& init_xk, const double _gravitational_acceleration = DEFAULT_GRAVITATIONAL_ACCELERATION, const double q = 1.0,
                             const double r = 1.0e-6, const double d = 1.6)
      : preview_control_base<4>(dt, zc, init_xk, _gravitational_acceleration, d), x_k_e(Eigen::Matrix<double, 4, 2>::Zero())
    {
      Eigen::Matrix<double, 4, 4> A;
      Eigen::Matrix<double, 4, 1> b;
      Eigen::Matrix<double, 1, 4> c;
      Eigen::Matrix<double, 1, 3> tmpca(tcc * tcA);
      Eigen::Matrix<double, 1, 1> tmpcb(tcc * tcb);
      A << 1.0, tmpca(0,0), tmpca(0,1), tmpca(0,2),
        0.0, tcA(0,0), tcA(0,1), tcA(0,2),
        0.0, tcA(1,0), tcA(1,1), tcA(1,2),
        0.0, tcA(2,0), tcA(2,1), tcA(2,2);
      b << tmpcb(0,0),
        tcb(0,0),
        tcb(1,0),
        tcb(2,0);
      c << 1,0,0,0;
      x_k_e(0,0) = init_xk(0);
      x_k_e(0,1) = init_xk(1);
      init_riccati(A, b, c, q, r);
    };
    virtual ~extended_preview_control() {};
  };

  template <class previw_T>
  class preview_dynamics_filter
  {
    previw_T preview_controller;
    bool finishedp;
  public:
    preview_dynamics_filter() {};
    preview_dynamics_filter(const double dt, const double zc, const hrp::Vector3& init_xk, const double _gravitational_acceleration = DEFAULT_GRAVITATIONAL_ACCELERATION, const double q = 1.0, const double r = 1.0e-6, const double d = 1.6)
        : preview_controller(dt, zc, init_xk, _gravitational_acceleration, q, r, d), finishedp(false) {};
    ~preview_dynamics_filter() {};
    bool update(hrp::Vector3& p_ret, hrp::Vector3& x_ret, std::vector<hrp::Vector3>& qdata_ret, const hrp::Vector3& pr, const std::vector<hrp::Vector3>& qdata, const bool updatep)
    {
      bool flg;
      if (updatep) {
        preview_controller.update_x_k(pr, qdata);
        flg = preview_controller.is_doing();
      } else {
        if ( !preview_controller.is_end() )
          preview_controller.update_x_k();
        flg = !preview_controller.is_end();
      }

      if (flg) {
        preview_controller.get_current_refzmp(p_ret.data());
        preview_controller.get_refcog(x_ret.data());
        preview_controller.get_current_qdata(qdata_ret);
      }
      return flg;
    };
    void remove_preview_queue(const size_t remain_length)
    {
      preview_controller.remove_preview_queue(remain_length);
    };
    void remove_preview_queue()
    {
      preview_controller.remove_preview_queue();
    };
    void set_preview_queue(const hrp::Vector3& pr, const std::vector<hrp::Vector3>& qdata, const size_t idx)
    {
      preview_controller.set_preview_queue(pr, qdata, idx);
    }
    size_t get_preview_queue_size()
    {
      return preview_controller.get_preview_queue_size();
    };
    void print_all_queue ()
    {
      preview_controller.print_all_queue();
    }

    void get_cart_zmp (double* ret) { preview_controller.get_cart_zmp(ret);}
    void get_refcog_vel (double* ret) { preview_controller.get_refcog_vel(ret);}
    void get_refcog_acc (double* ret) { preview_controller.get_refcog_acc(ret);}
    void get_current_refzmp (double* ret) { preview_controller.get_current_refzmp(ret);}
    //void get_current_qdata (double* ret) { preview_controller.get_current_qdata(ret);}
    size_t get_delay () { return preview_controller.get_delay(); };
    double get_preview_f (const size_t idx) { return preview_controller.get_preview_f(idx); };
  };
}
#endif /*PREVIEW_H_*/
