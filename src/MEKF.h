#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <numbers>
#include <stdexcept>
#include <string>
#include <vector>

namespace mekf {

    using Vector4d = Eigen::Matrix<double, 4, 1>;
    using Vector3d = Eigen::Matrix<double, 3, 1>;
    using Vector10d = Eigen::Matrix<double, 10, 1>;

    using Measurement = Eigen::Matrix<double, 11, 1>;
    using State = Eigen::Matrix<double, 16, 1>;
    using Error = Eigen::Matrix<double, 15, 1>;
    using Covariance = Eigen::Matrix<double, 15, 15>;

    struct NominalState {
        Vector4d quat{1.0, 0.0, 0.0, 0.0};
        Vector3d pos = Vector3d::Zero();
        Vector3d vel = Vector3d::Zero();
        Vector3d gyr_b = Vector3d::Zero();
        Vector3d acc_b = Vector3d::Zero();

        State asVector() const {
            State state;
            state << quat, vel, pos, gyr_b, acc_b;
            return state;
        };

        static NominalState fromVector(const State& x) {
            NominalState state;
            state.quat = x.segment<4>(0);
            state.vel = x.segment<3>(4);
            state.pos = x.segment<3>(7);
            state.gyr_b = x.segment<3>(10);
            state.acc_b = x.segment<3>(13);
            return state;
        };
    };

    struct ErrorState {
        Vector3d small_angle = Vector3d::Zero();
        Vector3d pos = Vector3d::Zero();
        Vector3d vel = Vector3d::Zero();
        Vector3d gyr_b = Vector3d::Zero();
        Vector3d acc_b = Vector3d::Zero();

        Error asVector() const {
            Error state;
            state << small_angle, vel, pos, gyr_b, acc_b;
            return state;
        };

        static ErrorState fromVector(const Error& x) {
            ErrorState state;
            state.small_angle = x.segment<3>(0);
            state.vel = x.segment<3>(3);
            state.pos = x.segment<3>(6);
            state.gyr_b = x.segment<3>(9);
            state.acc_b = x.segment<3>(12);
            return state;
        };
    };

    class MEKF {
        public:
            static constexpr double g_const = 9.80665;
            inline static const Vector3d g_vector{0, 0, 1};
            static constexpr double theta_const = 0;
            inline static const Vector3d north_vector{0, std::cos(theta_const), -std::sin(theta_const)};

            Eigen::Matrix3d gyro_cov_mat;
            Eigen::Matrix3d accel_cov_mat;
            Eigen::Matrix3d gyro_bias_cov_mat;
            Eigen::Matrix3d accel_bias_cov_mat;
            Eigen::Matrix3d gps_cov_mat;

            MEKF(const Vector4d& initial_q, const Vector3d& initial_vel,
                 const Vector3d& initial_pos, double gyro_cov, double accel_cov,
                 double gyro_bias_cov, double accel_bias_cov,
                 double gps_cov = 1.0, const Covariance* P_initial = nullptr)
                : gyro_cov_mat(gyro_cov * Eigen::Matrix3d::Identity()),
                  accel_cov_mat(accel_cov * Eigen::Matrix3d::Identity()),
                  gyro_bias_cov_mat(gyro_bias_cov * Eigen::Matrix3d::Identity()),
                  accel_bias_cov_mat(accel_bias_cov * Eigen::Matrix3d::Identity()),
                  gps_cov_mat(gps_cov * Eigen::Matrix3d::Identity()),
                  P(P_initial ? *P_initial : Covariance::Identity())
            {
                require_finite(initial_q, "initial_q");
                require_finite(initial_vel, "initial_vel");
                require_finite(initial_pos, "initial_pos");
                require_finite(P, "P_initial");
                // Preserve the initial quaternion exactly, as Python does.
                x.quat = initial_q;
                x.vel = initial_vel;
                x.pos = initial_pos;
            }

            void prediction(double dt, const Vector3d* gyr = nullptr,
                            const Vector3d* acc = nullptr) {
                require_dt(dt);
                if (gyr) require_finite(*gyr, "gyr");
                if (acc) require_finite(*acc, "acc");
                // Omission means zero corrected input, not a zero raw sample.
                Vector3d corrected_gyr = Vector3d::Zero();
                Vector3d corrected_acc = Vector3d::Zero();
                if (gyr) corrected_gyr = *gyr - x.gyr_b;
                if (acc) corrected_acc = *acc - x.acc_b;

                State predicted = x.asVector();
                predicted.head<10>() = f(dt, corrected_gyr, corrected_acc);
                x = NominalState::fromVector(predicted);
                // Python evaluates Phi and Q at the predicted attitude.
                const Covariance Phi = process_model(dt, corrected_gyr, corrected_acc);
                const Covariance Q = process_covariance(dt, corrected_gyr, corrected_acc);
                P = (Phi * P * Phi.transpose() + Q).eval();
            }

            void correction(double /*dt*/, const Vector3d* acc = nullptr,
                            const Vector3d* mag = nullptr, const Vector3d* gps = nullptr,
                            const double* lid = nullptr, const double* bar = nullptr) {
                std::vector<int> rows;
                Vector3d acc_norm = Vector3d::Zero();
                Vector3d mag_norm = Vector3d::Zero();
                if (acc) {
                    acc_norm = direction(*acc, "acc");
                    rows.insert(rows.end(), {0, 1, 2});
                }
                if (mag) {
                    mag_norm = direction(*mag, "mag");
                    rows.insert(rows.end(), {3, 4, 5});
                }
                if (gps) {
                    require_finite(*gps, "gps");
                    rows.insert(rows.end(), {6, 7, 8});
                }
                if (lid) {
                    if (!std::isfinite(*lid)) throw std::invalid_argument("lid must be finite");
                    rows.push_back(9);
                }
                if (bar) {
                    if (!std::isfinite(*bar)) throw std::invalid_argument("bar must be finite");
                    rows.push_back(10);
                }
                if (rows.empty()) return;

                const Eigen::Index m = static_cast<Eigen::Index>(rows.size());
                Eigen::VectorXd z(m);
                Eigen::Index offset = 0;
                if (acc) { z.segment<3>(offset) = acc_norm; offset += 3; }
                if (mag) { z.segment<3>(offset) = mag_norm; offset += 3; }
                if (gps) { z.segment<3>(offset) = *gps; offset += 3; }
                if (lid) z(offset++) = *lid;
                if (bar) z(offset++) = *bar;

                const Eigen::Matrix<double, 11, 15> H_full = H();
                const Eigen::Matrix<double, 11, 11> R_full = measurement_covariance();
                const Measurement h_full = h();
                Eigen::Matrix<double, Eigen::Dynamic, 15> H_k(m, 15);
                Eigen::MatrixXd R_k(m, m);
                Eigen::VectorXd h_k(m);
                for (Eigen::Index i = 0; i < m; ++i) {
                    H_k.row(i) = H_full.row(rows[i]);
                    h_k(i) = h_full(rows[i]);
                    for (Eigen::Index j = 0; j < m; ++j)
                        R_k(i, j) = R_full(rows[i], rows[j]);
                }

                const Eigen::Matrix<double, 15, Eigen::Dynamic> P_Ht = P * H_k.transpose();
                const Eigen::MatrixXd S = H_k * P_Ht + R_k;
                // Solve S^T K^T = (P H^T)^T without explicitly inverting S.
                const Eigen::FullPivLU<Eigen::MatrixXd> solve(S.transpose());
                if (!solve.isInvertible()) throw std::runtime_error("Singular innovation covariance");
                const Eigen::Matrix<double, 15, Eigen::Dynamic> K_k =
                    solve.solve(P_Ht.transpose()).transpose();
                const Error error = K_k * (z - h_k);
                Covariance posterior = (Covariance::Identity() - K_k * H_k) * P;

                Vector4d delta_q;
                delta_q << 1.0, 0.5 * error.head<3>();
                delta_q.normalize();
                x.quat = quat_multiply(x.quat, delta_q);
                x.vel += error.segment<3>(3);
                x.pos += error.segment<3>(6);
                x.gyr_b += error.segment<3>(9);
                x.acc_b += error.segment<3>(12);

                Covariance reset = Covariance::Identity();
                reset.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity()
                    - 0.5 * skewSymmetric(error.head<3>());
                P = reset * posterior * reset.transpose();
            }

            Vector10d f(double dt, const Vector3d& gyr, const Vector3d& acc) const {
                require_finite(gyr, "gyr");
                require_finite(acc, "acc");
                Vector4d q_new = (Eigen::Matrix4d::Identity() + 0.5 * dt * omega(gyr)) * x.quat;
                q_new.normalize();
                Vector3d vel_new = x.vel + rotation_body_to_earth(q_new) * acc * dt;
                vel_new(2) -= g_const * dt;
                const Vector3d pos_new = x.pos + vel_new * dt;
                Vector10d result;
                result << q_new, vel_new, pos_new;
                return result;
            }

            Covariance process_model(double dt, const Vector3d& gyr, const Vector3d& acc) const {
                require_finite(gyr, "gyr");
                require_finite(acc, "acc");
                const Eigen::Matrix3d R = quat_to_matrix(x.quat);
                Covariance G = Covariance::Zero();
                G.block<3, 3>(0, 0) = -skewSymmetric(gyr);
                G.block<3, 3>(3, 0) = -R * skewSymmetric(acc);
                G.block<3, 3>(3, 12) = -R;
                G.block<3, 3>(6, 3) = Eigen::Matrix3d::Identity();
                G.block<3, 3>(0, 9) = -Eigen::Matrix3d::Identity();
                return Covariance::Identity() + G * dt;
            }

            // Inputs are already bias-corrected. Attitude is the current nominal attitude.
            Covariance process_covariance(double dt,
                    const Vector3d& gyr = Vector3d::Zero(),
                    const Vector3d& acc = Vector3d::Zero()) const {
                require_dt(dt);
                require_finite(gyr, "gyr");
                require_finite(acc, "acc");
                Covariance Q = Covariance::Zero();
                if (dt == 0.0) return Q;
                const double dt2 = dt * dt;
                const double dt3 = dt2 * dt;
                const double dt4 = dt3 * dt;
                const double dt5 = dt4 * dt;
                const Eigen::Matrix3d R = rotation_body_to_earth(x.quat);

                Q.block<3, 3>(0, 0) = gyro_bias_cov_mat * (dt3 / 3.0);
                Q.block<3, 3>(9, 0) = -gyro_bias_cov_mat * (dt2 / 2.0);
                Q.block<3, 3>(0, 9) = Q.block<3, 3>(9, 0);
                Q.block<3, 3>(9, 9) = gyro_bias_cov_mat * dt;
                Q.block<3, 3>(3, 3) = accel_cov_mat * dt + accel_bias_cov_mat * (dt3 / 3.0);
                Q.block<3, 3>(6, 3) = accel_cov_mat * (dt2 / 2.0) + accel_bias_cov_mat * (dt4 / 8.0);
                Q.block<3, 3>(3, 6) = Q.block<3, 3>(6, 3);
                Q.block<3, 3>(12, 3) = -accel_bias_cov_mat * R.transpose() * (dt2 / 2.0);
                Q.block<3, 3>(3, 12) = Q.block<3, 3>(12, 3).transpose();
                Q.block<3, 3>(6, 6) = accel_cov_mat * (dt3 / 3.0) + accel_bias_cov_mat * (dt5 / 20.0);
                Q.block<3, 3>(12, 6) = -accel_bias_cov_mat * R.transpose() * (dt3 / 6.0);
                Q.block<3, 3>(6, 12) = Q.block<3, 3>(12, 6).transpose();
                Q.block<3, 3>(12, 12) = accel_bias_cov_mat * dt;

                const Vector3d half_step = 0.5 * dt * gyr;
                const Eigen::Matrix3d attitude_jacobian = dt / (1.0 + half_step.squaredNorm())
                    * (Eigen::Matrix3d::Identity() - skewSymmetric(half_step));
                const Eigen::Matrix3d acceleration_jacobian = -R * skewSymmetric(acc);
                Eigen::Matrix<double, 15, 3> gyro_jacobian = Eigen::Matrix<double, 15, 3>::Zero();
                gyro_jacobian.block<3, 3>(0, 0) = attitude_jacobian;
                gyro_jacobian.block<3, 3>(3, 0) = acceleration_jacobian * attitude_jacobian * dt;
                gyro_jacobian.block<3, 3>(6, 0) = gyro_jacobian.block<3, 3>(3, 0) * dt;
                Q += gyro_jacobian * (gyro_cov_mat / dt) * gyro_jacobian.transpose();
                return Q;
            }

            Measurement h() const {
                const Eigen::Matrix3d R = rotation_earth_to_body(x.quat);
                Measurement result;
                result << R * g_vector, R * north_vector, x.pos, x.pos(2), x.pos(2);
                return result;
            }

            Eigen::Matrix<double, 11, 15> H() const {
                const Eigen::Matrix3d R = rotation_earth_to_body(x.quat);
                Eigen::Matrix<double, 11, 15> result = Eigen::Matrix<double, 11, 15>::Zero();
                result.block<3, 3>(0, 0) = skewSymmetric(R * g_vector);
                result.block<3, 3>(0, 12) = Eigen::Matrix3d::Identity();
                result.block<3, 3>(3, 0) = skewSymmetric(R * north_vector);
                result.block<3, 3>(3, 12) = Eigen::Matrix3d::Identity();
                result.block<3, 3>(6, 6) = Eigen::Matrix3d::Identity();
                result(9, 8) = 1.0;
                result(10, 8) = 1.0;
                return result;
            }

            Eigen::Matrix<double, 11, 11> measurement_covariance() const {
                Eigen::Matrix<double, 11, 11> R = Eigen::Matrix<double, 11, 11>::Identity();
                R.block<3, 3>(6, 6) = gps_cov_mat;
                return R;
            }

            Vector4d q() const { return x.quat; }
            Vector3d vel() const { return x.vel; }
            Vector3d pos() const { return x.pos; }
            Vector3d gyr_b() const { return x.gyr_b; }
            Vector3d acc_b() const { return x.acc_b; }
            Covariance cov() const { return P; }
            State state_vector() const { return x.asVector(); }
            void set_state(const State& state) {
                require_finite(state, "state");
                x = NominalState::fromVector(state);
            }
            Error mean() const {
                Error result;
                result << quat_to_euler(x.quat), x.vel, x.pos, x.gyr_b, x.acc_b;
                return result;
            }

            static Eigen::Matrix4d omega(const Vector3d& gyr) {
                require_finite(gyr, "gyr");
                const double gp = gyr(0), gq = gyr(1), gr = gyr(2);
                Eigen::Matrix4d result;
                result << 0.0, -gp, -gq, -gr,
                          gp, 0.0, gr, -gq,
                          gq, -gr, 0.0, gp,
                          gr, gq, -gp, 0.0;
                return result;
            }

            static Eigen::Matrix3d rotation_body_to_earth(const Vector4d& q) {
                require_finite(q, "q");
                return quat_to_matrix(q);
            }

            static Eigen::Matrix3d rotation_earth_to_body(const Vector4d& q) {
                return rotation_body_to_earth(q).transpose();
            }

        private:
            NominalState x;
            Covariance P;

            template <typename Derived>
            static void require_finite(const Eigen::MatrixBase<Derived>& value, const char* name) {
                if (!value.allFinite())
                    throw std::invalid_argument(std::string(name) + " must contain finite values");
            }

            static void require_dt(double dt) {
                if (!std::isfinite(dt) || dt < 0.0)
                    throw std::invalid_argument("dt must be finite and nonnegative");
            }

            static Vector3d direction(const Vector3d& value, const char* name) {
                require_finite(value, name);
                const double norm = value.norm();
                if (!std::isfinite(norm) || norm <= 1e-12)
                    throw std::invalid_argument(std::string(name) + " must have a finite norm greater than 1e-12");
                return value / norm;
            }

            // helpers

            static Vector4d quat_multiply(const Vector4d& p, const Vector4d& q) {
                Vector4d r;
                r(0) = p(0) * q(0) - p(1) * q(1) - p(2) * q(2) - p(3) * q(3);
                r(1) = p(0) * q(1) + p(1) * q(0) + p(2) * q(3) - p(3) * q(2);
                r(2) = p(0) * q(2) - p(1) * q(3) + p(2) * q(0) + p(3) * q(1);
                r(3) = p(0) * q(3) + p(1) * q(2) - p(2) * q(1) + p(3) * q(0);
                return r;
            }

            static Eigen::Matrix<double,3,3> skewSymmetric(const Vector3d& v) {
                Eigen::Matrix<double,3,3> ss;
                ss << 0.0, -v(2), v(1),
                    v(2), 0.0, -v(0),
                    -v(1), v(0), 0.0;
                return ss;
            }

            // Scalar-first quaternion [w, x, y, z]; body-to-earth rotation.
            // As in the Python formula, the caller supplies a unit quaternion.
            static Eigen::Matrix3d quat_to_matrix(const Vector4d& q) {
                const double w = q(0);
                const Vector3d v = q.tail<3>();
                return (w * w - v.squaredNorm()) * Eigen::Matrix3d::Identity()
                     + 2.0 * v * v.transpose()
                     + 2.0 * w * skewSymmetric(v);
            }

            static Vector3d quat_to_euler(const Vector4d& q) {
                Vector3d euler_angles;
                // Roll
                double sinr_cosp = 2 * (q(0) * q(1) + q(2) * q(3));
                double cosr_cosp = 1 - 2 * (q(1) * q(1) + q(2) * q(2));
                double roll = std::atan2(sinr_cosp, cosr_cosp);

                // Pitch
                double pitch;
                double sinp = 2 * (q(0) * q(2) - q(1) * q(3));
                // Gimbal lock
                if (std::abs(sinp) >= 1) {
                    pitch = std::copysign(std::numbers::pi / 2.0, sinp);
                } else {
                    pitch = std::asin(sinp);
                }

                // Yaw
                double siny_cosp  = 2 * (q(0) * q(3) + q(1) * q(2));
                double cosy_cosp = 1 - 2 * (q(2) * q(2) + q(3) * q(3));
                double yaw = std::atan2(siny_cosp, cosy_cosp);

                euler_angles << roll, pitch, yaw;

                return euler_angles;
            }
    };

}
