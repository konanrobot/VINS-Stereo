#include "projection_factor.h"

Eigen::Matrix4d ProjectionFactor::sqrt_info;
double ProjectionFactor::sum_t;

#ifdef UNIT_SPHERE_ERROR  
#undef UNIT_SPHERE_ERROR  
#endif

ProjectionFactor::ProjectionFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j) : pts_i(_pts_i), pts_j(_pts_j)
{
#ifdef UNIT_SPHERE_ERROR
    Eigen::Vector3d b1, b2;
    Eigen::Vector3d a = pts_j.normalized();
    Eigen::Vector3d tmp(0, 0, 1);
    if(a == tmp)
        tmp << 1, 0, 0;
    b1 = (tmp - a * (a.transpose() * tmp)).normalized();
    b2 = a.cross(b1);
    tangent_base.block<1, 3>(0, 0) = b1.transpose();
    tangent_base.block<1, 3>(1, 0) = b2.transpose();
#endif
};

ProjectionFactor::ProjectionFactor(const Eigen::Vector3d &_pts_i, const Eigen::Vector3d &_pts_j, const Eigen::Vector3d &_pts_i_r, const Eigen::Vector3d &_pts_j_r) : pts_i(_pts_i), pts_j(_pts_j), pts_i_r(_pts_i_r), pts_j_r(_pts_j_r)
{
#ifdef UNIT_SPHERE_ERROR
    Eigen::Vector3d b1, b2;
    Eigen::Vector3d a = pts_j.normalized();
    Eigen::Vector3d tmp(0, 0, 1);
    if(a == tmp)
        tmp << 1, 0, 0;
    b1 = (tmp - a * (a.transpose() * tmp)).normalized();
    b2 = a.cross(b1);
    tangent_base.block<1, 3>(0, 0) = b1.transpose();
    tangent_base.block<1, 3>(1, 0) = b2.transpose();
#endif
};


bool ProjectionFactor::Evaluate(double const *const *parameters, double *residuals, double **jacobians) const
{
    TicToc tic_toc;
    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);

    double inv_dep_i = parameters[3][0];

    Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
    Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
    Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
    Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
    Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);
    Eigen::Map<Eigen::Vector4d> residual(residuals);

#ifdef UNIT_SPHERE_ERROR 
    residual =  tangent_base * (pts_camera_j.normalized() - pts_j.normalized());
#else
    // left eye
    double dep_j = pts_camera_j.z();
    residual.head<2>() = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();

#if 0
    0.9999937376404091, -0.00026539258488127593, -0.0035290574856257782, -0.07480350977811673,
    0.0002588685222140577, 0.9999982570946933, -0.0018489982856398398, 0.0003216683110021111,
    0.003529542045247232, 0.001848073144651583, 0.9999920634478051, -0.0005091817629381968,
    0.0, 0.0, 0.0, 1.0
#endif
    // right eye
    // qic: q from C to I
    Eigen::Vector3d tc21(-0.07480350977811673,0.0003216683110021111,-0.0005091817629381968);  // ....
    Eigen::Quaterniond qc21(0.99999797, 0.00092426967, -0.0017646535, 0.00013106555); //... (w,x,y,z)
    Eigen::Vector3d pts_camera_j_r = qc21 * pts_camera_j + tc21;
    double dep_j_r = pts_camera_j_r.z();
    residual.tail<2>() = (pts_camera_j_r / dep_j_r).head<2>() - pts_j_r.head<2>();
    
#endif

   residual = sqrt_info * residual;

    if (jacobians)
    {
        Eigen::Matrix3d Ri = Qi.toRotationMatrix();
        Eigen::Matrix3d Rj = Qj.toRotationMatrix();
        Eigen::Matrix3d ric = qic.toRotationMatrix();
        Eigen::Matrix<double, 4, 6> reduce(4, 6);

	Eigen::Matrix3d Rc21 = qc21.toRotationMatrix();
	
#ifdef UNIT_SPHERE_ERROR
        double norm = pts_camera_j.norm();
        Eigen::Matrix3d norm_jaco;
        double x1, x2, x3;
        x1 = pts_camera_j(0);
        x2 = pts_camera_j(1);
        x3 = pts_camera_j(2);
        norm_jaco << 1.0 / norm - x1 * x1 / pow(norm, 3), - x1 * x2 / pow(norm, 3),            - x1 * x3 / pow(norm, 3),
                     - x1 * x2 / pow(norm, 3),            1.0 / norm - x2 * x2 / pow(norm, 3), - x2 * x3 / pow(norm, 3),
                     - x1 * x3 / pow(norm, 3),            - x2 * x3 / pow(norm, 3),            1.0 / norm - x3 * x3 / pow(norm, 3);
        reduce = tangent_base * norm_jaco;
#else
        reduce <<
	  1. / dep_j, 0, -pts_camera_j(0) / (dep_j * dep_j), 0, 0, 0,
	  0, 1. / dep_j, -pts_camera_j(1) / (dep_j * dep_j), 0, 0, 0,
	  0, 0, 0, 1. / dep_j_r, 0, -pts_camera_j_r(0) / (dep_j_r * dep_j_r),
	  0, 0, 0, 0, 1. / dep_j_r, -pts_camera_j_r(1) / (dep_j_r * dep_j_r);
#endif
        reduce = sqrt_info * reduce;

        if (jacobians[0])
        {
            Eigen::Map<Eigen::Matrix<double, 4, 7, Eigen::RowMajor>> jacobian_pose_i(jacobians[0]);
	    //pts_left
            Eigen::Matrix<double, 6, 6> jaco_i;
            jaco_i.block<3,3>(0,0) = ric.transpose() * Rj.transpose();
            jaco_i.block<3,3>(0,3) = ric.transpose() * Rj.transpose() * Ri * -Utility::skewSymmetric(pts_imu_i);

	    // pts_right
            jaco_i.block<3,3>(3,0) = Rc21 * ric.transpose() * Rj.transpose(); //对Pi求导
            jaco_i.block<3,3>(3,3) = Rc21 * ric.transpose() * Rj.transpose() * Ri * -Utility::skewSymmetric(pts_imu_i); //对Qi求导

	    // 4x6 * 6x6
            jacobian_pose_i.leftCols<6>() = reduce * jaco_i;
            jacobian_pose_i.rightCols<1>().setZero();
        }

        if (jacobians[1])
        {
            Eigen::Map<Eigen::Matrix<double, 4, 7, Eigen::RowMajor>> jacobian_pose_j(jacobians[1]);
	    // residual of pt_left
            Eigen::Matrix<double, 6, 6> jaco_j;
            jaco_j.block<3,3>(0,0) = ric.transpose() * -Rj.transpose();
            jaco_j.block<3,3>(0,3) = ric.transpose() * Utility::skewSymmetric(pts_imu_j); // R.inv*vector 对R求导: [R.inv*vector]x

	    // residual of pt_right
	    jaco_j.block<3,3>(3,0) = Rc21 * ric.transpose() * -Rj.transpose();
            jaco_j.block<3,3>(3,3) = Rc21 * ric.transpose() * Utility::skewSymmetric(pts_imu_j); // R.inv*vector 对R求导: [R.inv*vector]x

            jacobian_pose_j.leftCols<6>() = reduce * jaco_j;
            jacobian_pose_j.rightCols<1>().setZero();
        }
        if (jacobians[2])
        {
            Eigen::Map<Eigen::Matrix<double, 4, 7, Eigen::RowMajor>> jacobian_ex_pose(jacobians[2]);
	    // residual of pt_left
            Eigen::Matrix<double, 6, 6> jaco_ex;
            jaco_ex.block<3,3>(0,0) = ric.transpose() * (Rj.transpose() * Ri - Eigen::Matrix3d::Identity());
            Eigen::Matrix3d tmp_r = ric.transpose() * Rj.transpose() * Ri * ric;
            jaco_ex.block<3,3>(0,3) =
	      -tmp_r * Utility::skewSymmetric(pts_camera_i) +
	      Utility::skewSymmetric(tmp_r * pts_camera_i) +
	      Utility::skewSymmetric(ric.transpose() * (Rj.transpose() * (Ri * tic + Pi - Pj) - tic));
	    // residual of pt_right
	    jaco_ex.block<3,3>(3,0) = Rc21 * ric.transpose() * (Rj.transpose() * Ri - Eigen::Matrix3d::Identity());
            jaco_ex.block<3,3>(3,3) =
	      Rc21 * -tmp_r * Utility::skewSymmetric(pts_camera_i) +
	      Rc21 * Utility::skewSymmetric(tmp_r * pts_camera_i) +
	      Rc21 * Utility::skewSymmetric(ric.transpose() * (Rj.transpose() * (Ri * tic + Pi - Pj) - tic));
	    
            jacobian_ex_pose.leftCols<6>() = reduce * jaco_ex;
            jacobian_ex_pose.rightCols<1>().setZero();
        }
        if (jacobians[3])
        {
            Eigen::Map<Eigen::Vector4d> jacobian_feature(jacobians[3]);
#if 0
            jacobian_feature = reduce * ric.transpose() * Rj.transpose() * Ri * ric * pts_i * -1.0 / (inv_dep_i * inv_dep_i);
#else
	    Eigen::Matrix<double, 6, 1> jaco_fe;
	    jaco_fe.block<3,1>(0,0) = ric.transpose() * Rj.transpose() * Ri * ric * pts_i * -1.0 / (inv_dep_i * inv_dep_i);
	    jaco_fe.block<3,1>(3,0) = Rc21 * ric.transpose() * Rj.transpose() * Ri * ric * pts_i * -1.0 / (inv_dep_i * inv_dep_i);
	    
            //jacobian_feature = reduce * ric.transpose() * Rj.transpose() * Ri * ric * pts_i;
	    jacobian_feature = reduce * jaco_fe;
#endif
        }
    }
    sum_t += tic_toc.toc();

    return true;
}

#if 0
void ProjectionFactor::check(double **parameters)
{
    double *res = new double[15];
    double **jaco = new double *[4];
    jaco[0] = new double[2 * 7];
    jaco[1] = new double[2 * 7];
    jaco[2] = new double[2 * 7];
    jaco[3] = new double[2 * 1];
    Evaluate(parameters, res, jaco);
    puts("check begins");

    puts("my");

    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 1>>(res).transpose() << std::endl
              << std::endl;
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[0]) << std::endl
              << std::endl;
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[1]) << std::endl
              << std::endl;
    std::cout << Eigen::Map<Eigen::Matrix<double, 2, 7, Eigen::RowMajor>>(jaco[2]) << std::endl
              << std::endl;
    std::cout << Eigen::Map<Eigen::Vector2d>(jaco[3]) << std::endl
              << std::endl;

    Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
    Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

    Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
    Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

    Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
    Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
    double inv_dep_i = parameters[3][0];

    Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
    Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
    Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
    Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
    Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);

    double dep_j = pts_camera_j.z();

    Eigen::Vector2d residual;
    residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();

    residual = sqrt_info * residual;

    puts("num");
    std::cout << residual.transpose() << std::endl;

    const double eps = 1e-6;
    Eigen::Matrix<double, 2, 19> num_jacobian;
    for (int k = 0; k < 19; k++)
    {
        Eigen::Vector3d Pi(parameters[0][0], parameters[0][1], parameters[0][2]);
        Eigen::Quaterniond Qi(parameters[0][6], parameters[0][3], parameters[0][4], parameters[0][5]);

        Eigen::Vector3d Pj(parameters[1][0], parameters[1][1], parameters[1][2]);
        Eigen::Quaterniond Qj(parameters[1][6], parameters[1][3], parameters[1][4], parameters[1][5]);

        Eigen::Vector3d tic(parameters[2][0], parameters[2][1], parameters[2][2]);
        Eigen::Quaterniond qic(parameters[2][6], parameters[2][3], parameters[2][4], parameters[2][5]);
        double inv_dep_i = parameters[3][0];

        int a = k / 3, b = k % 3;
        Eigen::Vector3d delta = Eigen::Vector3d(b == 0, b == 1, b == 2) * eps;

        if (a == 0)
            Pi += delta;
        else if (a == 1)
            Qi = Qi * Utility::deltaQ(delta);
        else if (a == 2)
            Pj += delta;
        else if (a == 3)
            Qj = Qj * Utility::deltaQ(delta);
        else if (a == 4)
            tic += delta;
        else if (a == 5)
            qic = qic * Utility::deltaQ(delta);
        else if (a == 6)
            inv_dep_i += delta.x();

        Eigen::Vector3d pts_camera_i = pts_i / inv_dep_i;
        Eigen::Vector3d pts_imu_i = qic * pts_camera_i + tic;
        Eigen::Vector3d pts_w = Qi * pts_imu_i + Pi;
        Eigen::Vector3d pts_imu_j = Qj.inverse() * (pts_w - Pj);
        Eigen::Vector3d pts_camera_j = qic.inverse() * (pts_imu_j - tic);

        double dep_j = pts_camera_j.z();

        Eigen::Vector2d tmp_residual;
        tmp_residual = (pts_camera_j / dep_j).head<2>() - pts_j.head<2>();
        tmp_residual = sqrt_info * tmp_residual;

        num_jacobian.col(k) = (tmp_residual - residual) / eps;
    }
    std::cout << num_jacobian << std::endl;
}
#endif
