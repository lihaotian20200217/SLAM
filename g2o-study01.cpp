void EdgeProjectXYZ2UV::linearizeOplus() {
    VertexSE3Expmap* vj = static_cast<VertexSE3Expmap*>(_vertices[1]);
    SE3Quat T(vj->estimate());
    VertexSBAPointXYZ* vi = static_cast<VertexSBAPointXYZ*>(_vertices[0]);
    Vector3D xyz = vi->estimate();
    Vector3D xyz_trans = T.map(xyz);

    double x = xyz_trans[0];
    double y = xyz_trans[1];
    double z = xyz_trans[2];
    double z_2 = z * z;

    const CameraParameters* cam = static_cast<const CameraParameters*>(parameter(0));

    Matrix<double,2,3,Eigen::ColMajor> tmp;
    tmp(0,0) = cam->focal_length;
    tmp(0,1) = 0;
    tmp(0,2) = -x/z*cam->focal_length;

    tmp(1,0) = 0;
    tmp(1,1) = cam->focal_length;
    tmp(1,2) = -y/z*cam->focal_length;

    _jacobianOplusXi = -1./z * tmp * T.rotation().toRotationMatrix();

    _jacobianOplusXj(0,0) = x*y/z_2*cam->focal_length;
    _jacobianOplusXj(0,1) = -(1+(x*x/z_2)) * cam->focal_length;
    _jacobianOplusXj(0,2) = y/z * cam->focal_length;
    _jacobianOplusXj(0,3) = -1./z*cam->focal_length;
    _jacobianOplusXj(0,4) = 0;
    _jacobianOplusXj(0,5) = x/z_2*cam->focal_length;

    _jacobianOplusXj(1,0) = (1+y*y/z_2)*cam->focal_length;
    _jacobianOplusXj(1,1) = -x*y/z_2 * cam->focal_length;
    _jacobianOplusXj(1,2) = -x/z*cam->focal_length;
    _jacobianOplusXj(1,3) = 0;
    _jacobianOplusXj(1,4) = -1./z*cam->focal_length;
    _jacobianOplusXj(1,5) = y/z_2*cam->focal_length;
}






































}