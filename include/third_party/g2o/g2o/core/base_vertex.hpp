//
// Created by yc_qian on 19-12-20.
//

template <int D, typename T>
BaseVertex<D, T>::BaseVertex() :
        OptimizableGraph::Vertex(),
        _hessian(0, D, D)
{
    _dimension = D;
}

template <int D, typename T>
double BaseVertex<D, T>::solveDirect(double lambda) {
    Matrix <double, D, D> tempA=_hessian + Matrix <double, D, D>::Identity()*lambda;
    double det=tempA.determinant();
    if (g2o_isnan(det) || det < std::numeric_limits<double>::epsilon())
        return det;
    Matrix <double, D, 1> dx=tempA.llt().solve(_b);
    oplus(&dx[0]);
    return det;
}

template <int D, typename T>
void BaseVertex<D, T>::clearQuadraticForm() {
    _b.setZero();
}

template <int D, typename T>
void BaseVertex<D, T>::mapHessianMemory(double* d)
{
    new (&_hessian) HessianBlockType(d, D, D);
}