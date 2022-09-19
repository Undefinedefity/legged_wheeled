#ifndef _ROBOTMOTIONCTRL_SAVEMAT_H_
#define _ROBOTMOTIONCTRL_SAVEMAT_H_
#include <iostream>
#include "matio.h"
#include "cppTypes.h"

template <typename T>
bool savemat(const DMat<T> &var, std::string file_name, std::string var_name)
{
    size_t dims[2];
    dims[0] = var.rows();
    dims[1] = var.cols();
    Eigen::MatrixXd _var = var.template cast<double>();

    mat_t *mat;
    mat = Mat_CreateVer(file_name.c_str(), NULL, MAT_FT_DEFAULT);
    if (!mat)
    {
        return false;
    }
    matvar_t *matvar;
    matvar = Mat_VarCreate(var_name.c_str(), MAT_C_DOUBLE, MAT_T_DOUBLE, 2, dims, _var.data(), 0);
    Mat_VarWrite(mat, matvar, MAT_COMPRESSION_NONE);
    Mat_VarFree(matvar);
    Mat_Close(mat);
    return true;
}

#endif