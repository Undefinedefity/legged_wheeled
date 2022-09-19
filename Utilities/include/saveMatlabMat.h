#ifndef _SAVEMATLABMAT_H_
#define _SAVEMATLABMAT_H_

#include <string.h>
#include <iostream>
#include <stdio.h>
#include <mat.h>
#include <eigen3/Eigen/Core>

using namespace Eigen;

using namespace std;

template <typename T>
bool SaveMatlabMat(T *src, string savePath, string matrixName, int cols, int rows)
{
    // tanspose befoee being saved
    int datasize = cols * rows;
    double *Final = new double[datasize]; //convert to double precision
    memset(Final, 0, datasize * sizeof(double));
    for (int i = 0; i < rows; i++)
    {
        for (int j = 0; j < cols; j++)
        {
            Final[j * rows + i] = double(src[i * cols + j]);
        }
    }
    mxArray *pWriteArray = NULL; //matlab mat form
    MATFile *pmatFile = NULL;    //.mat file
    pmatFile = matOpen(savePath.c_str(), "u");
    if (pmatFile == nullptr)
        pmatFile = matOpen(savePath.c_str(), "w");
    if (pmatFile == nullptr)
    {
        printf("mat save path is error");
        return false;
    }
    pWriteArray = mxCreateDoubleMatrix(rows, cols, mxREAL);
    memcpy((void *)(mxGetPr(pWriteArray)), (void *)Final, sizeof(double) * datasize);
    matPutVariable(pmatFile, matrixName.c_str(), pWriteArray);

    matClose(pmatFile);          //close file
    mxDestroyArray(pWriteArray); //release resource
    delete[] Final;              //release resource
    Final = nullptr;
    return true;
}

template <typename Derived>
bool saveEigenMatInMatlabMat(const MatrixBase<Derived> &src, string savePath, string matrixName)
{
    // tanspose befoee being saved
    int datasize = src.cols() * src.rows();
    double *Final = new double[datasize]; //convert to double precision
    memset(Final, 0, datasize * sizeof(double));
    for (int i = 0; i < src.rows(); i++)
    {
        for (int j = 0; j < src.cols(); j++)
        {
            Final[j * src.rows() + i] = double(src(i, j));
        }
    }
    mxArray *pWriteArray = NULL; //matlab mat form
    MATFile *pmatFile = NULL;    //.mat file
    pmatFile = matOpen(savePath.c_str(), "u");
    if (pmatFile == nullptr)
        pmatFile = matOpen(savePath.c_str(), "w");
    if (pmatFile == nullptr)
    {
        printf("mat save path is error");
        return false;
    }
    pWriteArray = mxCreateDoubleMatrix(src.rows(), src.cols(), mxREAL);
    memcpy((void *)(mxGetPr(pWriteArray)), (void *)Final, sizeof(double) * datasize);
    matPutVariable(pmatFile, matrixName.c_str(), pWriteArray);

    matClose(pmatFile);          //close file
    mxDestroyArray(pWriteArray); //release resource
    delete[] Final;              //release resource
    Final = nullptr;
    return true;
}

#endif