// This file is part of eigen-cddlib.

// eigen-cddlib is free software: you can redistribute it and/or
// modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// eigen-cddlib is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
//
// You should have received a copy of the GNU Lesser General Public License
// along with eigen-cddlib.  If not, see
// <http://www.gnu.org/licenses/>.

#include "Polyhedron.h"
#include <iostream>
#include <fstream>

namespace Eigen {

std::atomic_int Polyhedron::counter(0);
std::mutex Polyhedron::mtx;

Polyhedron::Polyhedron()
    : matPtr_(nullptr)
    , polytope_(nullptr)
{
    if (counter == 0)
        dd_set_global_constants();
    counter++;
}

Polyhedron::~Polyhedron()
{
    counter--;

    if (matPtr_ != nullptr)
        dd_FreeMatrix(matPtr_);
    if (polytope_ != nullptr)
        dd_FreePolyhedra(polytope_);

    if (counter == 0)
        dd_free_global_constants();
}

bool Polyhedron::setHrep(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
{
    std::unique_lock<std::mutex> lock(mtx);
    return hvrep(A, b, false);
}

bool Polyhedron::setVrep(const Eigen::MatrixXd& A, const Eigen::VectorXd& b)
{
    std::unique_lock<std::mutex> lock(mtx);
    return hvrep(A, b, true);
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> Polyhedron::vrep() const
{
    std::unique_lock<std::mutex> lock(mtx);
    dd_MatrixPtr mat = dd_CopyGenerators(polytope_);
    return ddfMatrix2EigenMatrix(mat, true);
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> Polyhedron::hrep() const
{
    std::unique_lock<std::mutex> lock(mtx);
    dd_MatrixPtr mat = dd_CopyInequalities(polytope_);
    return ddfMatrix2EigenMatrix(mat, false);
}

void Polyhedron::printVrep() const
{
    std::unique_lock<std::mutex> lock(mtx);
    dd_MatrixPtr mat = dd_CopyGenerators(polytope_);
    dd_WriteMatrix(stdout, mat);
}

void Polyhedron::printHrep() const
{
    std::unique_lock<std::mutex> lock(mtx);
    dd_MatrixPtr mat = dd_CopyInequalities(polytope_);
    dd_WriteMatrix(stdout, mat);
}

bool Polyhedron::setRays(const Eigen::MatrixXd& R)
{
    return setVrep(R, Eigen::VectorXd::Zero(R.rows()));
}

bool Polyhedron::setVertices(const Eigen::MatrixXd& V)
{
    return setVrep(V, Eigen::VectorXd::Ones(V.rows()));
}

/**
 * Private functions
 */

bool Polyhedron::hvrep(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, bool isFromGenerators)
{
    Eigen::MatrixXd cMat = concatenateMatrix(A, b, isFromGenerators);
    return doubleDescription(cMat, isFromGenerators);
}

void Polyhedron::initializeMatrixPtr(Eigen::Index rows, Eigen::Index cols, bool isFromGenerators)
{
    if (matPtr_ != nullptr)
        dd_FreeMatrix(matPtr_);
    
    matPtr_ = dd_CreateMatrix(rows, cols);
    matPtr_->representation = (isFromGenerators ? dd_Generator : dd_Inequality);
}

bool Polyhedron::doubleDescription(const Eigen::MatrixXd& matrix, bool isFromGenerators)
{
    initializeMatrixPtr(matrix.rows(), matrix.cols(), isFromGenerators);

    //std::cout<<matrix<<std::endl;

    for (auto row = 0; row < matrix.rows(); ++row)
        for (auto col = 0; col < matrix.cols(); ++col)
            matPtr_->matrix[row][col][0] = matrix(row, col);

    if (polytope_ != nullptr)
        dd_FreePolyhedra(polytope_);

    polytope_ = dd_DDMatrix2Poly(matPtr_, &err_);
    return (err_ == dd_NoError) ? true : false;
}

Eigen::MatrixXd Polyhedron::concatenateMatrix(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, bool isFromGenerators)
{
    double sign = (isFromGenerators ? 1 : -1);
    Eigen::MatrixXd mat(A.rows(), A.cols() + 1);
    mat.col(0) = b;
    mat.block(0, 1, A.rows(), A.cols()).noalias() = sign * A;
    return mat;
}

std::pair<Eigen::MatrixXd, Eigen::VectorXd> Polyhedron::ddfMatrix2EigenMatrix(const dd_MatrixPtr mat, bool isOuputVRep) const
{
    double sign = (isOuputVRep ? 1 : -1);
    auto rows = mat->rowsize;
    auto cols = mat->colsize;
    Eigen::MatrixXd mOut(rows, cols - 1);
    Eigen::VectorXd vOut(rows);
    for (auto row = 0; row < rows; ++row) {
        vOut(row) = mat->matrix[row][0][0];
        for (auto col = 1; col < cols; ++col)
            mOut(row, col - 1) = sign * mat->matrix[row][col][0];
    }

    dd_FreeMatrix(mat);    
    return std::make_pair(mOut, vOut);
}

std::string Polyhedron::lastErrorMessage()
{
    FILE * tmpFile = tmpfile();
    if (fseek(tmpFile, 0, SEEK_SET) != 0)
    {
        return "Cannot get error message: unable to create temporary binary file";
    }
    dd_WriteErrorMessages(tmpFile, err_);
    fseek(tmpFile, 0, SEEK_END);
    long length = ftell(tmpFile);
    fseek(tmpFile, 0, SEEK_SET);
    char * buffer = new char[length];
    size_t nChar = fread(buffer, sizeof(char), length, tmpFile);
    std::string errorMessage(buffer, nChar);
    delete[] buffer;
    fclose(tmpFile);
    return errorMessage;
}

} // namespace Eigen
