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

#pragma once

#include "typedefs.h"
#include <Eigen/Core>
#include <atomic>
#include <cdd/setoper.h> // Must be included before cdd.h (wtf)
#include <cdd/cdd.h>
#include <mutex>
#include <utility>

namespace Eigen {

  using HrepXd = std::pair<Eigen::MatrixXd, Eigen::VectorXd>;
  using VrepXd = std::pair<Eigen::MatrixXd, Eigen::VectorXd>;

/* Wrapper of Convex Polyhedron
 * This class aims to translate eigen matrix into cddlib matrix.
 * It automatically transforms a v-polyhedron into an h-polyhedron and vice-versa.
 */
class Polyhedron {
public:
    /* Default constructor that set cdd global constants. */
    Polyhedron();
    /* Free the pointers and unset the cdd global constants. */
    ~Polyhedron();

    /* Treat the inputs as an H-representation and compute its V-representation.
     * An H-polyhedron is such that \f$ Ax \leq b \f$.
     * \param A Matrix part of the H-representation.
     * \param b Vector part of the H-representation.
     * \return true if the conversion was successful.
     */
    bool setHrep(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);
    /* Treat the inputs as a V-representation and compute its H-representation.
     * V-polyhedron is such that \f$ A = [v r]^T, b=[1^T 0^T]^T \f$
     * with A composed of \f$ v \f$, the vertices, \f$ r \f$, the rays,
     * and b is a corresponding vector with 1 for vertices and 0 for rays.
     * \param A Matrix part of the V-representation.
     * \param b Vector part of the V-representation.
     * \return true if the conversion was successful.
     */
    bool setVrep(const Eigen::MatrixXd& A, const Eigen::VectorXd& b);
    /* Get the V-representation of the polyhedron.
     * V-polyhedron is such that \f$ A = [v r]^T, b=[1^T 0^T]^T \f$
     * with A composed of \f$ v \f$, the vertices, \f$ r \f$, the rays,
     * and b is a corresponding vector with 1 for vertices and 0 for rays.
     * \return Pair of vertex-ray matrix and vector of the V-representation.
     */
    VrepXd vrep() const;
    /* Get the H-representation of the polyhedron.
     * H-polyhedron is such that \f$ Ax \leq b \f$.
     * \return Pair of inequality matrix and inequality vector of the H-representation.
     */
    HrepXd hrep() const;

    /* Print the H-representation of the polyhedron */
    void printHrep() const;
    /* Print the V-representation of the polyhedron */
    void printVrep() const;

    /* Set the polyhedron from a matrix \f$ R = [r]^T$ of stacked ray vectors.
     * \param R m x n matrix of stacked rays (each ray has dimension n).
     * \return true if the conversion was successful.
     */
    bool setRays(const Eigen::MatrixXd& R);
    /* Set the polyhedron from a matrix \f$ V = [v]^T$ of stacked vertices.
     * \param V m x n matrix of stacked vertices (each vertex has dimension n).
     * \return true if the conversion was successful.
     */
    bool setVertices(const Eigen::MatrixXd& V);
    /* Get a readable error message for the last conversion error.
     * \return String for last error message.
     */
    std::string lastErrorMessage();

private:
    bool hvrep(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, bool isFromGenerators);
    void initializeMatrixPtr(Eigen::Index rows, Eigen::Index cols, bool isFromGenerators);
    bool doubleDescription(const Eigen::MatrixXd& matrix, bool isFromGenerators);
    Eigen::MatrixXd concatenateMatrix(const Eigen::MatrixXd& A, const Eigen::VectorXd& b, bool isFromGenerators);
    std::pair<Eigen::MatrixXd, Eigen::VectorXd> ddfMatrix2EigenMatrix(const dd_MatrixPtr mat, bool isOuputVRep) const;

private:
    dd_MatrixPtr matPtr_;
    dd_PolyhedraPtr polytope_;
    dd_ErrorType err_;

private:
    static std::atomic_int counter;
    static std::mutex mtx;
};

} // namespace Eigen
