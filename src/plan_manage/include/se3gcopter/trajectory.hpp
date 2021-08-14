/*
    MIT License

    Copyright (c) 2021 Zhepei Wang (wangzhepei@live.com)

    Permission is hereby granted, free of charge, to any person obtaining a copy
    of this software and associated documentation files (the "Software"), to deal
    in the Software without restriction, including without limitation the rights
    to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
    copies of the Software, and to permit persons to whom the Software is
    furnished to do so, subject to the following conditions:

    The above copyright notice and this permission notice shall be included in all
    copies or substantial portions of the Software.

    THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
    IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
    FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
    AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
    LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
    OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
    SOFTWARE.
*/

#ifndef TRAJECTORY_HPP
#define TRAJECTORY_HPP

#include "root_finder.hpp"
#include "lbfgs.hpp"
#include "geoutils.hpp"
#include <Eigen/Eigen>
#include <iostream>
#include <cmath>
#include <cfloat>
#include <vector>
#include <ctime>


typedef Eigen::Matrix<double, 3, 6> CoefficientMat;
typedef Eigen::Matrix<double, 3, 5> VelCoefficientMat;
typedef Eigen::Matrix<double, 3, 4> AccCoefficientMat;

class Piece
{
private:
    double duration;
    CoefficientMat coeffMat;

public:
    Piece() = default;

    Piece(double dur, const CoefficientMat &cMat)
        : duration(dur), coeffMat(cMat) {}

    inline int getDim() const
    {
        return 3;
    }

    inline int getOrder() const
    {
        return 5;
    }

    inline double getDuration() const
    {
        return duration;
    }

    inline const CoefficientMat &getCoeffMat() const
    {
        return coeffMat;
    }

    inline Eigen::Vector3d getPos(const double &t) const
    {
        Eigen::Vector3d pos(0.0, 0.0, 0.0);
        double tn = 1.0;
        for (int i = 5; i >= 0; i--)
        {
            pos += tn * coeffMat.col(i);
            tn *= t;
        }
        return pos;
    }

    inline Eigen::Vector3d getVel(const double &t) const
    {
        Eigen::Vector3d vel(0.0, 0.0, 0.0);
        double tn = 1.0;
        int n = 1;
        for (int i = 4; i >= 0; i--)
        {
            vel += n * tn * coeffMat.col(i);
            tn *= t;
            n++;
        }
        return vel;
    }

    inline Eigen::Vector3d getAcc(const double &t) const
    {
        Eigen::Vector3d acc(0.0, 0.0, 0.0);
        double tn = 1.0;
        int m = 1;
        int n = 2;
        for (int i = 3; i >= 0; i--)
        {
            acc += m * n * tn * coeffMat.col(i);
            tn *= t;
            m++;
            n++;
        }
        return acc;
    }

    inline Eigen::Vector3d getJer(const double &t) const
    {
        Eigen::Vector3d jer(0.0, 0.0, 0.0);
        double tn = 1.0;
        int l = 1;
        int m = 2;
        int n = 3;
        for (int i = 2; i >= 0; i--)
        {
            jer += l * m * n * tn * coeffMat.col(i);
            tn *= t;
            l++;
            m++;
            n++;
        }
        return jer;
    }

    inline CoefficientMat normalizePosCoeffMat() const
    {
        CoefficientMat nPosCoeffsMat;
        double t = 1.0;
        for (int i = 5; i >= 0; i--)
        {
            nPosCoeffsMat.col(i) = coeffMat.col(i) * t;
            t *= duration;
        }
        return nPosCoeffsMat;
    }

    inline VelCoefficientMat normalizeVelCoeffMat() const
    {
        VelCoefficientMat nVelCoeffMat;
        int n = 1;
        double t = duration;
        for (int i = 4; i >= 0; i--)
        {
            nVelCoeffMat.col(i) = n * coeffMat.col(i) * t;
            t *= duration;
            n++;
        }
        return nVelCoeffMat;
    }

    inline AccCoefficientMat normalizeAccCoeffMat() const
    {
        AccCoefficientMat nAccCoeffMat;
        int n = 2;
        int m = 1;
        double t = duration * duration;
        for (int i = 3; i >= 0; i--)
        {
            nAccCoeffMat.col(i) = n * m * coeffMat.col(i) * t;
            n++;
            m++;
            t *= duration;
        }
        return nAccCoeffMat;
    }

    inline double getMaxVelRate() const
    {
        Eigen::MatrixXd nVelCoeffMat = normalizeVelCoeffMat();
        Eigen::VectorXd coeff = RootFinder::polySqr(nVelCoeffMat.row(0)) +
                                RootFinder::polySqr(nVelCoeffMat.row(1)) +
                                RootFinder::polySqr(nVelCoeffMat.row(2));
        int N = coeff.size();
        int n = N - 1;
        for (int i = 0; i < N; i++)
        {
            coeff(i) *= n;
            n--;
        }
        if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON)
        {
            return 0.0;
        }
        else
        {
            double l = -0.0625;
            double r = 1.0625;
            while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON)
            {
                l = 0.5 * l;
            }
            while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON)
            {
                r = 0.5 * (r + 1.0);
            }
            std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(N - 1), l, r,
                                                                      FLT_EPSILON / duration);
            candidates.insert(0.0);
            candidates.insert(1.0);
            double maxVelRateSqr = -INFINITY;
            double tempNormSqr;
            for (std::set<double>::const_iterator it = candidates.begin();
                 it != candidates.end();
                 it++)
            {
                if (0.0 <= *it && 1.0 >= *it)
                {
                    tempNormSqr = getVel((*it) * duration).squaredNorm();
                    maxVelRateSqr = maxVelRateSqr < tempNormSqr ? tempNormSqr : maxVelRateSqr;
                }
            }
            return sqrt(maxVelRateSqr);
        }
    }

    inline double getMaxAccRate() const
    {
        Eigen::MatrixXd nAccCoeffMat = normalizeAccCoeffMat();
        Eigen::VectorXd coeff = RootFinder::polySqr(nAccCoeffMat.row(0)) +
                                RootFinder::polySqr(nAccCoeffMat.row(1)) +
                                RootFinder::polySqr(nAccCoeffMat.row(2));
        int N = coeff.size();
        int n = N - 1;
        for (int i = 0; i < N; i++)
        {
            coeff(i) *= n;
            n--;
        }
        if (coeff.head(N - 1).squaredNorm() < DBL_EPSILON)
        {
            return 0.0;
        }
        else
        {
            double l = -0.0625;
            double r = 1.0625;
            while (fabs(RootFinder::polyVal(coeff.head(N - 1), l)) < DBL_EPSILON)
            {
                l = 0.5 * l;
            }
            while (fabs(RootFinder::polyVal(coeff.head(N - 1), r)) < DBL_EPSILON)
            {
                r = 0.5 * (r + 1.0);
            }
            std::set<double> candidates = RootFinder::solvePolynomial(coeff.head(N - 1), l, r,
                                                                      FLT_EPSILON / duration);
            candidates.insert(0.0);
            candidates.insert(1.0);
            double maxAccRateSqr = -INFINITY;
            double tempNormSqr;
            for (std::set<double>::const_iterator it = candidates.begin();
                 it != candidates.end();
                 it++)
            {
                if (0.0 <= *it && 1.0 >= *it)
                {
                    tempNormSqr = getAcc((*it) * duration).squaredNorm();
                    maxAccRateSqr = maxAccRateSqr < tempNormSqr ? tempNormSqr : maxAccRateSqr;
                }
            }
            return sqrt(maxAccRateSqr);
        }
    }

    inline bool checkMaxVelRate(const double &maxVelRate) const
    {
        double sqrMaxVelRate = maxVelRate * maxVelRate;
        if (getVel(0.0).squaredNorm() >= sqrMaxVelRate ||
            getVel(duration).squaredNorm() >= sqrMaxVelRate)
        {
            return false;
        }
        else
        {
            Eigen::MatrixXd nVelCoeffMat = normalizeVelCoeffMat();
            Eigen::VectorXd coeff = RootFinder::polySqr(nVelCoeffMat.row(0)) +
                                    RootFinder::polySqr(nVelCoeffMat.row(1)) +
                                    RootFinder::polySqr(nVelCoeffMat.row(2));
            double t2 = duration * duration;
            coeff.tail<1>()(0) -= sqrMaxVelRate * t2;
            return RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
        }
    }

    inline bool checkMaxAccRate(const double &maxAccRate) const
    {
        double sqrMaxAccRate = maxAccRate * maxAccRate;
        if (getAcc(0.0).squaredNorm() >= sqrMaxAccRate ||
            getAcc(duration).squaredNorm() >= sqrMaxAccRate)
        {
            return false;
        }
        else
        {
            Eigen::MatrixXd nAccCoeffMat = normalizeAccCoeffMat();
            Eigen::VectorXd coeff = RootFinder::polySqr(nAccCoeffMat.row(0)) +
                                    RootFinder::polySqr(nAccCoeffMat.row(1)) +
                                    RootFinder::polySqr(nAccCoeffMat.row(2));
            double t2 = duration * duration;
            double t4 = t2 * t2;
            coeff.tail<1>()(0) -= sqrMaxAccRate * t4;
            return RootFinder::countRoots(coeff, 0.0, 1.0) == 0;
        }
    }
};

class Trajectory
{
private:
    typedef std::vector<Piece> Pieces;
    Pieces pieces;

public:
    Trajectory() = default;

    Trajectory(const std::vector<double> &durs,
               const std::vector<CoefficientMat> &cMats)
    {
        int N = std::min(durs.size(), cMats.size());
        pieces.reserve(N);
        for (int i = 0; i < N; i++)
        {
            pieces.emplace_back(durs[i], cMats[i]);
        }
    }

    inline int getPieceNum() const
    {
        return pieces.size();
    }

    inline Eigen::VectorXd getDurations() const
    {
        int N = getPieceNum();
        Eigen::VectorXd durations(N);
        for (int i = 0; i < N; i++)
        {
            durations(i) = pieces[i].getDuration();
        }
        return durations;
    }

    inline double getTotalDuration() const
    {
        int N = getPieceNum();
        double totalDuration = 0.0;
        for (int i = 0; i < N; i++)
        {
            totalDuration += pieces[i].getDuration();
        }
        return totalDuration;
    }

    inline Eigen::MatrixXd getPositions() const
    {
        int N = getPieceNum();
        Eigen::MatrixXd positions(3, N + 1);
        for (int i = 0; i < N; i++)
        {
            positions.col(i) = pieces[i].getCoeffMat().col(5);
        }
        positions.col(N) = pieces[N - 1].getPos(pieces[N - 1].getDuration());
        return positions;
    }

    inline const Piece &operator[](int i) const
    {
        return pieces[i];
    }

    inline Piece &operator[](int i)
    {
        return pieces[i];
    }

    inline void clear(void)
    {
        pieces.clear();
        return;
    }

    inline Pieces::const_iterator begin() const
    {
        return pieces.begin();
    }

    inline Pieces::const_iterator end() const
    {
        return pieces.end();
    }

    inline Pieces::iterator begin()
    {
        return pieces.begin();
    }

    inline Pieces::iterator end()
    {
        return pieces.end();
    }

    inline void reserve(const int &n)
    {
        pieces.reserve(n);
        return;
    }

    inline void emplace_back(const Piece &piece)
    {
        pieces.emplace_back(piece);
        return;
    }

    inline void emplace_back(const double &dur,
                             const CoefficientMat &cMat)
    {
        pieces.emplace_back(dur, cMat);
        return;
    }

    inline void append(const Trajectory &traj)
    {
        pieces.insert(pieces.end(), traj.begin(), traj.end());
        return;
    }

    inline int locatePieceIdx(double &t) const
    {
        int N = getPieceNum();
        int idx;
        double dur;
        for (idx = 0;
             idx < N &&
             t > (dur = pieces[idx].getDuration());
             idx++)
        {
            t -= dur;
        }
        if (idx == N)
        {
            idx--;
            t += pieces[idx].getDuration();
        }
        return idx;
    }

    inline Eigen::Vector3d getPos(double t) const
    {
        int pieceIdx = locatePieceIdx(t);
        return pieces[pieceIdx].getPos(t);
    }

    inline Eigen::Vector3d getVel(double t) const
    {
        int pieceIdx = locatePieceIdx(t);
        return pieces[pieceIdx].getVel(t);
    }

    inline Eigen::Vector3d getAcc(double t) const
    {
        int pieceIdx = locatePieceIdx(t);
        return pieces[pieceIdx].getAcc(t);
    }

    inline Eigen::Vector3d getJer(double t) const
    {
        int pieceIdx = locatePieceIdx(t);
        return pieces[pieceIdx].getJer(t);
    }

    inline Eigen::Vector3d getJuncPos(int juncIdx) const
    {
        if (juncIdx != getPieceNum())
        {
            return pieces[juncIdx].getCoeffMat().col(5);
        }
        else
        {
            return pieces[juncIdx - 1].getPos(pieces[juncIdx - 1].getDuration());
        }
    }

    inline Eigen::Vector3d getJuncVel(int juncIdx) const
    {
        if (juncIdx != getPieceNum())
        {
            return pieces[juncIdx].getCoeffMat().col(4);
        }
        else
        {
            return pieces[juncIdx - 1].getVel(pieces[juncIdx - 1].getDuration());
        }
    }

    inline Eigen::Vector3d getJuncAcc(int juncIdx) const
    {
        if (juncIdx != getPieceNum())
        {
            return pieces[juncIdx].getCoeffMat().col(3) * 2.0;
        }
        else
        {
            return pieces[juncIdx - 1].getAcc(pieces[juncIdx - 1].getDuration());
        }
    }

    inline double getMaxVelRate() const
    {
        int N = getPieceNum();
        double maxVelRate = -INFINITY;
        double tempNorm;
        for (int i = 0; i < N; i++)
        {
            tempNorm = pieces[i].getMaxVelRate();
            maxVelRate = maxVelRate < tempNorm ? tempNorm : maxVelRate;
        }
        return maxVelRate;
    }

    inline double getMaxAccRate() const
    {
        int N = getPieceNum();
        double maxAccRate = -INFINITY;
        double tempNorm;
        for (int i = 0; i < N; i++)
        {
            tempNorm = pieces[i].getMaxAccRate();
            maxAccRate = maxAccRate < tempNorm ? tempNorm : maxAccRate;
        }
        return maxAccRate;
    }

    inline bool checkMaxVelRate(const double &maxVelRate) const
    {
        int N = getPieceNum();
        bool feasible = true;
        for (int i = 0; i < N && feasible; i++)
        {
            feasible = feasible && pieces[i].checkMaxVelRate(maxVelRate);
        }
        return feasible;
    }

    inline bool checkMaxAccRate(const double &maxAccRate) const
    {
        int N = getPieceNum();
        bool feasible = true;
        for (int i = 0; i < N && feasible; i++)
        {
            feasible = feasible && pieces[i].checkMaxAccRate(maxAccRate);
        }
        return feasible;
    }

    inline void getRotation(const double &t,
                            const double &yaw,
                            const double &gAcc,
                            Eigen::Matrix3d &rotM) const
    {
        rotM.col(2) = getAcc(t);
        rotM(2, 2) += gAcc;
        rotM.col(2).normalize();
        rotM.col(1) = rotM.col(2).cross(Eigen::Vector3d(cos(yaw), sin(yaw), 0.0));
        rotM.col(1).normalize();
        rotM.col(0) = rotM.col(1).cross(rotM.col(2));
        return;
    }

    inline Eigen::Vector3d getTiltRate(const double &t,
                                       const double &gAcc) const
    {
        Eigen::Matrix3d rotM;
        rotM.col(2) = getAcc(t);
        rotM(2, 2) += gAcc;
        double thrAcc = rotM.col(2).norm();
        rotM.col(2) /= thrAcc;
        rotM.col(1) = rotM.col(2).cross(Eigen::Vector3d::UnitX());
        rotM.col(1).normalize();
        rotM.col(0) = rotM.col(1).cross(rotM.col(2));
        Eigen::Vector3d bdr = rotM.transpose() * getJer(t) / thrAcc;
        return Eigen::Vector3d(-bdr(1), bdr(0), 0.0);
    }
};
// The banded system class is used for solving
// banded linear system Ax=b efficiently.
// A is an N*N band matrix with lower band width lowerBw
// and upper band width upperBw.
// Banded LU factorization has O(N) time complexity.
class BandedSystem
{
public:
    // The size of A, as well as the lower/upper
    // banded width p/q are needed
    inline void create(const int &n, const int &p, const int &q)
    {
        // In case of re-creating before destroying
        destroy();
        N = n;
        lowerBw = p;
        upperBw = q;
        int actualSize = N * (lowerBw + upperBw + 1);
        ptrData = new double[actualSize];
        std::fill_n(ptrData, actualSize, 0.0);
        return;
    }

    inline void destroy()
    {
        if (ptrData != nullptr)
        {
            delete[] ptrData;
            ptrData = nullptr;
        }
        return;
    }

private:
    int N;
    int lowerBw;
    int upperBw;
    // Compulsory nullptr initialization here
    double *ptrData = nullptr;

public:
    // Reset the matrix to zero
    inline void reset(void)
    {
        std::fill_n(ptrData, N * (lowerBw + upperBw + 1), 0.0);
        return;
    }

    // The band matrix is stored as suggested in "Matrix Computation"
    inline const double &operator()(const int &i, const int &j) const
    {
        return ptrData[(i - j + upperBw) * N + j];
    }

    inline double &operator()(const int &i, const int &j)
    {
        return ptrData[(i - j + upperBw) * N + j];
    }

    // This function conducts banded LU factorization in place
    // Note that NO PIVOT is applied on the matrix "A" for efficiency!!!
    inline void factorizeLU()
    {
        int iM, jM;
        double cVl;
        for (int k = 0; k <= N - 2; k++)
        {
            iM = std::min(k + lowerBw, N - 1);
            cVl = operator()(k, k);
            for (int i = k + 1; i <= iM; i++)
            {
                if (operator()(i, k) != 0.0)
                {
                    operator()(i, k) /= cVl;
                }
            }
            jM = std::min(k + upperBw, N - 1);
            for (int j = k + 1; j <= jM; j++)
            {
                cVl = operator()(k, j);
                if (cVl != 0.0)
                {
                    for (int i = k + 1; i <= iM; i++)
                    {
                        if (operator()(i, k) != 0.0)
                        {
                            operator()(i, j) -= operator()(i, k) * cVl;
                        }
                    }
                }
            }
        }
        return;
    }

    // This function solves Ax=b, then stores x in b
    // The input b is required to be N*m, i.e.,
    // m vectors to be solved.
    inline void solve(Eigen::MatrixXd &b) const
    {
        int iM;
        for (int j = 0; j <= N - 1; j++)
        {
            iM = std::min(j + lowerBw, N - 1);
            for (int i = j + 1; i <= iM; i++)
            {
                if (operator()(i, j) != 0.0)
                {
                    b.row(i) -= operator()(i, j) * b.row(j);
                }
            }
        }
        for (int j = N - 1; j >= 0; j--)
        {
            b.row(j) /= operator()(j, j);
            iM = std::max(0, j - upperBw);
            for (int i = iM; i <= j - 1; i++)
            {
                if (operator()(i, j) != 0.0)
                {
                    b.row(i) -= operator()(i, j) * b.row(j);
                }
            }
        }
        return;
    }

    // This function solves ATx=b, then stores x in b
    // The input b is required to be N*m, i.e.,
    // m vectors to be solved.
    inline void solveAdj(Eigen::MatrixXd &b) const
    {
        int iM;
        for (int j = 0; j <= N - 1; j++)
        {
            b.row(j) /= operator()(j, j);
            iM = std::min(j + upperBw, N - 1);
            for (int i = j + 1; i <= iM; i++)
            {
                if (operator()(j, i) != 0.0)
                {
                    b.row(i) -= operator()(j, i) * b.row(j);
                }
            }
        }
        for (int j = N - 1; j >= 0; j--)
        {
            iM = std::max(0, j - lowerBw);
            for (int i = iM; i <= j - 1; i++)
            {
                if (operator()(j, i) != 0.0)
                {
                    b.row(i) -= operator()(j, i) * b.row(j);
                }
            }
        }
        return;
    }
};

#endif
