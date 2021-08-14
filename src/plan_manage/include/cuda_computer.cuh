

#ifndef CUDA_COMPUTER_CUH_
#define CUDA_COMPUTER_CUH_

#include <sstream>
#include <cuda_runtime.h>
#include <Eigen/Eigen>
#include <iostream>
#include <cmath>
#include <cfloat>
#include <vector>
#include <ctime>
#include <ros/console.h>
#include "ros/ros.h"
#include <stdio.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>
#include <stdio.h>
#include <cuda.h>
#include <sys/time.h>
#define THREADSPERBLOCK 64//hzc need to modify
#define MAXSEGS 100
#define MAXPOLYS 50
#define TIME_INC 100000000
#define INCS 10
#define USE_PROGRESS 1
#define MAT_DIMX 4000
#define MAT_DIMY MAT_DIMX
#define RUN_STATE 0
#define FINISHED_STATE 1
#define TERMINATE 99999

#define cudaCheckErrors(msg) \
    do { \
        cudaError_t __err = cudaGetLastError(); \
        if (__err != cudaSuccess) { \
            fprintf(stderr, "Fatal error: %s (%s at %s:%d)\n", \
                msg, cudaGetErrorString(__err), \
                __FILE__, __LINE__); \
            fprintf(stderr, "*** FAILED - ABORTING\n"); \
            exit(1); \
        } \
    } while (0)
  
struct CudaException : public std::exception
{
  CudaException(const std::string& what, cudaError err)
    : what_(what), err_(err) {}
  virtual ~CudaException() throw() {}
  virtual const char* what() const throw()
  {
    std::stringstream description;
    description << "CudaException: " << what_ << std::endl;
    if(err_ != cudaSuccess)
    {
      description << "cudaError code: " << cudaGetErrorString(err_);
      description << " (" << err_ << ")" << std::endl;
    }
    return description.str().c_str();
  }
  std::string what_;
  cudaError err_;
};

struct PenaltyParameter
{
    // ~PenaltyParameter(){delete []cfgHs;};
    PenaltyParameter(){};
    int cons[MAXSEGS];//n
    int idxHs[MAXSEGS];//n
    double cfgHs[MAXSEGS][6*MAXPOLYS];//(2n-1)*6*m
    int polys[MAXSEGS];
    double ellipsoid[3];//3*1
    double safeMargin;
    double vMaxSqr;
    double thrAccMinSqr;
    double thrAccMaxSqr;
    double bdrMaxSqr;
    double gAcc;
    double ci[4];	//4*1
    int segs;
    double T[MAXSEGS];//n*1
    double b[MAXSEGS][18];//6N*3
};
struct pgradpt{
    double gradT;//n*1
    double gradC[6*3];//6n*3
    // double cost;
};
struct  output_Gst{
  double gradT;//n*1
  double gradC[6*3];//6n*3
  double cost;
  int run_flag;
};
__global__ void cuda_pena_compute(PenaltyParameter *param, pgradpt * pgrad_devptr,double* cost_devptr);

class cuda_computer{
  public:
    cuda_computer(){};
    ~cuda_computer();
    /*
    volatile int *d_data, *h_data;
    volatile int *d_data2,*h_data2;
    */
    PenaltyParameter* pena_param_host;
    PenaltyParameter* pena_param_dev;                                                                                                                                    
    // volatile pgradpt* grad_block_dev;
    // volatile pgradpt* grad_block_host;
    // volatile double* cost_block_dev;
    // volatile double* cost_block_host;
    // volatile int* run_flag_host;
    // volatile int* run_flag_dev;
    volatile output_Gst* output_host;
    volatile output_Gst* output_dev;

    int segs;
    void setup(const int pieceNum);
    void compute(const Eigen::VectorXi cons,
                                  const Eigen::VectorXi &idxHs,
                                  const std::vector<Eigen::MatrixXd> &cfgHs,
                                  const Eigen::Vector3d &ellipsoid,
                                  const double safeMargin,
                                  const double vMax,
                                  const double thrAccMin,
                                  const double thrAccMax,
                                  const double bdrMax,
                                  const double gAcc,
                                  const Eigen::Vector4d ci,
                                  double &cost,
                                  Eigen::VectorXd& gdT,
                                  Eigen::MatrixXd&gdC,
                                  int pieceNum,
                                   Eigen::VectorXd T1,
                                  Eigen::MatrixXd b);
    void kill_kernel();
    
};

#endif /* RMD_CUDA_EXCEPTION_CUH_ */
