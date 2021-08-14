#include "cuda_computer.cuh"
using clock_value_t = long long;

__device__ void cuda_sleep(clock_value_t sleep_cycles)
{
    clock_value_t start = clock64();
    clock_value_t cycles_elapsed;
    do { cycles_elapsed = clock64() - start; } 
    while (cycles_elapsed < sleep_cycles);
}

__global__ void mykernel(volatile int *data,volatile int * data2){

  unsigned long time;
//   for (int i = 0; i < INCS; i++){
//     atomicAdd((int *)data,1);
//     __threadfence_system();
//     time = clock64();
//     while((clock64() - time)<TIME_INC) {};
//     }
//   printf("progress check finished\n");
    while(1){
        printf("111111111111111111\n");
        while(1)   
        {
            if(data[0]==0)
            break;
        }
        printf("begin to cal\n");
        int a;
        //
        *data2=10;
        printf("begin to exit\n");
        atomicAdd((int *)data,1);
        printf("data: %d\n",data[0]);
        // while (1)
        // {
        //     data[0]==1;
        //     if(data[0]==1)
        //         break;
        // }
    }
}
void cuda_computer::kill_kernel(){
    // cudaDeviceReset();
    // run_flag_host[0] = TERMINATE;
    output_host[0].run_flag = TERMINATE;
    
}

__global__ void cuda_pena_compute( PenaltyParameter *param,
    volatile output_Gst *output_dev){
    __shared__ double gradT_cache[THREADSPERBLOCK];
    __shared__ double cost_cache[THREADSPERBLOCK];
    __shared__ double gradC_cache[THREADSPERBLOCK][18];
    while(1){
        // cuda_sleep(10);
        while(1)   
        {
            if(output_dev[0].run_flag==TERMINATE)
                return;
            if(output_dev[blockIdx.x].run_flag ==RUN_STATE)
                break;
        }
    //     clock_t t;
    //     t = clock();
    //     // printf("112 %d \n",int(t));
        // if(blockIdx.x==1&&threadIdx.x==1) 
        //     printf("11111111111111111111111111111111111111111\n");
        const int index = threadIdx.x + blockIdx.x * blockDim.x; 
        const int i = blockIdx.x;
        const int j = threadIdx.x;
        clock_t t_0,t_1,t_2,t_3,t_4,t_5,t_11,t_12,t_13,t_14,t_15;
        clock_t tau_0,tau_1,tau_2,tau_3,tau_4;
        clock_t test_0,test_1,test_2,test_3,test_4;
        t_0 = clock();
        const int cacheIndex = threadIdx.x;
        const double vMaxSqr = param->vMaxSqr;
        const double thrAccMinSqr = param->thrAccMinSqr;
        const double thrAccMaxSqr = param->thrAccMaxSqr;
        const double bdrMaxSqr = param->bdrMaxSqr;
        const double safeMargin = param->safeMargin;
        const int segs = param->segs;
        const double gAcc = param->gAcc;
        // if(i==0&&j==0)
        //     printf("thrmin %f thramx: %f bdrMaxSqr: %f \n",thrAccMinSqr,thrAccMaxSqr,bdrMaxSqr);
        // Eigen::Map<Eigen::VectorXi> cons(param->cons,segs); 
        int cons = param->cons[i];
        // Eigen::Map<Eigen::VectorXi> idxHs(param->idxHs,segs);
        Eigen::Map<Eigen::Vector3d> ellipsoid(param ->ellipsoid);
        Eigen::Map<Eigen::Vector4d> ci(param->ci);
        // Eigen::Map<Eigen::VectorXd> inner_T(param->T,segs);
        double inner_T = param->T[i];
        // Eigen::Map<Eigen::MatrixXd> b(param->b,segs*6,3);
        Eigen::Map<Eigen::Matrix<double,6,3>> c (param->b[i]);
        const int poly_num = param->polys[i];
        Eigen::Map<Eigen::MatrixXd> cfgHs(param->cfgHs[i],6,poly_num);
        Eigen::Map<Eigen::MatrixXd> temp_gradC(gradC_cache[cacheIndex],6,3);
        
        temp_gradC.setZero();
        double temp_gradT = 0;
        double temp_cost = 0;
        

        
        double s1,s2,s3,s4,s5;
        Eigen::Vector3d pos, vel, acc, jer, sna;
        double step, alpha;
        Eigen::Matrix<double, 6, 1> beta0, beta1, beta2, beta3, beta4;
        int K;
        Eigen::Matrix3d rotM;
        double signedDist, signedDistSqr, signedDistCub;
        double gradSignedDt;
        Eigen::Vector3d h, zB, czB, xC, yB, xB;
        Eigen::Matrix3d dnczB, dzB, cdzB, dyB, dxB;
        Eigen::Vector3d outerNormal, point;
        double eNorm;
        Eigen::Vector3d eNormGd;
        Eigen::Matrix3d gradSdTxyz;
        Eigen::Vector3d gradSdT;
        double outerNormaldVel;
        Eigen::Matrix<double, 6, 3> gradSdCx, gradSdCy, gradSdCz, gradSdC;
        Eigen::Matrix<double, 6, 3> beta2dOuterNormalTp, beta0dOuterNormalTp;
        double violaVel, violaThrl, violaThrh, violaBdr;
        double violaVelPenaD, violaThrlPenaD, violaThrhPenaD, violaBdrPenaD;
        double violaVelPena, violaThrlPena, violaThrhPena, violaBdrPena;
        Eigen::Matrix<double, 6, 3> gradViolaVc, gradViolaThrlc, gradViolaThrhc, gradViolaBdrc;
        double gradViolaVt, gradViolaThrlt, gradViolaThrht, gradViolaBdrt;
        double fThr, sqrMagThr, sqrMagBdr;
        Eigen::Vector3d dfThr, dSqrMagThr, bdr, xyBdr;
        Eigen::Vector3d dSqrMagBdr, rotTrDotJer;
        Eigen::Matrix3d dBdr, dxyBdr;
        Eigen::Vector3d dJerSqrMagBdr;
        Eigen::Matrix3d dJerBdr, dJerxyBdr;
        double omg;
        int innerLoop, idx;
        t_1  = clock();
        if(i<segs){
            step =  inner_T/cons;
            innerLoop = cons+1;
            // if(i==0&&j==0){
            //     printf("---------------------------------------------gpu\n");
            //     printf("%f   %f    %f   \n %f   %f    %f\n%f   %f    %f \n  %f   %f    %f\n%f   %f    %f\n   %f   %f    %f\n",
            //     c(0,0),c(0,1),c(0,2),
            //     c(1,0),c(1,1),c(1,2),
            //     c(2,0),c(2,1),c(2,2),
            //     c(3,0),c(3,1),c(3,2),
            //     c(4,0),c(4,1),c(4,2),
            //     c(5,0),c(5,1),c(5,2));
            // }
            if(j<innerLoop){
                s1 = step*j;
                s2 = s1*s1;
                s3 = s2*s1;
                s4 = s2 * s2;
                s5 = s4 * s1;
                beta0 << 1.0, s1, s2, s3, s4, s5;
                beta1 << 0.0, 1.0, 2.0 * s1, 3.0 * s2, 4.0 * s3, 5.0 * s4;
                beta2 << 0.0, 0.0, 2.0, 6.0 * s1, 12.0 * s2, 20.0 * s3;
                beta3 << 0.0, 0.0, 0.0, 6.0, 24.0 * s1, 60.0 * s2;
                beta4 << 0.0, 0.0, 0.0, 0.0, 24.0, 120.0 * s1;
                alpha = 1.0 / cons * j;
                pos = c.transpose() * beta0;
                vel = c.transpose() * beta1;
                acc = c.transpose() * beta2;
                jer = c.transpose() * beta3;
                sna = c.transpose() * beta4;

                tau_0 = clock();

                h = acc;
                h(2)+=param->gAcc;


                double a_1 = h(0),b_1 = h(1),c_1 = h(2);
                double aSqr = a_1 * a_1, bSqr = b_1 * b_1, cSqr = c_1 * c_1; 
                double ab = a_1 * b_1, bc = b_1 * c_1, ca = c_1 * a_1;
                double xSqrNorm = aSqr + bSqr + cSqr;
                double xNorm = sqrt(xSqrNorm);
                double den = xSqrNorm * xNorm;
                zB = h / xNorm;
                dzB(0, 0) = bSqr + cSqr;
                dzB(0, 1) = -ab;
                dzB(0, 2) = -ca;
                dzB(1, 0) = -ab;
                dzB(1, 1) = aSqr + cSqr;
                dzB(1, 2) = -bc;
                dzB(2, 0) = -ca;
                dzB(2, 1) = -bc;
                dzB(2, 2) = aSqr + bSqr;
                dzB /= den;

                tau_1 = clock();

                
                czB << 0.0, zB(2), -zB(1);
                cdzB << Eigen::RowVector3d::Zero(), dzB.row(2), -dzB.row(1);
                test_0 = clock();
                a_1 = czB(0);
                b_1 = czB(1);
                c_1 = czB(2);
                test_1 = clock();
                aSqr = a_1 * a_1; bSqr = b_1* b_1; cSqr = c_1 * c_1;
                ab = a_1 * b_1; bc = b_1 * c_1; ca = c_1 * a_1;
                xSqrNorm = aSqr + bSqr + cSqr;
                test_2 = clock();
                xNorm = sqrt(xSqrNorm);
                den = xSqrNorm*xNorm;
                yB = czB/xNorm;
                // yB = czB.normalized();
                test_3 = clock();
                

                dnczB(0, 0) = bSqr + cSqr;
                dnczB(0, 1) = -ab;
                dnczB(0, 2) = -ca;
                dnczB(1, 0) = -ab;
                dnczB(1, 1) = aSqr + cSqr;
                dnczB(1, 2) = -bc;
                dnczB(2, 0) = -ca;
                dnczB(2, 1) = -bc;
                dnczB(2, 2) = aSqr + bSqr;
                test_4 = clock();
                dnczB /= den;

                tau_2 = clock();

                xB = yB.cross(zB);
                dyB = dnczB * cdzB;
                dxB.col(0) = dyB.col(0).cross(zB) + yB.cross(dzB.col(0));
                dxB.col(1) = dyB.col(1).cross(zB) + yB.cross(dzB.col(1));
                dxB.col(2) = dyB.col(2).cross(zB) + yB.cross(dzB.col(2));
                rotM << xB, yB, zB;
                gradSdTxyz.col(0) = dxB * jer;
                gradSdTxyz.col(1) = dyB * jer;
                gradSdTxyz.col(2) = dzB * jer;

                tau_3 = clock();


                fThr = h.norm();
                dfThr = h / fThr;
                sqrMagThr = fThr * fThr;
                dSqrMagThr = 2 * h;
                rotTrDotJer = rotM.transpose() * jer;
                bdr = rotTrDotJer / fThr;
                xyBdr << -bdr(1), bdr(0), 0.0;
                sqrMagBdr = xyBdr.squaredNorm();
                dBdr = -rotTrDotJer * dfThr.transpose() / (fThr * fThr) -
                        rotM.transpose() * (dxB * rotTrDotJer(0) + dyB * rotTrDotJer(1) + dzB * rotTrDotJer(2)) / fThr;
                dxyBdr << -dBdr.row(1), dBdr.row(0), Eigen::RowVector3d::Zero();
                dSqrMagBdr = 2.0 * xyBdr.transpose() * dxyBdr;
                dJerBdr = rotM.transpose() / fThr;
                dJerxyBdr << -dJerBdr.row(1), dJerBdr.row(0), Eigen::RowVector3d::Zero();
                dJerSqrMagBdr = 2.0 * xyBdr.transpose() * dJerxyBdr;
                violaVel = vel.squaredNorm() - vMaxSqr;
                violaThrl = thrAccMinSqr - sqrMagThr;
                violaThrh = sqrMagThr - thrAccMaxSqr;
                violaBdr = sqrMagBdr - bdrMaxSqr;


                omg = (j == 0 || j == innerLoop - 1) ? 0.5 : 1.0;

                // idx = idxHs(i);
                K = cfgHs.cols();
                t_11 = clock();


                for (int k = 0; k < K; k++)
                {
                    outerNormal = cfgHs.col(k).head<3>();
                    point = cfgHs.col(k).tail<3>();
                    beta0dOuterNormalTp = beta0 * outerNormal.transpose();
                    gradSdT = gradSdTxyz.transpose() * outerNormal;
                    outerNormaldVel = outerNormal.dot(vel);
                    beta2dOuterNormalTp = beta2 * outerNormal.transpose();
                    gradSdCx = beta2dOuterNormalTp * dxB;
                    gradSdCy = beta2dOuterNormalTp * dyB;
                    gradSdCz = beta2dOuterNormalTp * dzB;

                    eNormGd = (rotM.transpose() * outerNormal).array() * ellipsoid.array();
                    eNorm = eNormGd.norm();
                    eNormGd /= eNorm;
                    signedDist = outerNormal.dot(pos - point) + eNorm;
                    eNormGd.array() *= ellipsoid.array();

                    signedDist += param->safeMargin;
                    if (signedDist > 0)
                    {
                        signedDistSqr = signedDist * signedDist;
                        signedDistCub = signedDist * signedDistSqr;
                        gradSdC = beta0dOuterNormalTp +
                                    gradSdCx * eNormGd(0) +
                                    gradSdCy * eNormGd(1) +
                                    gradSdCz * eNormGd(2);
                        gradSignedDt = alpha * (outerNormaldVel +
                                                gradSdT(0) * eNormGd(0) +
                                                gradSdT(1) * eNormGd(1) +
                                                gradSdT(2) * eNormGd(2));
                        
                        // temp_grad.gradC.block<6, 3>(i * 6, 0) += omg * step * ci(0) * 3.0 * signedDistSqr * gradSdC;
                        temp_gradC+= omg * step * ci(0) * 3.0 * signedDistSqr * gradSdC;
                        // temp += omg * ci(0) * (3.0 * signedDistSqr * gradSignedDt * step + signedDistCub / cons(i));
                        temp_gradT+= omg * ci(0) * (3.0 * signedDistSqr * gradSignedDt * step + signedDistCub / cons);
                        temp_cost += omg * step * ci(0) * signedDistCub;
                        //omg step 
                    }
                }


                t_12 = clock();
                if (violaVel > 0.0)
                {
                    violaVelPenaD = violaVel * violaVel;
                    violaVelPena = violaVelPenaD * violaVel;
                    violaVelPenaD *= 3.0;
                    gradViolaVc = 2.0 * beta1 * vel.transpose();
                    gradViolaVt = 2.0*alpha*vel.dot(acc);
                    temp_gradC += omg * step * ci(1) * violaVelPenaD * gradViolaVc;
                    temp_gradT += omg * (ci(1) * violaVelPenaD * gradViolaVt * step +
                                        ci(1) * violaVelPena / cons);
                    temp_cost += omg * step * ci(1) * violaVelPena;
                }
                t_13 = clock();
                if (violaThrl > 0.0)
                {
                    violaThrlPenaD = violaThrl * violaThrl;
                    violaThrlPena = violaThrlPenaD * violaThrl;
                    violaThrlPenaD *= 3.0;
                    gradViolaThrlc = -beta2 * dSqrMagThr.transpose();
                    gradViolaThrlt = -alpha * dSqrMagThr.dot(jer);
                    temp_gradC += omg * step * ci(2) * violaThrlPenaD * gradViolaThrlc;
                    temp_gradT += omg * (ci(2) * violaThrlPenaD * gradViolaThrlt * step +
                                        ci(2) * violaThrlPena / cons);
                    temp_cost += omg * step * ci(2) * violaThrlPena;
                }
                t_14 = clock();
                if (violaThrh > 0.0)
                {
                    violaThrhPenaD = violaThrh * violaThrh;
                    violaThrhPena = violaThrhPenaD * violaThrh;
                    violaThrhPenaD *= 3.0;
                    gradViolaThrhc = beta2 * dSqrMagThr.transpose();
                    gradViolaThrht = alpha * dSqrMagThr.dot(jer);
                    temp_gradC += omg * step * ci(2) * violaThrhPenaD * gradViolaThrhc;
                    temp_gradT += omg * (ci(2) * violaThrhPenaD * gradViolaThrht * step +
                                        ci(2) * violaThrhPena / cons);
                    temp_cost += omg * step * ci(2) * violaThrhPena;
                }
                t_15 = clock();
                if (violaBdr > 0.0)
                {
                    violaBdrPenaD = violaBdr * violaBdr;
                    violaBdrPena = violaBdrPenaD * violaBdr;
                    violaBdrPenaD *= 3.0;
                    gradViolaBdrc = beta2 * dSqrMagBdr.transpose() + beta3 * dJerSqrMagBdr.transpose();
                    gradViolaBdrt = alpha * (dSqrMagBdr.dot(jer) + dJerSqrMagBdr.dot(sna));
                    temp_gradC += omg * step * ci(3) * violaBdrPenaD * gradViolaBdrc;
                    temp_gradT += omg * (ci(3) * violaBdrPenaD * gradViolaBdrt * step +
                                        ci(3) * violaBdrPena / cons);
                    temp_cost += omg * step * ci(3) * violaBdrPena;
                }
            }
        }
        t_2 = clock();
        // grad_cache[cacheIndex].cost = temp_cost;
        cost_cache[cacheIndex] = temp_cost;
        gradT_cache[cacheIndex] = temp_gradT;
        __syncthreads();
        t_3 = clock();
        int tmpindex = blockDim.x/2;//      
        while(tmpindex!=0){
            if(cacheIndex<tmpindex){
                gradT_cache[cacheIndex]+=gradT_cache[cacheIndex+tmpindex];
                cost_cache[cacheIndex]+=cost_cache[cacheIndex+tmpindex];
                for(int idx = 0;idx<18;idx++){
                    gradC_cache[cacheIndex][idx]+=gradC_cache[cacheIndex+tmpindex][idx];
                }
            }
            __syncthreads();
            tmpindex/=2;
        }
        t_4 = clock();
        if(cacheIndex==0){
            output_dev[blockIdx.x].gradT =gradT_cache[0];
            // for(int p =0;p<6;p++)
            //     for(int p1 = 0;p1<3;p1++){
            //         pgrad_devptr[blockIdx.x].gradC[p][p1] = grad_cache[0].gradC[p][p1];
            //     }
            // pgrad_devptr[blockIdx.x].cost = grad_cache[0].cost;  
            output_dev[blockIdx.x].cost =  cost_cache[0];
            for(int p = 0;p<18;p++)
                output_dev[blockIdx.x].gradC[p] = gradC_cache[0][p];
        }
        t_5 = clock();
        
       
        
       if(threadIdx.x==0) {
    //    __threadfence();
            __threadfence_system();
    //         atomicAdd((int *)(&output_dev[0].run_flag),1);
        output_dev[blockIdx.x].run_flag = FINISHED_STATE;

       }

       __syncthreads();

       
    //    if(blockIdx.x==1&&threadIdx.x==1) printf("run flag %d\n",run_flag[0]);
    }
}
void cuda_computer::setup(const int pieceNum){
    cudaSetDevice(0);
    cudaSetDeviceFlags (cudaDeviceMapHost);
    /*
    cudaHostAlloc((void **)&h_data, sizeof(int), cudaHostAllocMapped);
    cudaHostAlloc((void **)&h_data2, sizeof(int), cudaHostAllocMapped);
    */
    cudaError err;
    err =cudaHostAlloc((void **)&pena_param_host, sizeof(PenaltyParameter), cudaHostAllocMapped); 
    if(err != cudaSuccess) 
        throw CudaException("DeviceLinear: unable to allocate linear memory.", err);
    // err =  cudaHostAlloc((void **)&grad_block_host, sizeof(pgradpt)*pieceNum, cudaHostAllocMapped);
    // if(err != cudaSuccess) 
    //     throw CudaException("DeviceLinear: unable to allocate linear memory.", err);
    // err = cudaHostAlloc((void **)&cost_block_host, sizeof(double)*pieceNum, cudaHostAllocMapped);
    // if(err != cudaSuccess) 
    //     throw CudaException("DeviceLinear: unable to allocate linear memory.", err);
    // err = cudaHostAlloc((void **)&run_flag_host, sizeof(int), cudaHostAllocMapped);
    // if(err != cudaSuccess) 
    //     throw CudaException("DeviceLinear: unable to allocate linear memory.", err);
    err =  cudaHostAlloc((void **)&output_host, sizeof(output_Gst)*pieceNum, cudaHostAllocMapped);
    if(err != cudaSuccess) 
        throw CudaException("DeviceLinear: unable to allocate linear memory.", err);
    
    err = cudaHostGetDevicePointer((PenaltyParameter **)&pena_param_dev, (PenaltyParameter *)pena_param_host, 0);
    if(err != cudaSuccess) 
        throw CudaException("DeviceLinear: unable to allocate linear memory.", err);
    err = cudaHostGetDevicePointer((output_Gst **)&output_dev, (output_Gst *)output_host, 0);
    if(err != cudaSuccess) 
        throw CudaException("DeviceLinear: unable to allocate linear memory.", err);
    // err = cudaHostGetDevicePointer((pgradpt **)&grad_block_dev, (pgradpt *)grad_block_host, 0);
    // if(err != cudaSuccess) 
    //     throw CudaException("DeviceLinear: unable to allocate linear memory.", err);
    // err = cudaHostGetDevicePointer((double **)&cost_block_dev, (double *)cost_block_host, 0);
    // if(err != cudaSuccess) 
    //     throw CudaException("DeviceLinear: unable to allocate linear memory.", err);
    // err = cudaHostGetDevicePointer((int **)&run_flag_dev, (int *)run_flag_host, 0);
    // if(err != cudaSuccess) 
    //     throw CudaException("DeviceLinear: unable to allocate linear memory.", err);
    segs = pieceNum;
    // output_host[0].run_flag = pieceNum;
    for(int i = 0;i<pieceNum;i++)
        output_host[i].run_flag = FINISHED_STATE;
    dim3 blocks;
    dim3 threads;
    blocks.x = pieceNum;
    blocks.y=1;
    blocks.z= 1;
    threads.x = THREADSPERBLOCK;
    threads.y  =1;
    threads.z = 1;
    cuda_pena_compute<<<blocks,threads>>>(pena_param_dev,output_dev);

}


void cuda_computer::compute(const Eigen::VectorXi cons,
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
                                  Eigen::MatrixXd& gdC,
                                  int pieceNum,
                                  Eigen::VectorXd T1,
                                  Eigen::MatrixXd b){
        clock_t t1,t2,t3,t4,t5;
        t1 = clock();
        double vMaxSqr = vMax * vMax;
        double thrAccMinSqr = thrAccMin * thrAccMin;
        double thrAccMaxSqr = thrAccMax * thrAccMax;
        double bdrMaxSqr = bdrMax * bdrMax;
        pena_param_host[0].gAcc = gAcc;
        pena_param_host[0].safeMargin = safeMargin;
        pena_param_host[0].thrAccMaxSqr = thrAccMaxSqr;
        pena_param_host[0].thrAccMinSqr = thrAccMinSqr;
        pena_param_host[0].vMaxSqr = vMaxSqr;
        pena_param_host[0].bdrMaxSqr = bdrMaxSqr;
        pena_param_host[0].segs = pieceNum;
        
        for(int k = 0;k<3;k++)
            pena_param_host[0].ellipsoid[k] = ellipsoid[k];
        for(int k = 0;k<4;k++)
            pena_param_host[0].ci[k] = ci[k];
        
        for(int k = 0;k<pieceNum;k++){
            pena_param_host[0].cons[k] = cons[k];
            pena_param_host[0].idxHs[k] = idxHs[k];
            pena_param_host[0].T[k] = T1[k];
        }
        int count = 0;
        for(int p = 0;p<pieceNum;p++){
            count=0;
            for(int l = 0;l < 3;l++){
                for(int k = 0;k<6;k++){
                    pena_param_host[0].b[p][count++] = b.block<6,3>(p*6,0)(k,l);//列讀入
                }
            }
        }
        count  = 0;

        for(int i = 0;i<pieceNum;i++){
            pena_param_host[0].polys[i] = cfgHs[i].cols();
            count = 0;
            for(int k = 0;k<cfgHs[i].cols();k++)
                for(int j = 0;j<6;j++)
                    pena_param_host[0].cfgHs[i][count++] = cfgHs[i](j,k);//列讀入
        }
        t2 = clock();
        // output_host[0].run_flag = RUN_STATE;
        for(int i = 0;i<pieceNum;i++){
            output_host[i].run_flag = RUN_STATE;
        }
        // printf("begin to run \n");
        // ROS_INFO_STREAM("begin to run!");
        int value;
        int finished_flag;
        while(1){
            finished_flag = true; 
            for(int i = 0;i<pieceNum;i++){
                if(output_host[i].run_flag!=FINISHED_STATE){
                    finished_flag = false;
                    break;
                }
            }
            if(finished_flag)
                break;
        }
        t3=clock();
        for(int i = 0;i<pieceNum;i++){
            // printf("cost %f \n",cost_block_host[i]);
            cost+=output_host[i].cost;
            gdT(i)+=output_host[i].gradT;
            Eigen::Matrix<double,6,3> tmp_gradC;
            for(int col = 0;col<3;col++)
                for(int row = 0;row<6;row++)
                    tmp_gradC(row,col) = output_host[i].gradC[col*6+row];
            // Eigen::Map<Eigen::Matrix<double,6,3>> tmp_gradC(grad_block_host[i].gradC);
            gdC.block<6,3>(i*6,0)+=tmp_gradC;
        }
        t4 =clock();
        // ROS_INFO_STREAM("t1: "<<double(t2-t1)/CLOCKS_PER_SEC<<" t2: "<<double(t3-t2)/CLOCKS_PER_SEC<<" t3: "<<double(t4-t3)/CLOCKS_PER_SEC);
       
}


cuda_computer::~cuda_computer(){
    // delete[] grad_block_host;
    // delete[] cost_block_host;
    // free(grad_block_host);
    // free(cost_block_host);

    cudaFreeHost((void*)pena_param_host);
    cudaFreeHost((void*)output_host);
    // cudaFreeHost((void*)cost_block_host);
    // cudaFreeHost((void*)grad_block_host);
    // cudaFreeHost((void*)run_flag_host);
    //grad_block_host
    //run_flag_host
}