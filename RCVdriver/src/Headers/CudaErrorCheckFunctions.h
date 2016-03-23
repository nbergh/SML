#ifndef CUDAERRORCHECKFUNCTIONS_H_
#define CUDAERRORCHECKFUNCTIONS_H_

#include <iostream>

namespace CUDA_ERROR_CHECK_FUNCTIONS {
	static void CheckCudaErrorAux (const char *file, unsigned line, const char *statement, cudaError_t err) {
		if (err == cudaSuccess)
			return;
		std::cerr << statement<<" returned " << cudaGetErrorString(err) << "("<<err<< ") at "<<file<<":"<<line << std::endl;
		exit (1);
	}

	#define CUDA_CHECK_RETURN(value) CheckCudaErrorAux(__FILE__,__LINE__, #value, value)
}


#endif /* CUDAERRORCHECKFUNCTIONS_H_ */
