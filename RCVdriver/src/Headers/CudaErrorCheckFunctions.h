#ifndef CUDAERRORCHECKFUNCTIONS_H_
#define CUDAERRORCHECKFUNCTIONS_H_

void CheckCudaErrorAux (const char *, unsigned, const char *, cudaError_t); //TODO make static
#define CUDA_CHECK_RETURN(value) CheckCudaErrorAux(__FILE__,__LINE__, #value, value)

#endif /* CUDAERRORCHECKFUNCTIONS_H_ */
