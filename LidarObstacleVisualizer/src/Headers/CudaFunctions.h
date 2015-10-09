/* This header declares the two entry functions (for GPU calculations) used by the CPU
 *
 */

#ifndef CUDAFUNCTIONS_H_
#define CUDAFUNCTIONS_H_

#include "StructDefinitions.h"

void translateLidarDataFromRawToXYZ(MemoryPointers* memoryPointers);

#endif /* CUDAFUNCTIONS_H_ */
