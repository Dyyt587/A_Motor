
#ifndef __UTILS_H
#define __UTILS_H

// Compute rising edge timings (0.0 - 1.0) as a function of alpha-beta
// as per the magnitude invariant clarke transform
// The magnitude of the alpha-beta vector may not be larger than sqrt(3)/2
// Returns 0 on success, and -1 if the input was out of range
int SVM(int alpha, int beta, int* tA, int* tB, int* tC);

#endif //__UTILS_H
