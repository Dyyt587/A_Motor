
#include <utils.h>

static const int one_by_sqrt3 = 577;
static const int two_by_sqrt3 = 1155;

int SVM(int alpha, int beta, int* tA, int* tB, int* tC) {
    int Sextant;

    if (beta >= 0) {
        if (alpha >= 0) {
            //quadrant I
            if (one_by_sqrt3 * beta > alpha*1000)
                Sextant = 2; //sextant v2-v3
            else
                Sextant = 1; //sextant v1-v2

        } else {
            //quadrant II
            if (-one_by_sqrt3 * beta > alpha*1000)
                Sextant = 3; //sextant v3-v4
            else
                Sextant = 2; //sextant v2-v3
        }
    } else {
        if (alpha >= 0.0f) {
            //quadrant IV
            if (-one_by_sqrt3 * beta > alpha*1000)
                Sextant = 5; //sextant v5-v6
            else
                Sextant = 6; //sextant v6-v1
        } else {
            //quadrant III
            if (one_by_sqrt3 * beta > alpha*1000)
                Sextant = 4; //sextant v4-v5
            else
                Sextant = 5; //sextant v5-v6
        }
    }

    switch (Sextant) {
        // sextant v1-v2
        case 1: {
            // Vector on-times
            int t1 = (alpha*1000 - one_by_sqrt3 * beta)/1000;
            int t2 = two_by_sqrt3 * beta/1000;

            // PWM timings
            *tA = (1000 - t1 - t2) / 2;
            *tB = *tA + t1;
            *tC = *tB + t2;

            break;
        }
        // sextant v2-v3
        case 2: {
            // Vector on-times
            int t2 = (alpha*1000 + one_by_sqrt3 * beta)/1000;
            int t3 = (-alpha*1000 + one_by_sqrt3 * beta)/1000;

            // PWM timings
            *tB = (1000 - t2 - t3) / 2;
            *tA = *tB + t3;
            *tC = *tA + t2;

            break;
        }
        // sextant v3-v4
        case 3: {
            // Vector on-times
            int t3 = two_by_sqrt3 * beta/1000;
            int t4 = (-alpha*1000 - one_by_sqrt3 * beta)/1000;

            // PWM timings
            *tB = (1000 - t3 - t4) /2;
            *tC = *tB + t3;
            *tA = *tC + t4;

            break;
        }
        // sextant v4-v5
        case 4: {
            // Vector on-times
            int t4 = (-alpha*1000 + one_by_sqrt3 * beta)/1000;
            int t5 = -two_by_sqrt3 * beta/1000;

            // PWM timings
            *tC = (1000 - t4 - t5) /2;
            *tB = *tC + t5;
            *tA = *tB + t4;

            break;
        }
        // sextant v5-v6
        case 5: {
            // Vector on-times
            int t5 = (-alpha*1000 - one_by_sqrt3 * beta)/1000;
            int t6 = (alpha*1000 - one_by_sqrt3 * beta)/1000;

            // PWM timings
            *tC = (1000 - t5 - t6) /2;
            *tA = *tC + t5;
            *tB = *tA + t6;

            break;
        }
        // sextant v6-v1
        case 6: {
            // Vector on-times
            int t6 = -two_by_sqrt3 * beta/1000;
            int t1 = (alpha*1000 + one_by_sqrt3 * beta)/1000;

            // PWM timings
            *tA = (1000 - t6 - t1) /2;
            *tC = *tA + t1;
            *tB = *tC + t6;

            break;
        }
    }

    int retval = 0;
    if (
           *tA < 0
        || *tA > 1000
        || *tB < 0
        || *tB > 1000
        || *tC < 0
        || *tC > 1000
    ) retval = -1;
    return retval;
}

int64_t get_time_us(void) {
    return get_system_us();
}

int64_t get_time_ms(void) {
    return get_system_ms();
}