#include <math.h>

static double det3(const double m[9]) {
    return m[0] * (m[4] * m[8] - m[5] * m[7])
         - m[1] * (m[3] * m[8] - m[5] * m[6])
         + m[2] * (m[3] * m[7] - m[4] * m[6]);
}

static int inv3(const double m[9], double out[9]) {
    const double d = det3(m);
    if (fabs(d) < 1e-14) {
        return -1;
    }

    const double inv_d = 1.0 / d;

    out[0] = (m[4] * m[8] - m[5] * m[7]) * inv_d;
    out[1] = (m[2] * m[7] - m[1] * m[8]) * inv_d;
    out[2] = (m[1] * m[5] - m[2] * m[4]) * inv_d;

    out[3] = (m[5] * m[6] - m[3] * m[8]) * inv_d;
    out[4] = (m[0] * m[8] - m[2] * m[6]) * inv_d;
    out[5] = (m[2] * m[3] - m[0] * m[5]) * inv_d;

    out[6] = (m[3] * m[7] - m[4] * m[6]) * inv_d;
    out[7] = (m[1] * m[6] - m[0] * m[7]) * inv_d;
    out[8] = (m[0] * m[4] - m[1] * m[3]) * inv_d;
    return 0;
}

/*
 * Compute q_dot = J^T * inv(J*J^T + lambda^2 I) * v for 3x3 matrices/vectors.
 * Inputs are row-major arrays.
 */
int dls_solve_3x3(const double *J, const double *v, double damping, double *q_dot) {
    double A[9];
    double invA[9];
    double y[3];

    const double l2 = damping * damping;

    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            double sum = 0.0;
            for (int k = 0; k < 3; ++k) {
                sum += J[r * 3 + k] * J[c * 3 + k];
            }
            if (r == c) {
                sum += l2;
            }
            A[r * 3 + c] = sum;
        }
    }

    if (inv3(A, invA) != 0) {
        return -1;
    }

    for (int r = 0; r < 3; ++r) {
        y[r] = invA[r * 3 + 0] * v[0] + invA[r * 3 + 1] * v[1] + invA[r * 3 + 2] * v[2];
    }

    for (int c = 0; c < 3; ++c) {
        q_dot[c] = J[0 * 3 + c] * y[0] + J[1 * 3 + c] * y[1] + J[2 * 3 + c] * y[2];
    }

    return 0;
}
