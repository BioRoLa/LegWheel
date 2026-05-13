#include <math.h>

/*
 * Evaluate 12-control-point 3D Bezier point.
 * cp layout: [x0,y0,z0, x1,y1,z1, ... x11,y11,z11]
 * coeff layout: 12 binomial coefficients.
 */
void bezier_point_3d_12(
    const double *cp,
    const double *coeff,
    double t,
    double offset_x,
    double offset_y,
    double offset_z,
    double *out
) {
    const int n = 11;
    const double one_minus_t = 1.0 - t;

    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    for (int i = 0; i < 12; ++i) {
        const int p1 = n - i;
        const int p2 = i;
        const double basis = coeff[i] * pow(one_minus_t, p1) * pow(t, p2);
        x += basis * cp[i * 3 + 0];
        y += basis * cp[i * 3 + 1];
        z += basis * cp[i * 3 + 2];
    }

    out[0] = x + offset_x;
    out[1] = y + offset_y;
    out[2] = z + offset_z;
}

static void skew(const double w[3], double out[9]) {
    out[0] = 0.0;    out[1] = -w[2]; out[2] = w[1];
    out[3] = w[2];   out[4] = 0.0;   out[5] = -w[0];
    out[6] = -w[1];  out[7] = w[0];  out[8] = 0.0;
}

static double norm3(const double a[3]) {
    return sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
}

/*
 * Compute T = exp([omega, v] * theta), row-major 4x4 output.
 */
void screw_exp6(const double *omega_in, const double *v_in, double theta, double *out16) {
    double omega[3] = {omega_in[0], omega_in[1], omega_in[2]};
    double v[3] = {v_in[0], v_in[1], v_in[2]};

    const double wnorm = norm3(omega);

    for (int i = 0; i < 16; ++i) {
        out16[i] = 0.0;
    }
    out16[15] = 1.0;

    if (wnorm < 1e-9) {
        out16[0] = 1.0;
        out16[5] = 1.0;
        out16[10] = 1.0;
        out16[3] = v[0] * theta;
        out16[7] = v[1] * theta;
        out16[11] = v[2] * theta;
        return;
    }

    double w[9];
    double w2[9];
    skew(omega, w);

    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            double s = 0.0;
            for (int k = 0; k < 3; ++k) {
                s += w[r * 3 + k] * w[k * 3 + c];
            }
            w2[r * 3 + c] = s;
        }
    }

    const double st = sin(theta);
    const double ct = cos(theta);

    double R[9];
    double V[9];
    for (int r = 0; r < 3; ++r) {
        for (int c = 0; c < 3; ++c) {
            const double I = (r == c) ? 1.0 : 0.0;
            R[r * 3 + c] = I + st * w[r * 3 + c] + (1.0 - ct) * w2[r * 3 + c];
            V[r * 3 + c] = I * theta + (1.0 - ct) * w[r * 3 + c] + (theta - st) * w2[r * 3 + c];
        }
    }

    double p[3];
    for (int r = 0; r < 3; ++r) {
        p[r] = V[r * 3 + 0] * v[0] + V[r * 3 + 1] * v[1] + V[r * 3 + 2] * v[2];
    }

    out16[0] = R[0]; out16[1] = R[1]; out16[2] = R[2]; out16[3] = p[0];
    out16[4] = R[3]; out16[5] = R[4]; out16[6] = R[5]; out16[7] = p[1];
    out16[8] = R[6]; out16[9] = R[7]; out16[10] = R[8]; out16[11] = p[2];
}
