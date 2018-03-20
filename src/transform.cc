

#include "explore_large_map/transform.h"
#include <iostream>
#include <iomanip>

namespace cartographer {
    namespace transform {

        void geographic_to_tm_sphere(double R, double k0, double lon_mer, double FN, double FE, double lat_rad,
                                     double lon_rad, double *N, double *E) {
            double Rk0 = R * k0;
            double B = cos(lat_rad) * sin(lon_rad - lon_mer);
            *E = FE + Rk0 * (1.0 / 2) * log((1 + B) / (1 - B));//atanh(B);
            *N = FN + Rk0 * atan(tan(lat_rad) / cos(lon_rad - lon_mer));
        }

        void tm_to_geographic_sphere(double R, double k0, double lon_mer, double FN, double FE, double N, double E,
                                     double *lat_rad, double *lon_rad) {
            double Rk0 = R * k0;
            double D = (N - FN) / Rk0;
            *lon_rad = lon_mer + atan(sinh((E - FE) / Rk0) / cos(D));
            *lat_rad = asin(sin(D) / cosh((E - FE) / Rk0));
        }

        void geographic_to_ps_sphere(double R, double k0, Hemisphere hemi, double FN, double FE, double lat_rad,
                                     double lon_rad, double *N, double *E) {
            double Rk0 = R * k0;
            if (hemi == HEMI_NORTH) {
                *E = FE + 2 * Rk0 * tan(M_PI / 4 - lat_rad / 2) * sin(lon_rad);
                *N = FN - 2 * Rk0 * tan(M_PI / 4 - lat_rad / 2) * cos(lon_rad);
            } else if (hemi == HEMI_SOUTH) {
                *E = FE + 2 * Rk0 * tan(M_PI / 4 + lat_rad / 2) * sin(lon_rad);
                *N = FN + 2 * Rk0 * tan(M_PI / 4 + lat_rad / 2) * cos(lon_rad);
            }
        }

        void ps_to_geographic_sphere(double R, double k0, Hemisphere hemi, double FN, double FE, double N, double E,
                                     double *lat_rad, double *lon_rad) {
            double Rk0 = R * k0;
            double x = E - FE;
            double y = N - FN;
            double rho = sqrt(x * x + y * y);
            double c = 2 * atan(rho / (2 * Rk0));
            if (hemi == HEMI_NORTH) {
                *lat_rad = asin(cos(c));
                *lon_rad = atan2(x, -y);
            } else if (hemi == HEMI_SOUTH) {
                *lat_rad = -asin(cos(c));
                *lon_rad = atan2(x, y);
            }
        }

/* 
   The approximate series expansion equations for an ellipsoid are from 
   "The Universal Grids", Defense Mapping Agency Technical Manual 
   (DMATM) 8358.2
*/

        void geographic_to_tm(double a, double e2, double k0, double lon_mer, double FN, double FE, double lat_rad,
                              double lon_rad, double *N, double *E) {
            double ep2 = e2 / (1 - e2);
            double f = 1 - sqrt(1 - e2);
            double n = f / (2 - f);

            double phi = lat_rad;

            double s = sin(phi);
            double c = cos(phi);

            double s2 = s * s;
            double c2 = c * c;

            double nu = a / sqrt(1 - e2 * s2);

            double n2 = n * n;
            double n3 = n2 * n;
            double n4 = n3 * n;
            double n5 = n4 * n;

            double Ap = a * (1 - n + 5 * (n2 - n3) / 4 + 81 * (n4 - n5) / 64);
            double Bp = 3 * a * (n - n2 + 7 * (n3 - n4) / 8 + 55 * n5 / 64) / 2;
            double Cp = 15 * a * (n2 - n3 + 3 * (n4 - n5) / 4) / 16;
            double Dp = 35 * a * (n3 - n4 + 11 * n5 / 16) / 48;
            double Ep = 315 * a * (n4 - n5) / 512;

#if 0
            double s2phi = sin(2*phi);
            double s4phi = sin(4*phi);
            double s6phi = sin(6*phi);
            double s8phi = sin(8*phi);
#else
            double s2phi = 2.0 * s * c;
            double c2phi = c2 - s2;
            double s4phi = 2.0 * s2phi * c2phi;
            double c4phi = c2phi * c2phi - s2phi * s2phi;
            double s6phi = s4phi * c2phi + s2phi * c4phi;
            double s8phi = 2.0 * s4phi * c4phi;
#endif

            double S = Ap * phi - Bp * s2phi + Cp * s4phi - Dp * s6phi + Ep * s8phi;

            double sc = s * c;
            double nuck0 = nu * c * k0;
            double nusck0 = nu * sc * k0;

            double c4 = c2 * c2;
            double c6 = c4 * c2;

            double t = s / c;
            double t2 = t * t;
            double t4 = t2 * t2;
            double t6 = t4 * t2;

            double epc2 = ep2 * c2;
            double epc4 = epc2 * epc2;
            double epc6 = epc4 * epc2;
            double epc8 = epc6 * epc2;

            double T1 = S * k0;
            double T2 = nusck0 / 2;
            double T3 = nusck0 * c2 * (5 - t2 + 9 * epc2 + 4 * epc4) / 24;
            double T4 = nusck0 * c4 *
                        (61 - 58 * t2 + t4 + 270 * epc2 - 330 * t2 * epc2 + 445 * epc4 + 324 * epc6 - 680 * t2 * epc4 +
                         88 * epc8 - 660 * t2 * epc6 - 192 * t2 * epc8) / 720;
            double T5 = nusck0 * c6 * (1385 - 3111 * t2 + 543 * t4 - t6) / 40320;

            double T6 = nuck0;
            double T7 = nuck0 * c2 * (1 - t2 + epc2) / 6;
            double T8 = nuck0 * c4 *
                        (5 - 18 * t2 + t4 + 14 * epc2 - 58 * t2 * epc2 + 13 * epc4 + 4 * epc6 - 64 * t2 * epc4 -
                         24 * t2 * epc6) / 120;
            double T9 = nuck0 * c6 * (61 - 479 * t2 + 179 * t4 - t6) / 5040;

            double dl = lon_rad - lon_mer;
            double dl2 = dl * dl;
            double dl4 = dl2 * dl2;
            double dl6 = dl4 * dl2;
            double dl8 = dl6 * dl2;

            *N = FN + T1 + dl2 * T2 + dl4 * T3 + dl6 * T4 + dl8 * T5;
            *E = FE + dl * (T6 + dl2 * T7 + dl4 * T8 + dl6 * T9);
        }

        void tm_to_geographic(double a, double e2, double k0, double lon_mer, double FN, double FE, double N, double E,
                              double *lat_rad, double *lon_rad) {
            double ep2 = e2 / (1 - e2);
            double f = 1 - sqrt(1 - e2);
            double n = f / (2 - f);
            double b = a * (1 - f);

            double n2 = n * n;
            double n3 = n2 * n;
            double n4 = n3 * n;
            double n5 = n4 * n;

            double x = E - FE;
            double y = N - FN;

            /* ITERATE TO FIND PHI (DENOTED AS PHI PRIME IN DMA 8358.2) */
            /* THE LATITUDE AT THE CENTRAL MERIDIAN WHICH HAS COORDINATE (0,Y) */

            double phi = y / b / k0;
            double s;

            while (1) {
                s = sin(phi);

                double Ap = a * (1 - n + 5 * (n2 - n3) / 4 + 81 * (n4 - n5) / 64);
                double Bp = 3 * a * (n - n2 + 7 * (n3 - n4) / 8 + 55 * n5 / 64) / 2;
                double Cp = 15 * a * (n2 - n3 + 3 * (n4 - n5) / 4) / 16;
                double Dp = 35 * a * (n3 - n4 + 11 * n5 / 16) / 48;
                double Ep = 315 * a * (n4 - n5) / 512;

#if 0
                double s2phi = sin(2*phi);
                double s4phi = sin(4*phi);
                double s6phi = sin(6*phi);
                double s8phi = sin(8*phi);
#else
                double c = cos(phi);
                double s2phi = 2.0 * s * c;
                double c2phi = c * c - s * s;
                double s4phi = 2.0 * s2phi * c2phi;
                double c4phi = c2phi * c2phi - s2phi * s2phi;
                double s6phi = s4phi * c2phi + s2phi * c4phi;
                double s8phi = 2.0 * s4phi * c4phi;
#endif

                double S = Ap * phi - Bp * s2phi + Cp * s4phi - Dp * s6phi + Ep * s8phi;

                double T1 = S * k0;

#if 0
                std::cout << std::fixed << "ITERATE: "
                  << std::setw(10) << std::setprecision(5) << fabs(T1-y) << ' '
                  << std::setprecision(10) << phi*180/M_PI << ' '
                  << std::setprecision(3) << y << ' '
                  << std::setprecision(3) << T1 << std::endl;
#endif

                if (fabs(T1 - y) < TM_TO_GEOGRAPHIC_TOLERANCE_M)break;

                phi *= y / T1;
            }

#if !defined(__cplusplus)
            if(1) { /* Open up new scope to declare new variables in C */
#endif

            double s2 = s * s;

            double nu = a / sqrt(1 - e2 * s2);
            double rho = nu / (1 - e2 * s2) * (1 - e2);

            double c = cos(phi);
            double c2 = c * c;

            double t = s / c;
            double t2 = t * t;
            double t4 = t2 * t2;
            double t6 = t4 * t2;

            double nuk0 = nu * k0;
            double nuk02 = nuk0 * nuk0;
            double nuk04 = nuk02 * nuk02;
            double nuk06 = nuk04 * nuk02;

            double t_rhonuk0k0 = t / (rho * nuk0 * k0);
            double _nuck0 = 1 / (nu * c * k0);

            double epc2 = ep2 * c2;
            double epc4 = epc2 * epc2;
            double epc6 = epc4 * epc2;
            double epc8 = epc6 * epc2;

            double T10 = t_rhonuk0k0 / 2;
            double T11 = t_rhonuk0k0 / nuk02 * (5 + 3 * t2 + epc2 - 4 * epc4 - 9 * t2 * epc2) / 24;
            double T12 = t_rhonuk0k0 / nuk04 *
                         (61 + 90 * t2 + 46 * epc2 + 45 * t4 - 252 * t2 * epc2 - 3 * epc4 + 100 * epc6 -
                          66 * t2 * epc4 - 90 * t4 * epc2 + 88 * epc8 + 225 * t4 * epc4 + 84 * t2 * epc6 -
                          192 * t2 * epc8) / 720;
            double T13 = t_rhonuk0k0 / nuk06 * (1385 + 3633 * t2 + 4095 * t4 + 1575 * t6) / 40320;

            double T14 = _nuck0;
            double T15 = _nuck0 / nuk02 * (1 + 2 * t2 + epc2) / 6;
            double T16 = _nuck0 / nuk04 *
                         (5 + 6 * epc2 + 28 * t2 - 3 * epc4 + 8 * t2 * epc2 + 24 * t4 - 4 * epc6 + 4 * t2 * epc4 +
                          24 * t2 * epc6) / 120;
            double T17 = _nuck0 / nuk06 * (61 + 662 * t2 + 1320 * t4 + 720 * t6) / 5040;

            double x2 = x * x;
            double x4 = x2 * x2;
            double x6 = x4 * x2;
            double x8 = x6 * x2;

            *lat_rad = phi - x2 * T10 + x4 * T11 - x6 * T12 + x8 * T13;
            *lon_rad = lon_mer + x * (T14 - x2 * T15 + x4 * T16 - x6 * T17);

#if !defined(__cplusplus)
            }
#endif
        }

        void geographic_to_ps(double a, double e2, double k0, Hemisphere hemi, double FN, double FE, double lat_rad,
                              double lon_rad, double *N, double *E) {
            double e = sqrt(e2);
            double C0 = 2 * a / sqrt(1 - e2) * pow((1 - e) / (1 + e), e / 2);
            double tanzhalf;
            double R;

            double s_lat = sin(lat_rad);

            if (hemi == HEMI_NORTH)
                tanzhalf = pow((1 + e * s_lat) / (1 - e * s_lat), e / 2) * tan(M_PI / 4 - lat_rad / 2);
            else
                tanzhalf = pow((1 - e * s_lat) / (1 + e * s_lat), e / 2) * tan(M_PI / 4 + lat_rad / 2);

            R = k0 * C0 * tanzhalf;

            *E = FE + R * sin(lon_rad);
            if (hemi == HEMI_NORTH)
                *N = FN - R * cos(lon_rad);
            else if (hemi == HEMI_SOUTH)
                *N = FN + R * cos(lon_rad);
        }

        void ps_to_geographic(double a, double e2, double k0, Hemisphere hemi, double FN, double FE, double N, double E,
                              double *lat_rad, double *lon_rad) {
            double e = sqrt(e2);
            double C0 = 2 * a / sqrt(1 - e2) * pow((1 - e) / (1 + e), e / 2);
            double e4 = e2 * e2;
            double e6 = e4 * e2;
            double e8 = e6 * e2;
            double Abar = e2 / 2 + 5 * e4 / 24 + e6 / 12 + 13 * e8 / 360;
            double Bbar = 7 * e4 / 48 + 29 * e6 / 240 + 811 * e8 / 11520;
            double Cbar = 7 * e6 / 120 + 81 * e8 / 1120;
            double Dbar = 4279 * e8 / 161280;
            double x = E - FE;
            double y = N - FN;

            if ((x == 0) && (y == 0)) {
                *lon_rad = 0; /* undefined */
                if (hemi == HEMI_NORTH)
                    *lat_rad = M_PI / 2;
                else if (hemi == HEMI_SOUTH)
                    *lat_rad = -M_PI / 2;
            } else {
                double R;
                double tanzhalf;
                double chi;
                double phi;

                if (hemi == HEMI_NORTH)
                    *lon_rad = atan2(x, -y);
                else if (hemi == HEMI_SOUTH)
                    *lon_rad = atan2(x, y);

                if (y == 0)
                    R = fabs(x);
                else if (x == 0)
                    R = fabs(y);
                else
                    R = fabs(x / sin(*lon_rad));

                tanzhalf = R / (k0 * C0);
                chi = M_PI / 2 - 2 * atan(tanzhalf);

#if 0
                phi = chi +
              Abar*sin(2*chi) + Bbar*sin(4*chi) + Cbar*sin(6*chi) + Dbar*sin(8*chi);
#else
                double s2chi = sin(2.0 * chi);
                double c2chi = cos(2.0 * chi);
                double s4chi = 2.0 * s2chi * c2chi;
                double c4chi = c2chi * c2chi - s2chi * s2chi;
                double s6chi = s4chi * c2chi + s2chi * c4chi;
                double s8chi = 2.0 * s4chi * c4chi;
                phi = chi + Abar * s2chi + Bbar * s4chi + Cbar * s6chi + Dbar * s8chi;
#endif

                if (hemi == HEMI_NORTH)
                    *lat_rad = phi;
                else if (hemi == HEMI_SOUTH)
                    *lat_rad = -phi;
            }

            return;
        }

#define RAD(x) ((x)/180.0*M_PI)

#define UTM_K0       0.9996
#define UTM_FN_NH    0.0
#define UTM_FN_SH    10000000.0
#define UTM_FE       500000.0

#define UPS_K0       0.994
#define UPS_FN       2000000.0
#define UPS_FE       2000000.0

        int geographic_to_grid(double a, double e2, double lat_rad, double lon_rad, GridZone *zone, Hemisphere *hemi,
                               double *N, double *E) {
            if ((lat_rad > RAD(90)) || (lat_rad < RAD(-90)))return 0;
            if ((lon_rad > RAD(180)) || (lon_rad < RAD(-180))) {
                lon_rad = fmod(fmod(lon_rad, RAD(360)) + RAD(360), RAD(360));
                if (lon_rad > RAD(180))lon_rad -= RAD(360);
            }

            if (*zone == GRID_AUTO) {
                if (lat_rad >= RAD(84))*zone = UPS_NORTH;
                else if (lat_rad < RAD(-80))*zone = UPS_SOUTH;
                else *zone = UTM_ZONE_AUTO;
            }

            if ((*zone == UPS_NORTH) || (*zone == UPS_SOUTH)) {
                if (*zone == UPS_NORTH)*hemi = HEMI_NORTH;
                else *hemi = HEMI_SOUTH;

                if (e2 != 0)
                    geographic_to_ps(a, e2, UPS_K0, *hemi, UPS_FN, UPS_FE, lat_rad, lon_rad, N, E);
                else
                    geographic_to_ps_sphere(a, UPS_K0, *hemi, UPS_FN, UPS_FE, lat_rad, lon_rad, N, E);
            } else {
                unsigned izone = (unsigned) *zone;

                double lon_mer;
                double fn;

                if ((izone < 1) || (izone > 60)) {
                    izone = (unsigned) ((lon_rad + RAD(180)) / RAD(6)) + 1;
                    if ((lat_rad >= RAD(56)) && (lat_rad < RAD(64)) && (lon_rad >= RAD(3)) && (lon_rad < RAD(12)))
                        izone = 32;
                    else if ((lat_rad >= RAD(72)) && (lat_rad < RAD(84)) && (lon_rad >= RAD(0))) {
                        if (lon_rad < RAD(9))izone = 31;
                        else if (lon_rad < RAD(21))izone = 33;
                        else if (lon_rad < RAD(33))izone = 35;
                        else if (lon_rad < RAD(42))izone = 37;
                    }
                }

                *zone = (GridZone) izone;
                //std::cout<<*zone<<std::endl;
                if ((*hemi != HEMI_NORTH) && (*hemi != HEMI_SOUTH))
                    *hemi = lat_rad >= 0 ? HEMI_NORTH : HEMI_SOUTH;

                lon_mer = (double) (izone - 1) * RAD(6) - RAD(180) + RAD(3);

                if (*hemi == HEMI_NORTH)fn = UTM_FN_NH;
                else fn = UTM_FN_SH;

                if (e2 != 0)
                    geographic_to_tm(a, e2, UTM_K0, lon_mer, fn, UTM_FE, lat_rad, lon_rad, N, E);
                else
                    geographic_to_tm_sphere(a, UTM_K0, lon_mer, fn, UTM_FE, lat_rad, lon_rad, N, E);
            }

            return 1;
        }

        int grid_to_geographic(double a, double e2, GridZone zone, Hemisphere hemi, double N, double E, double *lat_rad,
                               double *lon_rad) {
            if ((zone == UPS_NORTH) || (zone == UPS_SOUTH)) {
                if (zone == UPS_NORTH)hemi = HEMI_NORTH;
                else hemi = HEMI_SOUTH;

                if (e2 != 0)
                    ps_to_geographic(a, e2, UPS_K0, hemi, UPS_FN, UPS_FE, N, E, lat_rad, lon_rad);
                else
                    ps_to_geographic_sphere(a, UPS_K0, hemi, UPS_FN, UPS_FE, N, E, lat_rad, lon_rad);
            } else {
                unsigned izone = (unsigned) zone;
                double lon_mer;
                double fn;

                if ((izone < 1) || (izone > 60))return 0;
                if ((hemi != HEMI_NORTH) && (hemi != HEMI_SOUTH))return 0;

                lon_mer = (double) (izone - 1) * RAD(6) - RAD(180) + RAD(3);

                if (hemi == HEMI_NORTH)fn = UTM_FN_NH;
                else fn = UTM_FN_SH;

                if (e2 != 0)
                    tm_to_geographic(a, e2, UTM_K0, lon_mer, fn, UTM_FE, N, E, lat_rad, lon_rad);
                else
                    tm_to_geographic_sphere(a, UTM_K0, lon_mer, fn, UTM_FE, N, E, lat_rad, lon_rad);
            }

            return 1;
        }


    }  // namespace transform
}  // namespace cartographer
