#pragma once


int circle_circle_intersection(double x0, double y0, double r0,
                               double x1, double y1, double r1,
                               double &xi, double &yi,
                               double &xi_prime, double &yi_prime);

void run_test(double x0, double y0, double r0,
              double x1, double y1, double r1);