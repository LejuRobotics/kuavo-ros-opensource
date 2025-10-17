#ifndef UTIL_H_
#define UTIL_H_

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include <iostream>
#include <vector>

char *floatToChar(double value);
void printAbsolutePath(const char* relativePath);
uint16_t sw2cw(const uint16_t state_word);
void toLittleEndian32(int32_t value,uint8_t* buf);
void printAbsolutePath(const char* relativePath);
bool csvRead(const char *file_name, bool skip_header, std::vector<std::vector<double>> &data);
double calcCos(double start, double stop, double T, double t);

double get_sin_wave(double A, double T, double b, double dt);
double get_cos_wave(double A, double T, double b, double dt);
double get_square_wave(double A, double T, double b, double dt);

#endif
