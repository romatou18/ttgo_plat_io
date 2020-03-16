#pragma once

enum {
    W = 0,
    X = 1,
    Y = 2,
    Z = 3
} quat_index;

typedef struct {	
    int a[3];
    int g[3];
    int m[3];
    long q[4];
    float yaw;
    float pitch;
    float roll;
    unsigned long time;
    float heading;
} imu_raw_t; 