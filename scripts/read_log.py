import numpy as np

if __name__ == "__main__":
    gps_data = np.loadtxt("gps_log_20230616_224322.csv",
                          delimiter=",", skiprows=1)
    print(gps_data[0], gps_data[0][0])

    imu_data = np.loadtxt("imu_log_20230616_224322.csv",
                          delimiter=",", skiprows=1)
    print(imu_data[0], imu_data[0][0])