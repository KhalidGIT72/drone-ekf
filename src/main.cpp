#include <iostream>
#include <fstream>
#include <sstream>
#include <vector>
#include <Eigen/Dense>

using namespace Eigen;
using namespace std;

// Read CSV file into a 2D vector
vector<vector<double>> readCSV(const string& filename) {
    vector<vector<double>> data;
    ifstream file(filename);
    string line;
    while (getline(file, line)) {
        vector<double> row;
        stringstream ss(line);
        string val;
        while (getline(ss, val, ',')) {
            row.push_back(stod(val));
        }
        data.push_back(row);
    }
    return file.is_open() ? data : vector<vector<double>>{};
}

int main() {
    // Load measurements
    auto meas_data = readCSV("../data/measurements.csv");
    int N = meas_data.size();

    if (N == 0) {
        cerr << "Error: could not read measurements.csv" << endl;
        return 1;
    }

    // EKF parameters
    double dt = 0.1;

    // State transition matrix F
    Matrix4d F;
    F << 1, 0, dt, 0,
         0, 1, 0,  dt,
         0, 0, 1,  0,
         0, 0, 0,  1;

    // Measurement matrix H
    MatrixXd H(2, 4);
    H << 1, 0, 0, 0,
         0, 1, 0, 0;

    // Process noise Q
    Matrix4d Q = Matrix4d::Zero();
    Q(0,0) = 0.01; Q(1,1) = 0.01;
    Q(2,2) = 0.04; Q(3,3) = 0.04;

    // Measurement noise R
    Matrix2d R;
    R << 4, 0,
         0, 4;

    // Initial state and covariance
    Vector4d x;
    x << 0, 0, 5, 2;
    Matrix4d P = Matrix4d::Identity();

    // Output file
    ofstream out("../results/ekf_output.csv");
    out << "t,x_est,y_est,vx_est,vy_est\n";

    // EKF loop
    for (int k = 0; k < N; k++) {
        double t  = meas_data[k][0];
        double zx = meas_data[k][1];
        double zy = meas_data[k][2];

        // Predict
        x = F * x;
        P = F * P * F.transpose() + Q;

        // Update
        Vector2d z;
        z << zx, zy;
        Vector2d y = z - H * x;
        Matrix2d S = H * P * H.transpose() + R;
        MatrixXd K = P * H.transpose() * S.inverse();
        x = x + K * y;
        P = (Matrix4d::Identity() - K * H) * P;

        // Write output
        out << t << "," << x(0) << "," << x(1) << ","
            << x(2) << "," << x(3) << "\n";
    }

    out.close();
    cout << "EKF complete. Results saved to results/ekf_output.csv" << endl;

    return 0;
}