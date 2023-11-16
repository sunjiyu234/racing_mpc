
struct CarParams {
    double wheelbase;
    double friction_coeff;
    double h_cg; // height of car's CG
    double l_f; // length from CG to front axle
    double l_r; // length from CG to rear axle
    double mass;
    double I_z; // moment of inertia about z axis from CG
    double R_wheel;  // radius of wheel
    double Cf; 
    double Bf;
    double Ef;
    double Cr;
    double Br;
    double Er;
};