// Author: Akash Patel (apatel435@gatech.edu)
// Purpose: Determine phi vectors for each input pose
//   This phi will be used for finding beta (weights of actual robot) via gradient descent
//
// Input: Ideal beta={mi, MXi, MYi, ...}, krang urdf model, perturbation value,
//   potentially unbalanced data points (q/poses) as a file,
// Output: Phi matrix as a file

#include <dart/dart.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <iostream>
#include <fstream>
#include <nlopt.hpp>

using namespace std;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;

#define MAXBUFSIZE ((int) 1e6)

int genPhiMatrixAsFile();

struct comOptParams {
  SkeletonPtr robot;
  Eigen::Matrix<double, 25, 1> qInit;
};

double comOptFunc(const std::vector<double> &x, std::vector<double> &grad, void *my_func_data) {
  comOptParams* optParams = reinterpret_cast<comOptParams *>(my_func_data);
  Eigen::Matrix<double, 25, 1> q(x.data());

  if (!grad.empty()) {
    Eigen::Matrix<double, 25, 1> mGrad = q-optParams->qInit;
    Eigen::VectorXd::Map(&grad[0], mGrad.size()) = mGrad;
  }
  return (0.5*pow((q-optParams->qInit).norm(), 2));
}

double comConstraint(const std::vector<double> &x, std::vector<double> &grad, void *com_const_data) {
  comOptParams* optParams = reinterpret_cast<comOptParams *>(com_const_data);
  Eigen::Matrix<double, 25, 1> q(x.data());
  optParams->robot->setPositions(q);
  return (pow(optParams->robot->getCOM()(0)-optParams->robot->getPosition(3), 2) \
    + pow(optParams->robot->getCOM()(1)-optParams->robot->getPosition(4), 2));
}

double wheelAxisConstraint(const std::vector<double> &x, std::vector<double> &grad, void *wheelAxis_const_data) {
  comOptParams* optParams = reinterpret_cast<comOptParams *>(wheelAxis_const_data);
  Eigen::Matrix<double, 25, 1> q(x.data());
  optParams->robot->setPositions(q);
  return optParams->robot->getBodyNode(0)->getTransform().matrix()(2,0);
}

double headingConstraint(const std::vector<double> &x, std::vector<double> &grad, void *heading_const_data) {
  comOptParams* optParams = reinterpret_cast<comOptParams *>(heading_const_data);
  Eigen::Matrix<double, 25, 1> q(x.data());
  optParams->robot->setPositions(q);
  Eigen::Matrix<double, 4, 4> Tf = optParams->robot->getBodyNode(0)->getTransform().matrix();
  double heading = atan2(Tf(0,0), -Tf(1,0));
  optParams->robot->setPositions(optParams->qInit);
  Tf = optParams->robot->getBodyNode(0)->getTransform().matrix();
  double headingInit = atan2(Tf(0,0), -Tf(1,0));
  return heading-headingInit;
}

int main() {
    genPhiMatrixAsFile();
}

int genPhiMatrixAsFile() {

    // Inputs go first (so you dont have to dig through code to change them)
    // Maybe make this method accept inputs so that you can assign inputs in
    // main method
    int cols = 0, rows = 0;
    double buff[MAXBUFSIZE];

    // I feel like there is a better way to read the file (if it fits it ships?)
    // Read numbers (the pose params)
    ifstream infile;
    infile.open("../defaultInit.txt");
    while(! infile.eof()) {
        string line;
        getline(infile, line);

        int temp_cols = 0;
        stringstream stream(line);
        while(! stream.eof())
            stream >> buff[cols*rows+temp_cols++];

        if (temp_cols == 0)
            continue;

        if (cols == 0)
            cols = temp_cols;

        rows++;
    }

    infile.close();
    rows--;

    // Populate matrix with numbers.
    // Eigen matrix is transpose of read file
    // every column is a pose, the rows are the pose params
    // Heading, qBase, x, y, z, qLWheel, qRWheel, qWaist, qTorso, qKinect,
    // qLArm0, ..., qLArm6, qRArm0, ..., qRArm6

    int matRows = cols;
    int matCols = rows;
    Eigen::MatrixXd allInitPoseParams(matRows, matCols);
    for (int i = 0; i < matRows; i++)
        for (int j = 0; j < matCols; j++)
            allInitPoseParams(i,j) = buff[matCols*i+j];

    // Perturbation Value
    // TODO: Change value
    // double perturbedValue = std::pow(1, -5);
    // Placeholder of perturbedValue
    double perturbedValue = 0.01;
    // I think following num is too small
    // double perturbedValue = std::pow(1, -17);

    // Instantiate "ideal" robot and n other robots
    // Somehow mayke idealRobot a copy of krang model
    // There has to be a way to do relative filepaths in DART
    // but until I figure that out I guess this works
    dart::utils::DartLoader loader;
    dart::dynamics::SkeletonPtr idealRobot = loader.parseSkeleton("/home/apatel435/Desktop/09-URDF/Krang/Krang.urdf");

    // Should this be like an eigenvector of doubles or something
    // Create ideal beta
    // Wait do we get beta from the ideal robot or give it to the ideal robot?
    //   beta = [5, 10, 10]
    // Determine dimension of beta for n
    //   int n = dim(beta)
    //   Below line in temporary placeholder (should be dimension/size of beta
    //   array
    //TODO
    int n = matRows;

    int numPertRobots = n;

    // Maybe this can be done perturbing one value of a single robot each time
    // Make an array of robots to be perturbed (initially just ideal robots)
    //   perturbedRobotArray = array of N idealRobot
    // dart::dynamics::SkeletonPtr perturbedRobotArray[numPertRobots];
    // Perturb respective beta values
    // for (int i = 0; i < numPertRobots; i++) {
        // Need to be careful of copying and referencing
        // perturbedRobotArray[i] = idealRobot;
        //  Get the ith beta (either by input beta or input idealRobot's beta
        //  WHOA actually do we need to assign a bunch of robots?
        //  cant we just make a perturbed beta matrix and an ideal beta matrix

        //  double idealB = (idealRobot->beta)(i)
        //  (perturbedRobotArray(i)->beta)(i) = idealB + perturbedValue;

    // What's the format for DART
    // first three axis-angle (aa)
    // aa1, aa2, aa3, x, y, z, qLWheel, qRWheel, qWaist, qTorso, qKinect,
    // qLArm0, ..., qLArm6, qRArm0, ..., qRArm6

    int numInputPoses = matCols;

    // Open output file to create phi matrix
    ofstream phiFile;
    phiFile.open("phiMatrix.txt");

    // Beta Definition/Format
    // mi, mxi, myi, mzi for each body
    int bodyParams = 4;
    int numBodies = idealRobot->getNumBodyNodes();
    dart::dynamics::BodyNodePtr bodyi;
    string namei;
    double mi;
    double xMi;
    double yMi;
    double zMi;
    //phiFile << "Number of bodies in Krang model: " << numBodies << "\n";

    Eigen::MatrixXd betaParams(1, numBodies*bodyParams);
    //Eigen::Matrix<double, 1, numBodies*bodyParams> betaParams;

    for (int i = 0; i < numBodies; i++) {
        // now how to change these values so when we find the whole robot COM it
        // is a value from the perturbed beta value
        bodyi = idealRobot->getBodyNode(i);
        namei = bodyi->getName();
        mi = bodyi->getMass();
        xMi = mi * bodyi->getLocalCOM()(0);
        yMi = mi * bodyi->getLocalCOM()(1);
        zMi = mi * bodyi->getLocalCOM()(2);
        //phiFile << namei << " " << mi << " " << xCOMi << " " << yCOMi << " " << zCOMi << "\n";

        betaParams(0, i * bodyParams + 0) = mi;
        betaParams(0, i * bodyParams + 1) = xMi;
        betaParams(0, i * bodyParams + 2) = yMi;
        betaParams(0, i * bodyParams + 3) = zMi;

    }

    // Find phi
    double phi = 0;

    for (int pose = 0; pose < numInputPoses; pose++) {
        Eigen::Matrix<double, 24, 1> initPoseParams;
        for (int j = 0; j < matRows; j++) {
            initPoseParams(j) = allInitPoseParams(j, pose);
        }

        double headingInit = initPoseParams(0);
        double qBaseInit = initPoseParams(1);
        Eigen::Matrix<double, 22, 1> unchangedValues; unchangedValues << initPoseParams.segment(2,22);

        // Calculating the axis angle representation of orientation from headingInit and qBaseInit:
        // RotX(pi/2)*RotY(-pi/2+headingInit)*RotX(-qBaseInit)
        Eigen::Transform<double, 3, Eigen::Affine> baseTf = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
        baseTf.prerotate(Eigen::AngleAxisd(-qBaseInit,Eigen::Vector3d::UnitX())).prerotate(Eigen::AngleAxisd(-M_PI/2+headingInit,Eigen::Vector3d::UnitY())).prerotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd aa(baseTf.matrix().block<3,3>(0,0));

       // Now compile this data into dartPoseParams

        Eigen::Matrix<double, 25, 1> dartPoseParams;
        dartPoseParams << aa.angle()*aa.axis(), unchangedValues;

        // Change Pose such that CoM is right on top of wheel axis
        const int dof = (const int) idealRobot->getNumDofs();
        comOptParams optParams;
        optParams.robot = idealRobot;
        optParams.qInit << dartPoseParams;
        nlopt::opt opt(nlopt::LN_COBYLA, dof);
        std::vector<double> unoptDartPoseParams(dof);
        double minf;
        opt.set_min_objective(comOptFunc, &optParams);
        opt.add_equality_constraint(comConstraint, &optParams, 1e-8);
        opt.add_equality_constraint(wheelAxisConstraint, &optParams, 1e-8);
        opt.add_equality_constraint(headingConstraint, &optParams, 1e-8);
        opt.set_xtol_rel(1e-4);
        opt.set_maxtime(10);
        opt.optimize(unoptDartPoseParams, minf);
        Eigen::Matrix<double, 25, 1> optDartPoseParams(unoptDartPoseParams.data());

        // Actually we can just compute straight from here for each robot right?

        // Set position of ideal robot to the pose in DART format
        idealRobot->setPositions(optDartPoseParams);
        // Need to make sure im using the right function call for getting
        // the COM (there is also a getCOM()(1) and getCOM()(2) i guess one for
        // each x, y, z
        double xCOMIdealRobot = idealRobot->getCOM()(0);
        phiFile << xCOMIdealRobot;
        // Set position of all perturbed robots to dartPoseParams
        // int pertRobotNum = 0;
        for (int pertRobotNum = 0; pertRobotNum < numPertRobots ; pertRobotNum++) {
            // Get the perturbed robot
            //   pertRobot = perturbedRobotArray(pertRobotNum);
            //   Maybe i can create the robot here and perturb the beta value as
            //   I go through the loop
            dart::dynamics::SkeletonPtr pertRobot = loader.parseSkeleton("/home/apatel435/Desktop/09-URDF/Krang/Krang.urdf");
            // pertRobot->beta(pertRobotNum) = pertRobot->beta(pertRobotNum) + perturbedValue;
            // Set perturbed robot position to pose
            // Look into if I can set positions before changing my beta values
            // would speed up (only set position once for each pose instead of
            // everytime i have a new beta value to change
            pertRobot->setPositions(optDartPoseParams);
            // Does above line cause a long time to run???
            // Get the center of mass of the perturbedRobot
            // Need to make sure im using the right function call for getting
            // the COM (there is also a getCOM()(1))
            double xCOMpertRobot = pertRobot->getCOM()(0);
            // Placeholder value for xCOMpertRobot
            // double xCOMpertRobot = pertRobotNum + pose;
            // Calculate phi for betai and pose
            phi = (xCOMpertRobot - xCOMIdealRobot)/perturbedValue;
            // Add phi value to line (if last phi value then add newline as well
            if (pertRobotNum == numPertRobots - 1) {
                phiFile << phi << "\n";
            } else {
                phiFile << phi << " ";
            }
        }
    }

    phiFile.close();

    return 0;
}
