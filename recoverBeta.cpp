// Author: Akash Patel (apatel435@gatech.edu)

// genPhiMatrixAsFile
// Purpose: Determine phi vectors for each input pose
//   This phi will be used for finding beta (weights of actual robot) via gradient descent
//
// Input: Ideal beta={mi, MXi, MYi, ...}, krang urdf model, perturbation value,
//   potentially unbalanced data points (q/poses) as a file,
// Output: Phi matrix as a file

// convergeToBeta
// Purpose: Converge to an optimal beta vector
//   This beta is the parameter vector of Krang
//
// Input: Perturbed beta={mi, MXi, MYi, ...}, phi matrix,
// Output: Converged beta vector as a file
//
//
// Overall Input: Poses in {heading, qBase, etc.} format
// Overall Output: Converged beta value
// Intermediary Input/Output Flow:
// Input Pose File -> Dart Poses -> Opt Dart Poses -> Phi Matrix
// Phi Matrix -> Converged Beta
//

////////////////////////////////////////////////////////////////////////////////
//           Use your search feature to find hardcoded inputs to this script
//           Search for "INPUT on below"
////////////////////////////////////////////////////////////////////////////////

// TODO: Need to combine methods together so I am not reading/writing files more
// than once and not creating multiple copies of the same data
// TODO: Perform C++ warning checks
// TODO: Check for memory leaks (valgrind)
// TODO: Optimize script for speed

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

int genPhiMatrixAsFile() {

    // Put a hard stop on reading poses just in case
    // INPUT on below line (Hard stop to number of pose readings)
    //int controlPoseNums = 4700;
    int controlPoseNums = 1000;
    // INPUT on below line (lines to skip so an even distribution of samples can
    // be taken) Dependent on file lines
    //int linesToSkip = 65000/controlPoseNums;

    int cols = 0, rows = 0;
    double buff[MAXBUFSIZE];

    // Read numbers (the pose params)
    ifstream infile;
    // infile.open("../defaultInit.txt");
    // INPUT on below line (input pose file)
    infile.open("../krangHardwarePoses.txt");
    cout << "Reading input poses ...\n";
//    int lineNumber = 0;
    while(! infile.eof() && rows <= controlPoseNums) {
 //       if (lineNumber == linesToSkip) {
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
    //        lineNumber = 0;
   //     }
  //      lineNumber++;
    }

    infile.close();
    cout << "|-> Done\n";
    rows--;

    // Populate matrix with numbers.
    // Eigen matrix is transpose of read file
    // every column is a pose, the rows are the pose params
    // Heading, qBase, x, y, z, qLWheel, qRWheel, qWaist, qTorso, qKinect,
    // qLArm0, ..., qLArm6, qRArm0, ..., qRArm6

    int numInputPoses = rows;
    int numParams = cols;

    Eigen::MatrixXd allInitPoseParamsFromFile(rows, cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            allInitPoseParamsFromFile(i,j) = buff[cols*i+j];

    Eigen::MatrixXd allInitPoseParams(cols, rows);
    allInitPoseParams = allInitPoseParamsFromFile.transpose();

    // Perturbation Value
    // INPUT on below line (perturbation value for finding phi)
    double perturbedValue = std::pow(1, -17);

    // Instantiate "ideal" robot and n other robots
    cout << "Creating ideal beta vector ...\n";
    dart::utils::DartLoader loader;
    // INPUT on below line (absolute path of the Krang URDF file)
    dart::dynamics::SkeletonPtr idealRobot = loader.parseSkeleton("/home/apatel435/Desktop/09-URDF/Krang/Krang.urdf");

    // Create ideal beta
    // Wait do we get beta from the ideal robot or give it to the ideal robot?
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

    Eigen::MatrixXd betaParams(1, numBodies*bodyParams);
    int numPertRobots = numBodies*bodyParams;

    for (int i = 0; i < numBodies; i++) {
        bodyi = idealRobot->getBodyNode(i);
        namei = bodyi->getName();
        mi = bodyi->getMass();
        xMi = mi * bodyi->getLocalCOM()(0);
        yMi = mi * bodyi->getLocalCOM()(1);
        zMi = mi * bodyi->getLocalCOM()(2);

        betaParams(0, i * bodyParams + 0) = mi;
        betaParams(0, i * bodyParams + 1) = xMi;
        betaParams(0, i * bodyParams + 2) = yMi;
        betaParams(0, i * bodyParams + 3) = zMi;
    }
    cout << "|-> Done\n";
    cout << "Creating robot array ...\n";
    // TODO: Need to create an array of pertRobots in a fast time
    // Create array of robots out of pose loop for fast times
    // then change appropriate values (betaParams(i)) for each robot when
    // going through all the robots
    dart::dynamics::SkeletonPtr pertRobotArray[sizeof(SkeletonPtr) * numPertRobots];
    for (int i = 0; i < numPertRobots; i++) {

        // TODO: Segfaulting right here
        // Trying to create an array of idealRobots by calling parseSkeleton
        // only once since it is time expenseive
        //memcpy(pertRobotArray[i], idealRobot, sizeof(SkeletonPtr));

        pertRobotArray[i] = loader.parseSkeleton("/home/apatel435/Desktop/09-URDF/Krang/Krang.urdf");

    }

    cout << "|-> Done\n";
    // Create file to print the actual balanced poses used in DART
    ofstream balancedPosesFile;
    balancedPosesFile.open("balancedPoses.txt");

    // Find phiMatrix
    cout << "Calculating Phi Matrix ...\n";
    Eigen::MatrixXd phiMatrix(numInputPoses, numPertRobots);
    double phi = 0;

    // Find the pose in DART formats
    for (int pose = 0; pose < numInputPoses; pose++) {
        Eigen::Matrix<double, 24, 1> initPoseParams;
        for (int j = 0; j < numParams; j++) {
            initPoseParams(j) = allInitPoseParams(j, pose);
        }

        double headingInit = initPoseParams(0);
        double qBaseInit = initPoseParams(1);
        Eigen::Matrix<double, 22, 1> unchangedValues;
        unchangedValues << initPoseParams.segment(2,22);

        // Calculating the axis angle representation of orientation from headingInit and qBaseInit:
        // RotX(pi/2)*RotY(-pi/2+headingInit)*RotX(-qBaseInit)
        Eigen::Transform<double, 3, Eigen::Affine> baseTf = Eigen::Transform<double, 3, Eigen::Affine>::Identity();
        baseTf.prerotate(Eigen::AngleAxisd(-qBaseInit,Eigen::Vector3d::UnitX())).prerotate(Eigen::AngleAxisd(-M_PI/2+headingInit,Eigen::Vector3d::UnitY())).prerotate(Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitX()));
        Eigen::AngleAxisd aa(baseTf.matrix().block<3,3>(0,0));

        // Now compile this data into dartPoseParams
        Eigen::Matrix<double, 25, 1> dartPoseParams;
        dartPoseParams << aa.angle()*aa.axis(), unchangedValues;

        // Change Pose such that CoM is right on top of wheel axis
        // Im not sure on how this works or the accuracy of it
        // What I am sure is that it changes the input pose to something
        // different than specified
        // This value should be recorded
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

        // Print optDartPoseParams since these are the actual poses used in
        // following calculations
        // Remember this is in Dart's pose format which is
        // axis-angle1, axis-angle2, axix-angle3, x, y, z, qLWheel, qRWheel,
        // qWaist, qTorso, qKinect,
        // qLArm0, ..., qLArm6, qRArm0, ..., qRArm6
        // The difference in input pose and dart pose format is the conversion
        // from heading and base to axis-angle representation
        // all other values are the same
        balancedPosesFile << optDartPoseParams.transpose() << "\n";

        // Set position of ideal robot to the pose in DART format
        idealRobot->setPositions(optDartPoseParams);

        // Get x center of mass
        double xCOMIdealRobot = idealRobot->getCOM()(0);
        // Set position of all perturbed robots to dartPoseParams

        for (int pertRobotNum = 0; pertRobotNum < numPertRobots; pertRobotNum++) {

            if (pertRobotNum % bodyParams == 0) {
                pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum / bodyParams)->setMass(betaParams(0, pertRobotNum) + perturbedValue);
            }
            else if (pertRobotNum % bodyParams == 1) {
                Eigen::Vector3d bodyCOM(betaParams(0, pertRobotNum) + perturbedValue, betaParams(0, pertRobotNum + 1), betaParams(0, pertRobotNum + 2));
                pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum / bodyParams)->setLocalCOM(bodyCOM);
            } else if (pertRobotNum % bodyParams == 2) {
                Eigen::Vector3d bodyCOM(betaParams(0, pertRobotNum - 1), betaParams(0, pertRobotNum) + perturbedValue, betaParams(0, pertRobotNum + 1));
                pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum / bodyParams)->setLocalCOM(bodyCOM);
            } else {
                Eigen::Vector3d bodyCOM(betaParams(0, pertRobotNum - 2), betaParams(0, pertRobotNum - 1), betaParams(0, pertRobotNum) + perturbedValue);
                pertRobotArray[pertRobotNum]->getBodyNode(pertRobotNum / bodyParams)->setLocalCOM(bodyCOM);
            }

            // Set perturbed robot position to pose
            pertRobotArray[pertRobotNum]->setPositions(optDartPoseParams);

            // Get the center of mass of the perturbedRobot
            double xCOMpertRobot = pertRobotArray[pertRobotNum]->getCOM()(0);

            // Calculate phi for betai and pose
            phi = (xCOMpertRobot - xCOMIdealRobot)/perturbedValue;

            // Add phi to phiMatrix and then print it looks cleaner
            phiMatrix(pose, pertRobotNum) = phi;
        }
    }

    // Open output file to write phi matrix
    ofstream phiFile;
    phiFile.open("phiMatrix.txt");

    // phi should be the same no matter estimation/perturbation
    phiFile << phiMatrix;
    phiFile.close();
    cout << "|-> Done\n";

    // Close balanced pose file
    balancedPosesFile.close();

    return 0;
}

int convergeToBeta() {

    int cols = 0, rows = 0;
    double buff[MAXBUFSIZE];

    // I feel like there is a better way to read the file (if it fits it ships?)
    // Read numbers (the pose params)
    ifstream infile;
    // infile.open("../defaultInit.txt");
    // INPUT on below line (input phi matrix file)
    infile.open("phiMatrix.txt");
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
    //rows--;
    int numInputPoses = rows;
    int numBetaParams = cols;

    // Populate matrix with numbers.
    // every column is a paramCoeff, the rows are the phi vectors for each data
    // point
    Eigen::MatrixXd phiMatrix(rows, cols);
    for (int i = 0; i < rows; i++)
        for (int j = 0; j < cols; j++)
            phiMatrix(i, j) = buff[cols*i+j];

    // make idealRobot a copy of krang model
    dart::utils::DartLoader loader;
    // INPUT on below line (absolute path of the Krang URDF model
    dart::dynamics::SkeletonPtr idealRobot = loader.parseSkeleton("/home/apatel435/Desktop/09-URDF/Krang/Krang.urdf");

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

    //TODO: Need to add perturbation to the ideal beta value read
    //I think should be read as input but for now let's manually create the
    //perturbation from the ideal beta
    // INPUT on below lines (need to create a prior beta value aka betaHat)
    double deviation = 0.01;
    double offset = 0.1;
    Eigen::MatrixXd nonIdealBetaParams(1, numBodies*bodyParams);

    ofstream krangSpecsFile;
    krangSpecsFile.open("krangSpecs.txt");

    for (int i = 0; i < numBodies; i++) {
        bodyi = idealRobot->getBodyNode(i);
        namei = bodyi->getName();
        mi = bodyi->getMass();
        xMi = mi * bodyi->getLocalCOM()(0);
        yMi = mi * bodyi->getLocalCOM()(1);
        zMi = mi * bodyi->getLocalCOM()(2);

        krangSpecsFile << namei << " " << mi << " " << xMi << " " << yMi << " " << zMi << "\n";

        nonIdealBetaParams(0, i * bodyParams + 0) = mi + deviation * mi + offset;
        nonIdealBetaParams(0, i * bodyParams + 1) = xMi + deviation * xMi + offset;
        nonIdealBetaParams(0, i * bodyParams + 2) = yMi + deviation * yMi + offset;
        nonIdealBetaParams(0, i * bodyParams + 3) = zMi + deviation * zMi + offset;

    }

    krangSpecsFile.close();

    // Initialize constants/hyperparameters
    // Eons (how many times to learn on same dataset)
    // INPUT on below line (eons)
    int eons = 10;

    // Learning Rate
    // INPUT on below line (learning rate)
    double n = 0.05;
    //0.1

    // Regularizes the importance of the masses with respect to the moments
    // INPUT on below line (mass coefficient for regularization
    double u = 0.1;
    //0.1

    // Mass Indicator Matrix
    Eigen::MatrixXd massIndicatorMatrix(1, numBetaParams);
    for (int i = 0; i < numBetaParams; i++) {
        if (i % 4 == 0) {
            massIndicatorMatrix(0, i) = 1;
        } else {
            massIndicatorMatrix(0, i) = 0;
        }
    }

    // Open output file to write beta vector
    ofstream betaFile;
    betaFile.open("betaVectors.txt");
    // Open output file to write xCOM errors
    ofstream xCOMErrorFile;
    xCOMErrorFile.open("xCOMError.txt");

    // Update beta params while looping through all the input phi vectors of
    // their respective poses
    Eigen::MatrixXd currBeta = nonIdealBetaParams;
    Eigen::MatrixXd nextBeta(1, numBetaParams);

    Eigen::MatrixXd phiVec(1, numBetaParams);
    Eigen::MatrixXd xCOM(1, 1);
    Eigen::MatrixXd xCOMError(1, 1);
    Eigen::MatrixXd delta(1, numBetaParams);

    Eigen::MatrixXd totalMass(1, 1);
    totalMass << idealRobot->getMass();

    // 1 means it has converged to a solution with a suitable error
    int hasConverged = 0;
    // INPUT on below line (satisfactory error value)
    double suitableError = 0;

    // Write the prior beta vector
    cout << "Converging to a Beta Vector ...\n";
    betaFile << currBeta << "\n";
    for (int k = 0; k < eons; k++) {
    for (int pose = 0; pose < numInputPoses && hasConverged == 0; pose++) {
        phiVec = phiMatrix.row(pose);
        // Not sure how to get total mass do i use perturbed robot or ideal
        // robot ?
        xCOM = (phiVec * currBeta.transpose()) + (u * ((massIndicatorMatrix * currBeta.transpose()) - totalMass));

        // Is this right way to calculate error or is it just xCOM from above?
        //xCOMError = phiVec * currBeta.transpose();

        // Use absolute value of error to see if solution is suitable or not
        if (abs(xCOM(0, 0)) <= suitableError) {
            hasConverged = 1;
        }
        // Should/can also write the error to a file for analysis
        xCOMErrorFile << xCOM << "\n";

        delta = phiVec + (u * massIndicatorMatrix);
        // Update currBeta parameter vector
        // currBeta = currBeta - n * delta;
        currBeta = currBeta - (n * (xCOM * delta));

        // Write the updated beta vector
        betaFile << currBeta << "\n";

    }
    }

    // Write the xCOM error of the last beta and the last pose (so the last pose
    // is used twice with two different betas)
    xCOM = (phiVec * currBeta.transpose()) + (u * ((massIndicatorMatrix * currBeta.transpose()) - totalMass));

    //xCOMError = phiVec * currBeta.transpose();
    xCOMErrorFile << xCOM << "\n";

    betaFile.close();
    cout << "|-> Done\n";
    xCOMErrorFile.close();

    return 0;
}

int main() {
    genPhiMatrixAsFile();
    convergeToBeta();
}


