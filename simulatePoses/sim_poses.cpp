// Author: Akash Patel (apatel435@gatech.edu)
// Purpose: To simulate Krang moving from one pose to another pose
//          i.e. modelling how exactly Krang does this (qdot and
//          qdoubledout[qddot])
//          This code can be used as a basis to ensure the final pose and
//          trajectory is feasible (it doesnt collide with itself) (code does not
//          consider balancing)
//
//          Question: given a final pose, the trajectory taken depends on the
//          intial pose right? so there might be cases where velocity commands
//          need to be split up into multiple commands (by vel com I mean the
//          commands of angular velocities sent for the actuators moving all
//          each joint)
//          there might also be cases where the final pose can never be reached
//          no matter the trajectory based on the intial pose
//
//          ALSO: (this is prob a whole another project and ties into the
//          dynamics of Krang) but,
//          What is the optimal trajectory from one pose to another pose
//
// Input: Krang urdf model, data points (q/poses and qdots qddots) as a file
// Output: A visualization of Krang moving from one pose to another pose as well
//          as a truth value if (initial pose -> velocity commands -> final pose) is
//          feasible
//
// STOP!!!!!
// ONE STEP AT A TIME
// LET'S FOCUS ON DETERMING IF A SINGLE POSE IS FEASIBLE
// BEFORE THAT LET'S SIMULATE A POSE IN DART
// algo lets try to read a pose file and then step through all poses in file in
// DART sim (use a keypress to advance to next pose)
// Could also use this to check collisions systemically (brute force in DART)

#include <dart/dart.hpp>
#include <dart/gui/gui.hpp>
#include <dart/utils/urdf/urdf.hpp>
#include <iostream>
#include <fstream>

using namespace std;
using namespace dart::common;
using namespace dart::dynamics;
using namespace dart::simulation;
using namespace dart::math;
using namespace dart::gui;

#define MAXBUFSIZE ((int) 1e6)

class MyWindow : public SimWindow {
public:

    // Constructor
    MyWindow(WorldPtr world) {
        setWorld(world);

        // Find the Skeleton named krang within the World
        // mKrang = world->getSkeleton("krang");
        // Make sure that krang was found in the World
        // assert(mKrang != nullptr);

    }

protected:

    // Krang
    SkeletonPtr mKrang;

};

int genFeasiblePoses() {

    // Inputs go first (so you dont have to dig through code to change them)
    // Maybe make this method accept inputs so that you can assign inputs in
    // main method

    int controlInputPoses = 1;

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

    rows++;

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

    // Instantiate krang
    // There has to be a way to do relative filepaths in DART
    // but until I figure that out I guess this works
    dart::utils::DartLoader loader;
    dart::dynamics::SkeletonPtr krang = loader.parseSkeleton("/home/apatel435/Desktop/09-URDF/Krang/Krang.urdf");
    krang->setName("krang");

    const int dof = (const int) krang->getNumDofs();
    // dof should be 25
    // What's the format
    // first three axis-angle (aa)
    // aa1, aa2, aa3, x, y, z, qLWheel, qRWheel, qWaist, qTorso, qKinect,
    // qLArm0, ..., qLArm6, qRArm0, ..., qRArm6

    int numInputPoses = matCols;

    // Open output file to create isFeasible matrix
    ofstream feasibleFile;
    feasibleFile.open("feasiblePoses.txt");

    int isFeasible = 0;

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

        // Set position of krang
        krang->setPositions(dartPoseParams);

        // TODO:
        // Need to simulate it so atleast can visually inspect feasibility

        // TODO:
        // Determine if pose is feasible

        // Write out result to file in same order as input poses
        // Should I write out the feasible pose itself?
        feasibleFile << isFeasible << "\n";
    }

    feasibleFile.close();

    return 0;
}

dart::dynamics::SkeletonPtr createFloor() {

    dart::dynamics::SkeletonPtr floor = Skeleton::create("floor");

    return floor;

}

//Eigen::MatrixXd genDartPoseParams() {
dart::dynamics::SkeletonPtr createKrang() {

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

    rows++;

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

    // Instantiate krang
    // There has to be a way to do relative filepaths in DART
    // but until I figure that out I guess this works
    dart::utils::DartLoader loader;
    dart::dynamics::SkeletonPtr krang = loader.parseSkeleton("/home/apatel435/Desktop/09-URDF/Krang/Krang.urdf");

    const int totalParams = (const int) krang->getNumDofs();
    // dof should be 25
    // What's the format
    // first three axis-angle (aa)
    // aa1, aa2, aa3, x, y, z, qLWheel, qRWheel, qWaist, qTorso, qKinect,
    // qLArm0, ..., qLArm6, qRArm0, ..., qRArm6

    int numInputPoses = matCols;
    Eigen::MatrixXd allDartPoseParams(matRows, matCols);

    int pose = 0;
    //for (int pose = 0; pose < numInputPoses; pose++) {
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

    ofstream dartPoseParamsFile;
    dartPoseParamsFile.open("dartPoseParams.txt");

    dartPoseParamsFile << dartPoseParams;

    dartPoseParamsFile.close();

    // Really Dirty i feel bad about writing this
    //for (int param = 0; param < totalParams; param++) {
    //    allDartPoseParams(pose, param) = dartPoseParams(param);
    //}

    //}


    krang->setPositions(dartPoseParams);

    return krang;

    //return allDartPoseParams;

}



//dart::dynamics::SkeletonPtr createKrang(Eigen::MatrixXd dartPoseParams) {
dart::dynamics::SkeletonPtr createKrangOG() {

    // Instantiate krang
    // There has to be a way to do relative filepaths in DART
    // but until I figure that out I guess this works
    dart::utils::DartLoader loader;
    dart::dynamics::SkeletonPtr krang = loader.parseSkeleton("/home/apatel435/Desktop/09-URDF/Krang/Krang.urdf");
    krang->setName("krang");

    // first three axis-angle (aa)
    // aa1, aa2, aa3, x, y, z, qLWheel, qRWheel, qWaist, qTorso, qKinect,
    // qLArm0, ..., qLArm6, qRArm0, ..., qRArm6

    // Set position of krang
    // krang->setPositions(dartPoseParams);

    return krang;
}

int main(int argc, char* argv[]) {

    // create and initialize the world
    WorldPtr world = World::create();

    // load skeletons
    dart::dynamics::SkeletonPtr floor = createFloor();


    // TODO: Creating a runtime error
    Eigen::MatrixXd allDartPoseParams(2, 25);
    //allDartPoseParams = genDartPoseParams();


    //Initial pose
    //for (int i = 0; i < 25; i++) {
    //    allDartPoseParams
    //}

    //dart::dynamics::SkeletonPtr mKrang = createKrang(allDartPoseParams.row(0));
    dart::dynamics::SkeletonPtr mKrang = createKrang();

    // add ground and robot to the world pointer
    world->addSkeleton(floor);
    world->addSkeleton(mKrang);

    // create a window and link it to the world
    MyWindow window(world);

    glutInit(&argc, argv);
    window.initWindow(960, 720, "Feasible Poses");
    glutMainLoop();

}
