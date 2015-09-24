#ifndef IMAGEPROCESSOR_H
#define IMAGEPROCESSOR_H

#include <kinematics/ForwardKinematics.h>
#include <common/RobotDimensions.h>
#include <common/Profiling.h>
#include <memory/TextLogger.h>
#include <vision/CameraMatrix.h>
#include <vision/VisionBlocks.h>
#include <common/RobotInfo.h>
#include <vision/Classifier.h>
#include <common/RobotCalibration.h>
#include <vision/structures/BallCandidate.h>
#include <math/Pose3D.h>
#include <math.h>

class BeaconDetector;

struct BoundingBox{
	BoundingBox* prevBox;
	BoundingBox* nextBox;
	int ULx;
	int ULy;
	int LRx;
	int LRy;
	bool lastBox;
	int valid;
	int numRunLength;
	int numPixels;
	int rrcount;
	int color;
	RunRegion* listRR;
	RunRegion* eoList;
	float prob;
};

/// @ingroup vision
class ImageProcessor {
  public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW  
    ImageProcessor(VisionBlocks& vblocks, const ImageParams& iparams, Camera::Type camera);
    void processFrame();
    void createWorldObj(Blob& topBlob, Blob& botBlob);
    void findMatching(vector<vector<Blob>>& allBlobs, int color1, int color2);
    void init(TextLogger*);
    void SetColorTable(unsigned char*);
    Classifier* classifier_;
    unsigned char* getImg();
    unsigned char* getSegImg();
    unsigned char* getColorTable();
    bool isRawImageLoaded();
    int getImageHeight();
    int getImageWidth();
    const ImageParams& getImageParams() const { return iparams_; }
    const CameraMatrix& getCameraMatrix();
    void setCalibration(RobotCalibration);
    void enableCalibration(bool value);
    void updateTransform();
    std::vector<BallCandidate*> getBallCandidates();
    BallCandidate* getBestBallCandidate();
    bool isImageLoaded();
    bool detectBall(Blob& orangeBlob);
    bool detectGoal(Blob& blueBlob);
    void findBall(int& imageX, int& imageY);
	//***********************************
    void find();
    void regionUnion();
  private:
	//***********************************
    int getTeamColor();
    double getCurrentTime();

    VisionBlocks& vblocks_;
    const ImageParams& iparams_;
    Camera::Type camera_;
    CameraMatrix cmatrix_;
    
    VisionParams vparams_;
    unsigned char* color_table_;
    TextLogger* textlogger;

    float getHeadPan() const;
    float getHeadTilt() const;
    float getHeadChange() const;

    RobotCalibration* calibration_;
    bool enableCalibration_;
    BeaconDetector* beacon_detector_;
    bool ballFound;

    BallCandidate* bestBall;
};

#endif
