#ifndef BLOB_H
#define BLOB_H

#include <constants/VisionConstants.h>
#include <vector>
#include <inttypes.h>

/// @ingroup vision

struct RunRegion{
	int color;
	int xStart;
	int xEnd;
	int y;
	int length;
	int regionID;
};

struct Blob {
  uint16_t xi, xf, dx, yi, yf, dy;
  uint16_t lpCount;
  //******
  std::vector<RunRegion*> runList;
  std::vector<uint16_t> lpIndex;
  int color;
  //******
  float diffStart;
  float diffEnd;
  float doubleDiff;
  uint16_t widthStart;
  uint16_t widthEnd;
  uint16_t avgX;
  uint16_t avgY;
  float avgWidth;
  float correctPixelRatio;
  bool invalid;
  bool isBeacon;
  // GOAL DETECTION
  int edgeSize;
  int edgeStrength;

  Blob() : lpIndex(MAX_BLOB_VISIONPOINTS, 0), isBeacon(false) { }
};

/// @ingroup vision
bool sortBlobAreaPredicate(Blob* left, Blob* right);
bool matchBlobBeacon(Blob* a, Blob* b);
bool relaxedmatchBlobBeacon(Blob* a, Blob* b);

#endif
