#include <vision/ImageProcessor.h>
#include <vision/BeaconDetector.h>
#include <iostream>

ImageProcessor::ImageProcessor(VisionBlocks& vblocks, const ImageParams& iparams, Camera::Type camera) :
  vblocks_(vblocks), iparams_(iparams), camera_(camera), cmatrix_(iparams_, camera), calibration_(NULL)
{
  enableCalibration_ = false;
  ballFound = false;
  bestBall = new BallCandidate();
  classifier_ = new Classifier(vblocks_, vparams_, iparams_, camera_);
  beacon_detector_ = new BeaconDetector(DETECTOR_PASS_ARGS);
}

void ImageProcessor::init(TextLogger* tl){
  textlogger = tl;
  vparams_.init();
  classifier_->init(tl);
  beacon_detector_->init(tl);
}

unsigned char* ImageProcessor::getImg() {
  if(camera_ == Camera::TOP)
    return vblocks_.image->getImgTop();
  return vblocks_.image->getImgBottom();
}

unsigned char* ImageProcessor::getSegImg(){
  if(camera_ == Camera::TOP)
    return vblocks_.robot_vision->getSegImgTop();
  return vblocks_.robot_vision->getSegImgBottom();
}

unsigned char* ImageProcessor::getColorTable(){
  return color_table_;
}

const CameraMatrix& ImageProcessor::getCameraMatrix(){
  return cmatrix_;
}

void ImageProcessor::updateTransform(){
  BodyPart::Part camera;
  if(camera_ == Camera::TOP)
    camera = BodyPart::top_camera;
  else
    camera = BodyPart::bottom_camera;

  Pose3D pcamera;
  if(enableCalibration_) {
    float joints[NUM_JOINTS], sensors[NUM_SENSORS], dimensions[RobotDimensions::NUM_DIMENSIONS];
    memcpy(joints, vblocks_.joint->values_, NUM_JOINTS * sizeof(float));
    memcpy(sensors, vblocks_.sensor->values_, NUM_SENSORS * sizeof(float));
    memcpy(dimensions, vblocks_.robot_info->dimensions_.values_, RobotDimensions::NUM_DIMENSIONS * sizeof(float));
    Pose3D *rel_parts = vblocks_.body_model->rel_parts_, *abs_parts = vblocks_.body_model->abs_parts_;
    calibration_->applyJoints(joints);
    calibration_->applySensors(sensors);
    calibration_->applyDimensions(dimensions);
    ForwardKinematics::calculateRelativePose(joints, rel_parts, dimensions);
#ifdef TOOL
    Pose3D base = ForwardKinematics::calculateVirtualBase(calibration_->useLeft, rel_parts);
    ForwardKinematics::calculateAbsolutePose(base, rel_parts, abs_parts);
#else
    ForwardKinematics::calculateAbsolutePose(sensors, rel_parts, abs_parts);
#endif
    cmatrix_.setCalibration(*calibration_);
    pcamera = abs_parts[camera];
  }
  else pcamera = vblocks_.body_model->abs_parts_[camera];

  if(vblocks_.robot_state->WO_SELF == WO_TEAM_COACH) {
    auto self = vblocks_.world_object->objects_[vblocks_.robot_state->WO_SELF];
    pcamera.translation.z += self.height;
  }

  cmatrix_.updateCameraPose(pcamera);
}

bool ImageProcessor::isRawImageLoaded() {
  if(camera_ == Camera::TOP)
    return vblocks_.image->img_top_;
  return vblocks_.image->img_bottom_;
}

int ImageProcessor::getImageHeight() {
  return iparams_.height;
}

int ImageProcessor::getImageWidth() {
  return iparams_.width;
}

double ImageProcessor::getCurrentTime() {
  return vblocks_.frame_info->seconds_since_start;
}

void ImageProcessor::setCalibration(RobotCalibration calibration){
  if(calibration_) delete calibration_;
  calibration_ = new RobotCalibration(calibration);
}

bool contain(int id, vector<int>& regionRow){
   for(int i = 0; i < regionRow.size(); ++i){
      if(regionRow[i] == id) return true;
   }
   return false;	
}

int findRow(int id, vector<vector<int>>& regionTable){
   for(int i = 0; i < regionTable.size(); ++i){
      for(int k = 0; k < regionTable[i].size(); ++k){
         if(regionTable[i][k] == id){
	    return i;
	 }
      }	   
   }
   return -1;
}

void sortColor(RunRegion* run, vector<vector<RunRegion*>>& colorClasses){
	switch(run->color){
	   case 1:
		colorClasses[0].push_back(run);
		break;
	   case 2:
		colorClasses[1].push_back(run);
		break;
	   case 3:
		colorClasses[2].push_back(run);
		break;
	   case 4:
		colorClasses[3].push_back(run);
		break;
	   case 5:
		colorClasses[4].push_back(run);
		break;
	   case 6:
		colorClasses[5].push_back(run);
		break;
	   case 7:
		colorClasses[6].push_back(run);
		break;
	   default:
		colorClasses[7].push_back(run);
		break;
	}
}

vector<Blob> UnionRun(vector<RunRegion*>& colorVec){
   vector<vector<int>> regionTable;
   int regionID = 1;
   //child row
   for(int i = 0; i < colorVec.size(); ++i){
      //parent row
      vector<RunRegion*> parentRow;
      for(int k = 0; k < colorVec.size(); ++k){
         if(colorVec[i]->y == (colorVec[k]->y + 2)){ //TODO
            parentRow.push_back(colorVec[k]);			
         }
      }
      //cout<<parentRow.size()<<endl;
      if(parentRow.size() == 0){
         colorVec[i]->regionID = regionID;
         regionTable.push_back(vector<int>(1, regionID)); 
	 ++regionID;
      } else {
	 for(int j = 0; j < parentRow.size(); ++j){
	    //cout<<"ch xs: "<< colorVec[i]->xStart<<"xe: "<<colorVec[i]->xEnd<<endl;
	    //cout<<"pr xs: "<< parentRow[j]->xStart<<"xe: "<<parentRow[j]->xEnd<<endl;
	    if(((colorVec[i]->xStart >= (parentRow[j]->xStart)) && (colorVec[i]->xStart <= parentRow[j]->xEnd)) || 
		((colorVec[i]->xEnd <= (parentRow[j]->xEnd)) && ((colorVec[i]->xEnd >= parentRow[j]->xStart))) || 
		((parentRow[j]->xStart <= (colorVec[i]->xEnd)) && ((parentRow[j]->xStart >= colorVec[i]->xStart)))||
		((parentRow[j]->xEnd <= (colorVec[i]->xEnd)) && ((parentRow[j]->xEnd >= colorVec[i]->xStart)))){
	          if(colorVec[i]->regionID == 0){
		     colorVec[i]->regionID = parentRow[j]->regionID;
		  } else {
			//merge and delete
		     int mergingRow = findRow(colorVec[i]->regionID, regionTable);
		     int deletingRow = findRow(parentRow[j]->regionID, regionTable);
		     if(mergingRow != deletingRow){
		     	regionTable[mergingRow].insert(regionTable[mergingRow].end(), 
				regionTable[deletingRow].begin(), regionTable[deletingRow].end());
		     	regionTable.erase(regionTable.begin() + deletingRow);
		     }

		  }	
	     }		
	 }
	 if(colorVec[i]->regionID == 0){
	    colorVec[i]->regionID = regionID;
            regionTable.push_back(vector<int>(1, regionID)); 
	    ++regionID;
	 }
      }
   }
   //create blobs
   vector<Blob> blobList;
   for(int i = 0; i < regionTable.size(); ++i){
      Blob newBlob; 
      for(int k = 0; k < regionTable[i].size(); ++k){
	for(int a = 0; a < colorVec.size(); ++a){
	   if(colorVec[a]->regionID == regionTable[i][k]){
	      newBlob.runList.push_back(colorVec[a]);
	      newBlob.color = colorVec[a]->color;
	   } 
	}	
       }
       blobList.push_back(newBlob);
   }
   return blobList;
	/*
   cout<<"size: "<<regionTable.size()<<endl;
	for(int i = 0; i < regionTable.size(); ++i){
		cout<<"blob: "<<i<<endl;
		for(int k = 0; k < regionTable[i].size(); ++k){
			cout<<"regionID: "<<regionTable[i][k]<<endl;		
		}
	}
	for(int i = 0; i < colorVec.size(); ++i){
		cout<<"actual ID: "<<colorVec[i]->regionID<<" y: "<<colorVec[i]->y<<" xs: "<<colorVec[i]->xStart<<" xe: "<<colorVec[i]->xEnd<<endl;
	}
	*/
}

void ImageProcessor::createWorldObj(Blob& topBlob, Blob& botBlob){
	//cout<<"GOT HERE:!!!!!!!!!!!!!!!!!!!!"<<endl;		
	WorldObjectType obj;
	int ht = 0;

	if((topBlob.color == 5) && (botBlob.color == 4)){
		obj = WO_BEACON_BLUE_PINK;
		ht = 200;
	}
	if((topBlob.color == 4) && (botBlob.color == 5)){
		obj = WO_BEACON_PINK_BLUE;
		ht = 200;
	}
	if((topBlob.color == 6) && (botBlob.color == 5)){
		obj = WO_BEACON_YELLOW_BLUE;
		ht = 300;
	}
	if((topBlob.color == 5) && (botBlob.color == 6)){
		obj = WO_BEACON_BLUE_YELLOW;
		ht = 300;
	}
	if((topBlob.color == 4) && (botBlob.color == 6)){
		obj = WO_BEACON_PINK_YELLOW;
		ht = 200;
	}
	if((topBlob.color == 6) && (botBlob.color == 4)){
		obj = WO_BEACON_YELLOW_PINK;
		ht = 200;
	}
	auto& object = vblocks_.world_object->objects_[obj];
	object.imageCenterX = (topBlob.xi+topBlob.xf+botBlob.xi+botBlob.xf)/4;
	object.imageCenterY = (topBlob.yi+topBlob.yf+botBlob.yi+botBlob.yf)/4;
	auto position = cmatrix_.getWorldPosition(object.imageCenterX, object.imageCenterY, ht);
	object.visionDistance = cmatrix_.groundDistance(position);
	object.visionBearing = cmatrix_.bearing(position);
	object.seen = true;
	object.fromTopCamera = camera_ == Camera::TOP;
	cout << getName(obj) <<" distance " << object.visionDistance<<endl; 
	visionLog(30, "saw %s at (%i,%i) with calculated distance %2.4f", getName(obj), object.imageCenterX, object.imageCenterY, object.visionDistance);
}

void ImageProcessor::findMatching(vector<vector<Blob>>& allBlobs, int color1, int color2){
  bool overMatch = false;
  bool whiteBot = true;
  for (int i = 0;i<allBlobs[color1].size();i++)
  {
	for (int j = 0;j<allBlobs[color2].size();j++)
	{	
		//cout<<&(allBlobs[nBlue][i])<<endl;
		//cout<<"I am in"<<i<<j<<endl;
		if (matchBlobBeacon(&(allBlobs[color1][i]),&(allBlobs[color2][j])))
		{
			
			//cout<<"GOT HERE:@@@@@@@@@@@@@@@@@"<<endl;
			//cout<<"Match!!!!!!!"<<endl;
			//cout<<"I am in"<<i<<"  @ <<j<<endl;
			Blob topBlob;
			Blob botBlob;
			if(allBlobs[color1][i].yf < allBlobs[color2][j].yf){
				topBlob = allBlobs[color1][i];
				botBlob = allBlobs[color2][j];
			} else {
				topBlob = allBlobs[color2][j];
				botBlob = allBlobs[color1][i];
			}
			if(allBlobs[color1][i].color == 5){
				allBlobs[color1][i].isBeacon = true;
			}
			if(allBlobs[color2][j].color == 5){
				allBlobs[color2][j].isBeacon = true;
			}

			for(int k = 0; k < allBlobs.size(); ++k){
				for(int p = 0; p < allBlobs[k].size(); ++p){
					if((k != 1) && (k != 6)){
						if(!(((allBlobs[k][p].xi == topBlob.xi) && (allBlobs[k][p].yi == topBlob.yi)
						&& (allBlobs[k][p].xf == topBlob.xf) && (allBlobs[k][p].yf == topBlob.yf))
						|| ((allBlobs[k][p].xi == botBlob.xi) && (allBlobs[k][p].yi == botBlob.yi)
						&& (allBlobs[k][p].xf == botBlob.xf) && (allBlobs[k][p].yf == botBlob.yf)))){
							if(relaxedmatchBlobBeacon(&(topBlob), &(allBlobs[k][p]))){
								if(allBlobs[k][p].color == 5){
									allBlobs[k][p].isBeacon = true;
								}
								overMatch = true;
								//cout<<"GOT HERE:#####***************"<<endl;
							}
					  	}
					}	
				}
			}

			int pixCount = 0;
			for(int i = botBlob.yf + 1; i <= botBlob.yf + botBlob.dy; ++i){
				for(int j = botBlob.xi; j <= botBlob.xi + botBlob.dx; ++j){
					if((i < 240) && (j < 320)){
						auto c = getSegImg()[i * iparams_.width + j];
						if(c == c_WHITE){
							++pixCount;
						}
					}
				}
			}
			//cout<<"yf "<< botBlob.yf << "dy "<<botBlob.dy<<"xi "<<botBlob.xi<<"dx "<<botBlob.dx<<"c "<<botBlob.color<<endl;
			//cout<<"pix count "<< pixCount<<endl;
			double whiteDensity = (double) pixCount / (double) ((double) (botBlob.dx/4) * (double) (botBlob.dy/2));
			//cout<<"density "<<whiteDensity<<endl;
			if(whiteDensity < .55){
				whiteBot = false;
				//cout<<"GOT HERE:$$$$$$$$00000$$$$$$$$$$$$$"<<endl;
			}
			if(!overMatch && whiteBot){
				//cout<<"GOT HERE:###################S"<<endl;
				createWorldObj(topBlob, botBlob);
			}
			overMatch = false;
			whiteBot = true;
		}
	}
  }

}
void ImageProcessor::processFrame(){
  if(vblocks_.robot_state->WO_SELF == WO_TEAM_COACH && camera_ == Camera::BOTTOM) return;
  visionLog(30, "Process Frame camera %i", camera_);

  updateTransform();
  
  // Horizon calculation
  visionLog(30, "Calculating horizon line");
  HorizonLine horizon = HorizonLine::generate(iparams_, cmatrix_, 30000);
  vblocks_.robot_vision->horizon = horizon;
  visionLog(30, "Classifying Image", camera_);
  if(!classifier_->classifyImage(color_table_)) return;
  /*int image[6][6] = {{3,3,3,4,4,4},
		     {3,3,3,5,5,5},
		     {2,2,2,3,3,5},
		     {2,2,4,3,3,6},
	 	     {6,3,3,3,6,6},
		     {1,1,1,2,3,3}};
  */
  /*int image[10][10] = {{3,3,1,1,1,1,1,3,3,1},
		       {1,3,3,1,3,3,1,3,1,1},
		       {3,1,3,1,1,3,1,3,1,1},
		       {3,1,3,1,1,1,1,3,3,1},
		       {1,1,3,1,1,1,1,3,1,1},
		       {3,1,3,3,3,1,1,3,3,1},
		       {3,3,1,1,3,3,1,3,1,3},
		       {1,3,1,1,1,3,3,3,1,1},
		       {1,3,3,3,1,1,1,1,1,1},
		       {1,1,1,3,1,1,1,1,1,1}};
*/

/*
  cout<<"aaaaaaaaaaaaaaaaaa";
  std::ofstream output("segImg.txt");
  for (int i = 0;i < 240; i++)
  {
    for (int j = 0;j < 320; j++)
    {
      //cout<<std::to_string((int)getSegImg()[i * iparams_.width + j])<<endl;
      output<<std::to_string((int)getSegImg()[i * iparams_.width + j])<<" ";
    }
  }
  output.close();
*/
  auto fid = vblocks_.frame_info->frame_id;
  //cout<<"processing!"<<fid<<endl;
  

  vector<vector<RunRegion*>> colorClasses;
  for(int i = 0; i < 8; ++i){
     colorClasses.push_back(vector<RunRegion*>(0));
  }
  vector<RunRegion*> allRuns;
  RunRegion* newRun = NULL;
  for(int y = 0; y < 240; y+=2){//TODO
    for(int x = 0; x < 320; x+=4){
      auto c = getSegImg()[y * iparams_.width + x];
      if(x == 0){
	newRun = new RunRegion;
	newRun->color = c;
        newRun->xStart = x;
	newRun->xEnd = x;
	newRun->y = y;
	newRun->length = 1;
	newRun->regionID = 0;
	allRuns.push_back(newRun);
        sortColor(newRun, colorClasses);
      } else if(c == newRun->color){
      	newRun->xEnd = x;
	newRun->length = x - newRun->xStart + 1;
      } else {
	//end of this runlength and start next new run with new color
	newRun->xEnd = x - 1;
	newRun->length = x - newRun->xStart;
	newRun = new RunRegion;
	newRun->color = c;
        newRun->xStart = x;
	newRun->xEnd = x;
	newRun->y = y;
	newRun->length = 1;
	newRun->regionID = 0;
	allRuns.push_back(newRun);
        sortColor(newRun, colorClasses);
      }
    }
  }
/*
  cout<<allRuns.size()<<endl;
  for(int i = 0; i < colorClasses.size(); ++i){
	cout<<"i "<<i<<" s "<<colorClasses[i].size()<<endl;
  }
*/  
  //NOT ENUMMMMMM
  //green 0
  //white 1
  //orange 2
  //pink 3
  //blue 4
  //yellow 5
  //rwhite 6
  //undef 7

  //union
  vector<vector<Blob>> allBlobs;
  allBlobs.push_back(UnionRun(colorClasses[0]));
  allBlobs.push_back(UnionRun(colorClasses[1]));
  allBlobs.push_back(UnionRun(colorClasses[2]));
  allBlobs.push_back(UnionRun(colorClasses[3]));
  allBlobs.push_back(UnionRun(colorClasses[4]));
  allBlobs.push_back(UnionRun(colorClasses[5]));
  allBlobs.push_back(UnionRun(colorClasses[6]));
  /*
 for(int i = 0; i < allBlobs[4].size(); ++i){
        cout<<"blob "<<i;
	for(int j = 0; j < allBlobs[4][i].runList.size(); ++j){
	   cout<<" length: "<<allBlobs[4][i].runList[j]->length<<" color: "<<allBlobs[4][i].runList[j]->color<<" x beg: "<<allBlobs[4][i].runList[j]->xStart<<" x end: "<<allBlobs[4][i].runList[j]->xEnd<<" y:"<<allBlobs[4][i].runList[j]->y<<endl;
	}
  }
*/
  //beacon_detector_->findBeacons();
/*
  for(int i = 0; i < allBlobs.size(); ++i){
     cout<<i<<" "<<allBlobs[i].size()<<endl;
  }
*/


  /*char buffer[200];
  sprintf(buffer, "segImg_%d.txt",fid);
  std::ofstream output(buffer);
  for (int i = 0;i < 240; i++)
  {
    for (int j = 0;j < 320; j++)
    {
      //cout<<std::to_string((int)getSegImg()[i * iparams_.width + j])<<endl;
      output<<std::to_string((int)getSegImg()[i * iparams_.width + j])<<" ";
    }
  }
  output.close();

  int colorMap[320][240];
  int blobMap[320][240];
  int blobCount = 0;
  
  for (int i = 0;i<320;i++)
  {
	for (int j = 0;j<240;j++)
	{
		blobMap[i][j] = -1;
		colorMap[i][j] = -1;
	}
  }
  
  for (int i = 0;i < allBlobs.size();i++)
  {
	for (int j = 0;j < allBlobs[i].size();j++)
	{
		blobCount++;
		for (int k = 0;k < allBlobs[i][j].runList.size();k++)
		{
			for (int p = allBlobs[i][j].runList[k]->xStart;p < allBlobs[i][j].runList[k]->xEnd; p++)
			{
				blobMap[p][allBlobs[i][j].runList[k]->y] = blobCount;
				colorMap[p][allBlobs[i][j].runList[k]->y] = allBlobs[i][j].color;
				if (allBlobs[i][j].color!=allBlobs[i][j].runList[k]->color)
				{
					cout<<"Error!!!Blob color is not the same as run color"<<endl;
				}
			}
		}
	}
  }
  
  sprintf(buffer, "BlobIndex_%d.txt",fid);
  std::ofstream outputBlob(buffer);
  for (int i = 0;i < 240; i++)
  {
    for (int j = 0;j < 320; j++)
    {
      //cout<<std::to_string((int)getSegImg()[i * iparams_.width + j])<<endl;
      outputBlob<<std::to_string(blobMap[j][i])<<" ";
    }
  }
  outputBlob.close();
  
  sprintf(buffer, "BlobColor_%d.txt",fid);
  std::ofstream outputColor(buffer);
  for (int i = 0;i < 240; i++)
  {
    for (int j = 0;j < 320; j++)
    {
      //cout<<std::to_string((int)getSegImg()[i * iparams_.width + j])<<endl;
      outputColor<<std::to_string(colorMap[j][i])<<" ";
    }
  }
  outputColor.close();*/
  
  std::vector<RunRegion*> runListVec;
  RunRegion* regionTemp;
  int xmin, xmax, ymin, ymax, pixelCount, xcount, ycount;
  for (int k = 0;k < allBlobs.size();k++)
  {
	  for(int i = 0; i < allBlobs[k].size(); i++)
	  {
		xcount = 0;
		ycount = 0;
		pixelCount = 0;
		xmin = 1000;
		xmax = 0;
		ymin = 1000;
		ymax = 0;
		runListVec = allBlobs[k][i].runList;
		for (int j = 0;j < runListVec.size(); j++)
		{
			regionTemp = runListVec[j];
			xmin = (xmin<regionTemp->xStart)?xmin:regionTemp->xStart;
			xmax = (xmax>regionTemp->xEnd)?xmax:regionTemp->xEnd;
			ymin = (ymin<regionTemp->y)?ymin:regionTemp->y;
			ymax = (ymax>regionTemp->y)?ymax:regionTemp->y;
			xcount += ((regionTemp->xStart + regionTemp->xEnd)*regionTemp->length)/2;
			ycount += regionTemp->y*regionTemp->length;
			pixelCount += regionTemp->length;
		}
		allBlobs[k][i].xi = xmin;
		allBlobs[k][i].xf = xmax;
		allBlobs[k][i].yi = ymin;
		allBlobs[k][i].yf = ymax;
		allBlobs[k][i].dx = allBlobs[k][i].xf - allBlobs[k][i].xi + 1;
		allBlobs[k][i].dy = allBlobs[k][i].yf - allBlobs[k][i].yi + 1;
		allBlobs[k][i].lpCount = pixelCount;
		allBlobs[k][i].avgX = xcount/pixelCount;
		allBlobs[k][i].avgY = ycount/pixelCount;
	  }
  }
  findMatching(allBlobs, 4, 3);
  findMatching(allBlobs, 5, 4);
  findMatching(allBlobs, 3, 5);

  int nOrange = 2;
  ballFound = false;
  for (int i = 0;i < allBlobs[nOrange].size();i++)
  {
	if (detectBall(allBlobs[nOrange][i]))
	{
		//cout<<"Yes YEs YEs"<<endl;
		ballFound = true;
		WorldObject* myobject = &vblocks_.world_object->objects_[WO_BALL];
		myobject->imageCenterX = allBlobs[nOrange][i].avgX;
		myobject->imageCenterY = allBlobs[nOrange][i].avgY;
		auto position = cmatrix_.getWorldPosition(myobject->imageCenterX, myobject->imageCenterY, 33);
		myobject->radius = (allBlobs[nOrange][i].dx + allBlobs[nOrange][i].dy)/2;
		myobject->visionDistance = cmatrix_.groundDistance(position);
		myobject->visionBearing = cmatrix_.bearing(position);
		myobject->seen = true;
		myobject->fromTopCamera = camera_ == Camera::TOP;
		visionLog(30, "saw %s at (%i,%i) with calculated distance %2.4f", "ball", myobject->imageCenterX, myobject->imageCenterY, myobject->visionDistance);
		
		//auto position = cmatrix_.getWorldPosition(object.imageCenterX, object.imageCenterY, 33);
		bestBall->centerX = allBlobs[nOrange][i].avgX;;
		bestBall->centerY = allBlobs[nOrange][i].avgY;
		bestBall->radius = (allBlobs[nOrange][i].dx + allBlobs[nOrange][i].dy)/2;
		bestBall->width = allBlobs[nOrange][i].dx;
		bestBall->height = allBlobs[nOrange][i].dy;
		bestBall->groundDistance = cmatrix_.groundDistance(position);
		bestBall->blob = &(allBlobs[nOrange][i]);
		bestBall->valid = true;
		cout<<"ball "<<"distance:"<<bestBall->groundDistance<<endl;
	}
  }

  int nblue = 4;
  for (int i = 0;i < allBlobs[nblue].size();i++)
  {
	if (detectGoal(allBlobs[nblue][i]))
	{
		for(int p = allBlobs[nblue][i].xi; p < allBlobs[nblue][i].xf+1 ; p++)
		{
			for (int q = allBlobs[nblue][i].yi;q < allBlobs[nblue][i].yf+1; q++)
			{
				getSegImg()[q * iparams_.width + p] = c_UNDEFINED;
			}	
		}
		WorldObject* myobject = &vblocks_.world_object->objects_[WO_OWN_GOAL];
		myobject->imageCenterX = allBlobs[nblue][i].avgX;
		myobject->imageCenterY = allBlobs[nblue][i].avgY;
		auto position = cmatrix_.getWorldPosition(myobject->imageCenterX, myobject->imageCenterY, 250);
		myobject->visionDistance = cmatrix_.groundDistance(position);
		myobject->visionBearing = cmatrix_.bearing(position);
		myobject->seen = true;
		myobject->fromTopCamera = camera_ == Camera::TOP;
		cout << "Goal Distance: " << myobject->visionDistance <<endl;		
//visionLog(30, "saw %s at (%i,%i) with calculated distance %2.4f", "ball", myobject->imageCenterX, myobject->imageCenterY, myobject->visionDistance);
		
	}
  }

  for(int i = 0; i < allRuns.size(); ++i){
	delete allRuns[i];
  }

  

}

bool ImageProcessor::detectBall(Blob& orangeBlob) 
{
	double densityMin = 0.3;
	double densityMax = 0.8;
	double ratioMin = 0.67;
	double ratioMax = 1.5;
	double pixelCountMin = 50;
	double pixelCountMax = 10000;
	double sigmaSquareMax = 10;
	//double gDistanceMin = 0;
	//double gDistanceMax = 6000000000;
	double gDistanceMin = 500;
	double gDistanceMax = 6000;
	/*double densityMin = 0;
	double densityMax = 1;
	double ratioMin = 0.01;
	double ratioMax = 90;
	double pixelCountMin = 0;
	double pixelCountMax = 10000000;
	double sigmaSquareMax = 100000000;
	double gDistanceMin = 0;
	double gDistanceMax = 6000000000;*/
	/*cout<<"starting"<<endl;
	for(int j = 0; j < orangeBlob.runList.size(); ++j){
	   cout<<" length: "<<orangeBlob.runList[j]->length<<" color: "<<orangeBlob.runList[j]->color<<" x beg: "<<orangeBlob.runList[j]->xStart<<" x end: "<<orangeBlob.runList[j]->xEnd<<" y:"<<orangeBlob.runList[j]->y<<endl;
	}*/

	double density = ((double)(orangeBlob.lpCount))/(orangeBlob.dx*orangeBlob.dy);
	double ratio = ((double)(orangeBlob.dx))/orangeBlob.dy;
	
	//cout<<"Blob Color!!!!"<<orangeBlob.color<<endl;
	if ((orangeBlob.lpCount<pixelCountMin)||(orangeBlob.lpCount>pixelCountMax))
	{
		//cout<<"PixelCount fail"<<endl;
		return false;
	}
	if ((ratio<ratioMin)||(ratio>ratioMax))
	{
		//cout<<"Ratio fail"<<endl;
		return false;
	}
	if ((density<densityMin)||(density>densityMax))
	{
		//cout<<"Desnsity fail"<<density<<endl;
		return false;
	}

	Position p = cmatrix_.getWorldPosition(orangeBlob.avgX, orangeBlob.avgY, 33);
	double gDistance = cmatrix_.groundDistance(p);
	if ((gDistance<gDistanceMin)||(gDistance>gDistanceMax))
	{
		//cout<<"Distance fail"<<endl;
		return false;
	}
	
	int height = orangeBlob.dy;
	int leftX[250];
	int rightX[250];
	vector<double> distance;
	int runY, runXStart, runXEnd;
	int dx, dy;
	for (int i=0;i<height;i++)
	{
		rightX[i] = -1;
		leftX[i] = 1000;
	}
	for (int i = 0;i < orangeBlob.runList.size(); i++)
	{
		runY = orangeBlob.runList[i]->y;
		runXStart = orangeBlob.runList[i]->xStart;
		runXEnd = orangeBlob.runList[i]->xEnd;
		if (runXStart<leftX[runY-orangeBlob.yi])
		{
			leftX[runY-orangeBlob.yi] = runXStart;
		}
		if (runXEnd>rightX[runY-orangeBlob.yi])
		{
			rightX[runY-orangeBlob.yi] = runXEnd;
		}
	}
	for (int i=0;i<height;i++)
	{
		if ((rightX[i]!=-1)&&(leftX[i]!=1000))
		{
			dx = leftX[i] - orangeBlob.avgX;
			dy = (i+orangeBlob.yi) - orangeBlob.avgY;
			distance.push_back(sqrt(dx*dx+dy*dy));
			if (leftX[i]!=rightX[i])
			{
				dx = rightX[i] - orangeBlob.avgX;
				distance.push_back(sqrt(dx*dx+dy*dy));
			}			
		}
	}
	double sum = 0;
	for (int i = 0;i < distance.size(); i++)
	{
		sum += distance[i];
	}
	double average = sum/distance.size();
	sum = 0;
	for (int i = 0;i < distance.size(); i++)
	{
		sum += (distance[i]-average)*(distance[i]-average);
	}
	double sigma_square = sum/distance.size();
	if (sigma_square > sigmaSquareMax)
	{
		//cout<<"sigma fail"<<endl;
		return false;
	}
	/*cout<<"lpcount "<<orangeBlob.lpCount<<endl;
	cout<<"dx "<<orangeBlob.dx<<endl;
	cout<<"dy "<<orangeBlob.dy<<endl;
	cout<<"xi "<<orangeBlob.xi<<endl;
	cout<<"xf "<<orangeBlob.xf<<endl;
	cout<<"yi "<<orangeBlob.yi<<endl;
	cout<<"yf "<<orangeBlob.yf<<endl;*/
	//cout<<"lpcount"<<orangeBlob.lpCount<<endl;
	//cout<<"lpcount"<<orangeBlob.lpCount<<endl;
	//cout<<"lpcount"<<orangeBlob.lpCount<<endl;
	return true;
	
}

void ImageProcessor::findBall(int& imageX, int& imageY) {
  imageX = imageY = 0;
}

int ImageProcessor::getTeamColor() {
  return vblocks_.robot_state->team_;
}

void ImageProcessor::SetColorTable(unsigned char* table) {
  color_table_ = table;
}

float ImageProcessor::getHeadChange() const {
  if (vblocks_.joint == NULL)
    return 0;
  return vblocks_.joint->getJointDelta(HeadPan);
}

std::vector<BallCandidate*> ImageProcessor::getBallCandidates() {
  /*cout<<"Enter getBestBallCandidate"<<endl;
	bestBall->centerX = 100;
		bestBall->centerY = 100;
		bestBall->radius = 10;
		bestBall->width = 10;
		bestBall->height = 10;
		bestBall->groundDistance = 600;
		//bestBall->blob = &(allBlobs[nOrange][i]);
		bestBall->valid = true;
	return bestBall;*/
  //cout<<"Enter getBallCandidates"<<endl;
  return std::vector<BallCandidate*>();
}

BallCandidate* ImageProcessor::getBestBallCandidate() {
	//cout<<"Enter getBestBallCandidate"<<endl;
	/*bestBall->centerX = 100;
		bestBall->centerY = 100;
		bestBall->radius = 10;
		bestBall->width = 10;
		bestBall->height = 10;
		bestBall->groundDistance = 600;
		//bestBall->blob = &(allBlobs[nOrange][i]);
		bestBall->valid = true;
	return bestBall;*/
	if (ballFound)
	{
		return bestBall;

	}
	else
	{
		return NULL;
	}	
}

void ImageProcessor::enableCalibration(bool value) {
  enableCalibration_ = value;
}

bool ImageProcessor::isImageLoaded() {
  return vblocks_.image->loaded_;
}

bool ImageProcessor::detectGoal(Blob& blueBlob) 
{
	//cout<<"Enter Detect Goal!"<<endl;
	if (blueBlob.isBeacon)
	{
		return false;
	}
	//cout<<"Not a beacon"<<endl;
	double densityMin = 0.25;
	double densityMax = 1.4;
	double pixelCountMin = 500;
	double pixelCountMax = 1000000;
	double ratioMin = 1;
	double ratioMax = 2.5;
	double density = ((double)(blueBlob.lpCount))/(blueBlob.dx*blueBlob.dy);
	double ratio = ((double)(blueBlob.dx))/blueBlob.dy;
	
	if ((blueBlob.lpCount<pixelCountMin)||(blueBlob.lpCount>pixelCountMax))
	{
		return false;
	}
	if ((ratio<ratioMin)||(ratio>ratioMax))
	{
		return false;
	}
	//cout << "density for goal: "<<density<<endl;
	if ((density<densityMin)||(density>densityMax))
	{
		return false;
	}
	return true;
}
