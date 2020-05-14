#include "ofApp.h"

/*
    If you are struggling to get the device to connect ( especially Windows Users )
    please look at the ReadMe: in addons/ofxKinect/README.md
*/

//--------------------------------------------------------------
void ofApp::setup() {
	ofSetLogLevel(OF_LOG_VERBOSE);
    ofHideCursor();
    //Animation-code start
	  ofSetFullscreen(true);
    ofSetBackgroundColor(255);
       ofNoFill();
       side = ofGetWidth()/n;
       
       pos= {ofRandomWidth()/2, ofRandomHeight()/2};
       vel = {0.0,0.0};
       acc = {0.0,0.0};
       offset = {ofRandom(1000), ofRandom(1000)};
    
    //Animation-code end
    
    
	// enable depth->video image calibration
	kinect.setRegistration(true);
    kinect.init();
	//kinect.init(true); // shows infrared instead of RGB video image
	//kinect.init(false, false); // disable video image (faster fps)
	kinect.open();		// opens first available kinect
	//kinect.open(1);	// open a kinect by id, starting with 0 (sorted by serial # lexicographically))
	//kinect.open("A00362A08602047A");	// open a kinect using it's unique serial #
	if(kinect.isConnected()) {
		ofLogNotice() << "sensor-emitter dist: " << kinect.getSensorEmitterDistance() << "cm";
		ofLogNotice() << "sensor-camera dist:  " << kinect.getSensorCameraDistance() << "cm";
		ofLogNotice() << "zero plane pixel size: " << kinect.getZeroPlanePixelSize() << "mm";
		ofLogNotice() << "zero plane dist: " << kinect.getZeroPlaneDistance() << "mm";
	}
	

	
	colorImg.allocate(kinect.width, kinect.height);
	grayImage.allocate(kinect.width, kinect.height);
	grayThreshNear.allocate(kinect.width, kinect.height);
	grayThreshFar.allocate(kinect.width, kinect.height);
    
    
	
	nearThreshold = 255;  //Tää toimii sillee oudosti että 160cm on se takaseinä, 255 on lähin arvo                        mikä
	farThreshold = 230;   //voi näkyä. Eli 255-160 välillä sä näyt ruudussa.
	bThreshWithOpenCV = true;
	
	ofSetFrameRate(60);
	
	// zero the tilt on startup
	angle = 0;
	kinect.setCameraTiltAngle(angle);
	
	// start from the front
	bDrawPointCloud = false;
}

//--------------------------------------------------------------
void ofApp::update() {
	
	
  
	kinect.update();
	
	// there is a new frame and we are connected
	if(kinect.isFrameNew()) {
		
		// load grayscale depth image from the kinect source
		grayImage.setFromPixels(kinect.getDepthPixels());
		
		// we do two thresholds - one for the far plane and one for the near plane
		// we then do a cvAnd to get the pixels which are a union of the two thresholds
		if(bThreshWithOpenCV) {
			grayThreshNear = grayImage;
			grayThreshFar = grayImage;
			grayThreshNear.threshold(nearThreshold, true);
			grayThreshFar.threshold(farThreshold);
			cvAnd(grayThreshNear.getCvImage(), grayThreshFar.getCvImage(), grayImage.getCvImage(), NULL);
		} else {
			
			// or we do it ourselves - show people how they can work with the pixels
			ofPixels & pix = grayImage.getPixels();
			int numPixels = pix.size();
			for(int i = 0; i < numPixels; i++) {
				if(pix[i] < nearThreshold && pix[i] > farThreshold) {
					pix[i] = 255;
				} else {
					pix[i] = 0;
				}
			}
		}
		
		// update the cv images
		grayImage.flagImageChanged();

		contourFinder.findContours(grayImage, 10, (kinect.width*kinect.height)/2, 20, false);
     
	}
//Animation code
     time = ofGetElapsedTimef()*0.01;
      noiseRadius = 2;
     
     glm::vec3 vecX = {0, noiseRadius*cos(TWO_PI*t), noiseRadius*sin(TWO_PI*t)};
     glm::vec3 vecY = {1000, noiseRadius*cos(TWO_PI*t), noiseRadius*sin(TWO_PI*t)};
     float myNoiseX = glm::simplex(vecX);
     float myNoiseY = glm::simplex(vecY);
     
     noise= {myNoiseX,myNoiseY};
     
     //Matti's target code
     target = {targetX,targetY};
     target = target-pos;
     
     glm::normalize(target);
     
     target*=0.001;
     
     acc=target;
     vel=vel+acc;
     pos = pos+vel;
     pos = pos+noise;
     

     if(glm::length(vel)>5.0){
         vel = glm::normalize(vel)*5.0;
     }
    for(int i = 0; i < contourFinder.nBlobs; i++) {
       
            glm::vec2 center = contourFinder.blobs.at(i).centroid;
        
      
        cenX = center.x;
        cenY = center.y;
  
     }
                                
        
    if (contourFinder.nBlobs<1){
        targetX = ofGetWidth()/2;
        targetY =ofGetHeight()/2;
        
     
         }else {
         // targetX = mouseX;
         //   targetY = mouseY;
            
             targetX = ofMap(cenX,0,640,ofGetWidth(),0);
      targetY =  ofMap(cenY,0,480,0,ofGetHeight());
       
             cout<<cenX<<endl;
             cout<<cenY<<endl;
            
        
    
           
       }
     xb = pos.x;
     yb = pos.y;
               
    cout<<"fps: "<<ofGetFrameRate()<<endl;

}

//--------------------------------------------------------------
void ofApp::draw() {
    
   //START OF PROPER
  
   for(float x=0;x<n;x++){
              for(int y=0;y<n;y++){
                  
             
                  
               //   glm::vec4 vecR = {};
                  
              //    float myNoiseR = glm::simplex(vecR);
                  
                  radius = 100*(float)ofNoise(x*side*0.01, y*side*0.01, noiseRadius*cos(TWO_PI*t), noiseRadius*sin(TWO_PI*t))+100;
                  
                   ofSetColor(0);
                   ofSetLineWidth(2);
                  
                //  ofPushMatrix();
                //  ofTranslate(ofGetWidth(), 0);
                //  ofScale(-1, 1);
             //       contourFinder.draw(0, 0, ofGetWidth(), ofGetHeight());
              //    ofPopMatrix();
                 
                  if (ofDist(x*side, y*side, xb, yb)<radius){
             ofDrawLine(x*side,y*side, (x+1)*side,(y+1)*side);
                   
         } else{
             ofDrawLine((x+1)*side, y*side, x*side, (y+1)*side);
         }
           
       }
           }

}

void ofApp::drawPointCloud() {
   
}

//--------------------------------------------------------------
void ofApp::exit() {
	kinect.setCameraTiltAngle(0); // zero the tilt on exit
	kinect.close();
	

}

//--------------------------------------------------------------
void ofApp::keyPressed (int key) {
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button)
{
	
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button)
{

}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button)
{

}

//--------------------------------------------------------------
void ofApp::mouseEntered(int x, int y){

}

//--------------------------------------------------------------
void ofApp::mouseExited(int x, int y){

}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h)
{

}
