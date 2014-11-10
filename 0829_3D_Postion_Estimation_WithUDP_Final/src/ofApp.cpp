#include "ofApp.h"

//--------------------------------------------------------------
void ofApp::setup(){
    
    camWidth 		= 640;	// try to grab at this size.
	camHeight 		= 480;
    
    //画面の基本設定
    ofBackground(100, 100, 100);
    ofEnableAlphaBlending();
    ofSetFrameRate(60);
    
    //Set the window to be the width of 3 cameras
    ofSetWindowShape(2 * camWidth, camHeight);
    //Get back a list of devices.
	vector<ofVideoDevice> devices = vidGrabber[0].listDevices();
    //vector<ofVideoDevice> devices = vidGrabber[1].listDevices();
	
    for(int i = 0; i < devices.size(); i++){
		cout << devices[i].id << ": " << devices[i].deviceName;
        if( devices[i].bAvailable ){
            cout << " - available " << endl;
        }else{
            cout << " - unavailable " << endl;
        }
	}
    
    vidGrabber[0].setDeviceID(1);//USB_Camera #3
    vidGrabber[1].setDeviceID(2);//USB_Camera #4
    vidGrabber[0].initGrabber(camWidth,camHeight);
    vidGrabber[1].initGrabber(camWidth,camHeight);
    
    ofSetVerticalSync(true);
    
    //使用する画像の領域を確保
    //InvertHomoImg.allocate(camWidth, camHeight);
    colorImg[0].allocate(camWidth,camHeight);
    colorImg[1].allocate(camWidth,camHeight);
    ColorHomography[0].allocate(camWidth, camHeight);
    ColorHomography[1].allocate(camWidth, camHeight);
    grayImage[0].allocate(camWidth,camHeight);
    grayImage[1].allocate(camWidth,camHeight);
    grayBg.allocate(camWidth,camHeight);
    grayDiff[0].allocate(camWidth,camHeight);
    grayDiff[1].allocate(camWidth,camHeight);
    
    //変数の初期化
    threshold = 200;
    
    
    //Initialization for Matrices
    CameraIntrinsic[0] = cvCreateMat(3, 3, CV_64FC1);
    CameraIntrinsic[1] = cvCreateMat(3, 3, CV_64FC1);
    Homography_Camera[0] = cvCreateMat( 3, 3, CV_64FC1 );
    Homography_Camera[1] = cvCreateMat( 3, 3, CV_64FC1 );
    T[0] = cvCreateMat(4, 4, CV_64FC1);
    T[1] = cvCreateMat(4, 4, CV_64FC1);
    RVec = cvCreateMat(3, 1, CV_64FC1);
    TVec = cvCreateMat(3, 1, CV_64FC1);
    DVec[0] = cvCreateMat(5, 1, CV_64FC1);
    DVec[1] = cvCreateMat(5, 1, CV_64FC1);
    
    
    Camera_Origin[0] = cvCreateMat(3, 1, CV_64FC1);
    Camera_Origin[1] = cvCreateMat(3, 1, CV_64FC1);
    RVec_World[0] = cvCreateMat(3,1,CV_64FC1);
    RVec_World[1] = cvCreateMat(3,1,CV_64FC1);
    Edge_A = cvCreateMat(3, 1, CV_64FC1);
    LED_Pos3D = cvCreateMat(3, 1, CV_64FC1);
    
    P[1] = cvCreateMat(3, 1, CV_64FC1);
    P[2] = cvCreateMat(3, 1, CV_64FC1);
    P[3] = cvCreateMat(3, 1, CV_64FC1);
    P[4] = cvCreateMat(3, 1, CV_64FC1);
    
    P_opt[0] = cvCreateMat(3, 1, CV_64FC1);
    P_opt[1] = cvCreateMat(3, 1, CV_64FC1);
    
    //为各自的InstrinsicMatrix赋上数据
    cvInitMatHeader(CameraIntrinsic[0], 3, 3, CV_64FC1,intrinsic[0]);
    cvInitMatHeader(CameraIntrinsic[1], 3, 3, CV_64FC1,intrinsic[1]);
    
    //为各自的Distortion Vector赋上数据
    cvInitMatHeader(DVec[0], 5, 1, CV_64FC1 ,distortionVector[0]);
    cvInitMatHeader(DVec[1], 5, 1, CV_64FC1 ,distortionVector[1]);
    
    //Initialize UDP Sender
    sender.setup(HOST, PORT);
    
}

//--------------------------------------------------------------
void ofApp::update(){
    //カメラから新規フレーム取り込み
    //每有一帧新的图像时
    //两个相机中的Blob一直都会被检测到,并且存入各自的blobX,blobY中
    //各自的Blob大小可能需要各自调节下
    vidGrabber[0].update();
    vidGrabber[1].update();
    
    //フレームが切り替わった際のみ画像を解析
    if (vidGrabber[0].isFrameNew()){ // for camera 0
        
        colorImg[0].setFromPixels(vidGrabber[0].getPixels(), camWidth,camHeight);
        
        
        //尝试在转化后的图像上找Blob以排除在垂直于平面上的移动对原图像上的影响
        //cvWarpPerspective(colorImg[0].getCvImage(),ColorHomography[0].getCvImage(),Homography_Camera[0]);
        
        
        //左右反転
        //colorImg[0].mirror(false, true);
        
        //カラーのイメージをグレースケールに変換
        grayImage[0] = colorImg[0];
        
        //グレースケールのイメージと取り込んだ背景画像との差分を算出
        grayDiff[0].absDiff(grayBg, grayImage[0]);
        
        //画像を2値化(白と黒だけに)する
        grayDiff[0].threshold(threshold);
        
        /*
         grayImage[0] = ColorHomography[0];
         grayDiff[0].absDiff(grayBg, grayImage[0]);
         grayDiff[0].threshold(threshold);
         */
        //0828 Change the Min Blob Size to 1
        contourFinder[0].findContours(grayDiff[0], 1, grayDiff[0].width * grayDiff[0].height, 1, false, false);
        //读取新追踪到的Blob作为追踪到的LED的位置
        for (int i = 0; i < contourFinder[0].nBlobs; i++){
            
            blobX[0]= contourFinder[0].blobs[i].centroid.x;//centroid is the middle of the shape
            blobY[0] = contourFinder[0].blobs[i].centroid.y;
            
        }
    }
    if (vidGrabber[1].isFrameNew()){ // for camera 1
        
        colorImg[1].setFromPixels(vidGrabber[1].getPixels(), camWidth,camHeight);
        
        
        //尝试在转化后的图像上找Blob以排除在垂直于平面上的移动对原图像上的影响
        //cvWarpPerspective(colorImg[1].getCvImage(),ColorHomography[1].getCvImage(),Homography_Camera[1]);
        
        //左右反転
        //colorImg[1].mirror(false, true);
        
        //カラーのイメージをグレースケールに変換
        grayImage[1] = colorImg[1];
        
        //グレースケールのイメージと取り込んだ背景画像との差分を算出
        grayDiff[1].absDiff(grayBg, grayImage[1]);
        
        //画像を2値化(白と黒だけに)する
        grayDiff[1].threshold(threshold);
        
        /*
         grayImage[1] = ColorHomography[1];
         grayDiff[1].absDiff(grayBg, grayImage[1]);
         grayDiff[1].threshold(threshold);
         */
        // 20为最小blob size
        // 1为最大检测个数
        //0828 Change the Min Blob Size to 1
        contourFinder[1].findContours(grayDiff[1], 1, grayDiff[1].width * grayDiff[1].height, 1, false, false);
        
        for (int i = 0; i < contourFinder[1].nBlobs; i++){
            
            blobX[1]= contourFinder[1].blobs[i].centroid.x;//centroid is the middle of the shape
            blobY[1] = contourFinder[1].blobs[i].centroid.y;
        }
    }
}

//--------------------------------------------------------------
void ofApp::draw(){
    
    // Apply the Homography Matrix
    //cvWarpPerspective(colorImg[0].getCvImage(),ColorHomography[0].getCvImage(),Homography_Camera[0]);
    //cvWarpPerspective(colorImg[1].getCvImage(),ColorHomography[1].getCvImage(),Homography_Camera[1]);
    
    
    colorImg[0].draw(0, 0, camWidth, camHeight);
    colorImg[1].draw(camWidth, 0, camWidth, camHeight);
    
    
    ofPushMatrix();
    ofSetColor(255, 0, 0);
    ofFill();
    ofEllipse(blobX[0], blobY[0], 4, 4);
    ofEllipse(blobX[1]+camWidth, blobY[1], 4, 4);//
    ofPopMatrix();
    
    
    
    //两个方向的相机都计算好Homography之后，计算被追踪的坐标点。
    //其中，Camera[0]的X和Y用来计算水平方向的X和Z
    //Camera[1]的Y用来计算竖直方向的Y
    //Tracked_X
    //Tracked_Y
    //Tracked_Z
    //是新检测出来的点的世界坐标
    //注意Blob从0开始刷新. blob[0],blob[1],blob[2].....
    double Tracked_X = blobX[0]; // Camera[0]检测到的点的X
    double Tracked_Y = blobY[0]; // Camera[0]检测到的点的Y
    double Tracked_Z = blobX[1]; // Camera[1]检测到的点的X. 在640 x 480的这个空间里(非1280)
    double Tracked_W = blobY[1]; // Camera[1]检测到的点的Y. 在640 x 480的这个空间里
    
    //cout << "Blob0_X : " << blobX[0] << endl;
    //cout << "Blob0_Y : " << blobY[0] << endl;
    //cout << "Blob1_X : " << blobX[1] << endl;
    //cout << "Blob1_Y : " << blobY[1] << endl;
    
    //将各自的blobX[i] 2D 转化到各自的CameraCoor[i] 3D去
    double TrackPoint[2][4];
    PixelToCamera(0,blobX[0],blobY[0],TrackPoint);
    PixelToCamera(1,blobX[1],blobY[1],TrackPoint);
    
    //4点法在这里求两个方向向量
    
    
    // Use the Matrix Multiplication to compute the Point Tracked
    //Pos_world[i]用来存放第i个相机计算出来的LED 3D World Coordinate结果
    CvMat Pos_World[2];
    Pos_World[0] = cvMat(4, 1, CV_64FC1,TrackPoint[0]);//(它的第四个元素是1)
    Pos_World[1] = cvMat(4, 1, CV_64FC1,TrackPoint[1]);//(它的第四个元素是1)
    cvmMul(T[0], &Pos_World[0], &Pos_World[0]);
    cvmMul(T[1], &Pos_World[1], &Pos_World[1]);
    
    // Pi - Oi法在这里求两个方向向量
    //0814 还是想尝试自己的方法
    //注意单位化
    //RVec_World[i] = cvmGet( Pos_World[i], 0, 0) - cvmGet( Camera_Origin[i], 0, 0)
    cvmSet(RVec_World[0], 0, 0, cvmGet(&Pos_World[0], 0, 0) - cvmGet(Camera_Origin[0], 0, 0));
    cvmSet(RVec_World[0], 1, 0, cvmGet(&Pos_World[0], 1, 0) - cvmGet(Camera_Origin[0], 1, 0));
    cvmSet(RVec_World[0], 2, 0, cvmGet(&Pos_World[0], 2, 0) - cvmGet(Camera_Origin[0], 2, 0));
    //cvNormalize(RVec_World[0], RVec_World[0]);
    
    cvmSet(RVec_World[1], 0, 0, cvmGet(&Pos_World[1], 0, 0) - cvmGet(Camera_Origin[1], 0, 0));
    cvmSet(RVec_World[1], 1, 0, cvmGet(&Pos_World[1], 1, 0) - cvmGet(Camera_Origin[1], 1, 0));
    cvmSet(RVec_World[1], 2, 0, cvmGet(&Pos_World[1], 2, 0) - cvmGet(Camera_Origin[1], 2, 0));
    //cvNormalize(RVec_World[1], RVec_World[1]);
    
    //P1 = Origin1
    //P3 = Origin2
    P[1] = Camera_Origin[0];
    P[3] = Camera_Origin[1];
    
    //P2 = P1W
    //P4 = P2W
    cvmSet(P[2], 0, 0, cvmGet(&Pos_World[0],0,0));
    cvmSet(P[2], 1, 0, cvmGet(&Pos_World[0],1,0));
    cvmSet(P[2], 2, 0, cvmGet(&Pos_World[0],2,0));
    
    cvmSet(P[4], 0, 0, cvmGet(&Pos_World[1],0,0));
    cvmSet(P[4], 1, 0, cvmGet(&Pos_World[1],1,0));
    cvmSet(P[4], 2, 0, cvmGet(&Pos_World[1],2,0));
    
    //mua = ( d1343 d4321 - d1321 d4343 ) / ( d2121 d4343 - d4321 d4321 )
    //mub = ( d1343 + mua d4321 ) / d4343
    
    double mua = ( Dmnop(1,3,4,3) * Dmnop(4,3,2,1) - Dmnop(1,3,2,1) * Dmnop(4,3,4,3)) / ( Dmnop(2,1,2,1) *Dmnop(4,3,4,3) - Dmnop(4,3,2,1) * Dmnop(4,3,2,1) );
    
    double mub = ( Dmnop(1,3,4,3) + mua * Dmnop(4,3,2,1) ) / Dmnop(4,3,4,3);
    
    //Po0 = Origin[0] + mua * R1;
    //Po1 = Origin[1] + mub * R2;
    cvmSet(P_opt[0], 0, 0, cvmGet(Camera_Origin[0], 0, 0) + mua * cvmGet(RVec_World[0], 0, 0));
    cvmSet(P_opt[0], 1, 0, cvmGet(Camera_Origin[0], 1, 0) + mua * cvmGet(RVec_World[0], 1, 0));
    cvmSet(P_opt[0], 2, 0, cvmGet(Camera_Origin[0], 2, 0) + mua * cvmGet(RVec_World[0], 2, 0));
    
    cvmSet(P_opt[1], 0, 0, cvmGet(Camera_Origin[1], 0, 0) + mub * cvmGet(RVec_World[1], 0, 0));
    cvmSet(P_opt[1], 1, 0, cvmGet(Camera_Origin[1], 1, 0) + mub * cvmGet(RVec_World[1], 1, 0));
    cvmSet(P_opt[1], 2, 0, cvmGet(Camera_Origin[1], 2, 0) + mub * cvmGet(RVec_World[1], 2, 0));
    
    //LED_Pos3D为 (Po1 + Po2)/2
    double Position_X = cvmGet(P_opt[0], 0, 0) + cvmGet(P_opt[1], 0, 0);
    Position_X /= 2;
    double Position_Y = cvmGet(P_opt[0], 1, 0) + cvmGet(P_opt[1], 1, 0);
    Position_Y /= 2;
    double Position_Z = cvmGet(P_opt[0], 2, 0) + cvmGet(P_opt[1], 2, 0);
    Position_Z /= 2;
    
    //0820 Adds UDP Communication here
    //Format decided
    
    //0829 modified to match the pool coordinates
    Position_X = Position_X + 890 - 1000;
    Position_Y = Position_Y + 1015 - 500;
    Position_Z = Position_Z + 75 - 1500;
    ofxOscMessage m;
    m.setAddress("/test1");
    m.addFloatArg(Position_X);
    m.addFloatArg(Position_Y);
    m.addFloatArg(Position_Z);
    sender.sendMessage(m);
    
    
    cvmSet(LED_Pos3D, 0, 0, Position_X);
    cvmSet(LED_Pos3D, 1, 0, Position_Y);
    cvmSet(LED_Pos3D, 2, 0, Position_Z);
    
    ofDrawBitmapString("LED Position  X :" + ofToString(cvmGet(LED_Pos3D, 0, 0)),   20,180);
    ofDrawBitmapString("LED Position  Y :" + ofToString(cvmGet(LED_Pos3D, 1, 0)),   20,200);
    ofDrawBitmapString("LED Position  Z :" + ofToString(cvmGet(LED_Pos3D, 2, 0)),   20,220);
    
    //ログと操作説明を表示
    
    ofSetColor(255, 255, 255);
    ofDrawBitmapString("Number of Blobs: "+ofToString(contourFinder[0].nBlobs), 20, 50);
    ofDrawBitmapString("Number of Blobs: "+ofToString(contourFinder[1].nBlobs), 20, 70);
    
    ofDrawBitmapString("Camera 0 is in : ", 20, 80);
    ofDrawBitmapString("X : "+ofToString(cvmGet(T[0],0,3)), 20, 90);
    ofDrawBitmapString("Y : "+ofToString(cvmGet(T[0],1,3)), 20, 100);
    ofDrawBitmapString("Z : "+ofToString(cvmGet(T[0],2,3)), 20, 110);
    
    
    ofDrawBitmapString("Camera 1 is in : ", 20, 130);
    ofDrawBitmapString("X : "+ofToString(cvmGet(T[1],0,3)), 20, 140);
    ofDrawBitmapString("Y : "+ofToString(cvmGet(T[1],1,3)), 20, 150);
    ofDrawBitmapString("Z : "+ofToString(cvmGet(T[1],2,3)), 20, 160);
    
}
/*
 cout << "Pos_World[1] X" << cvmGet(&Pos_World[1], 0, 0) << endl;
 cout << "Pos_World[1] Y" << cvmGet(&Pos_World[1], 1, 0) << endl;
 cout << "Pos_World[1] Z" << cvmGet(&Pos_World[1], 2, 0) << endl;
 cout << "Pos_World[0] X" << cvmGet(&Pos_World[0], 0, 0) << endl;
 cout << "Pos_World[0] Y" << cvmGet(&Pos_World[0], 1, 0) << endl;
 cout << "Pos_World[0] Z" << cvmGet(&Pos_World[0], 2, 0) << endl;
 
 
 cout << "向量A的X : " << cvmGet(Edge_A,0,0) << endl;
 cout << "向量A的Y : " << cvmGet(Edge_A,1,0) << endl;
 cout << "向量A的Z : " << cvmGet(Edge_A,2,0) << endl;
*/

//--------------------------------------------------------------
void ofApp::keyPressed(int key){
    
    
}

//--------------------------------------------------------------
//该函数的作用是利用各自的Minstrinsic[i]^-1将各自的blobX[i]和blobY[i]
//转化为各自CameraCoor[i]上的3D点 (X,Y,Z)
//输入为Camera Index i, 各自追踪到的blobX[i],blobY[i]，以及作为结果带回主调函数的TrackedPoint_Camera3D[i][4]
//其中输出的数据格式为(X,Y,Z,1)
//将TrackedPoint的各行指针数组当一维数组了.注意此时TrackedPoint_Camera3D[i]即为TrackedPoint_Camera3D[i][4]的首指针
//此函数正确
void ofApp::PixelToCamera(int i, double X, double Y, double TrackedPoint_Camera3D[][4]){
    //Change (x,y)to homogeous Coordinates (x,y,1)
    double Pixel[3] = {X, Y, 1};
    
    //Cameras is the Points in Camera Coor that Correspondes to (x,y) in Image Space
    // M^-1 * (x,y,1) = (X,Y,Z). Which is 3x1
    CvMat *Camera = cvCreateMat(3, 1, CV_64FC1);
    cvInitMatHeader(Camera,3, 1, CV_64FC1 ,Pixel);
    
    //Compute M^-1
    CvMat *InvertM = cvCreateMat(3, 3, CV_64FC1);
    cvmInvert(CameraIntrinsic[i], InvertM);
    
    cvmMul(InvertM, Camera, Camera);
    //Copy the result to TrackedPoint_Camera3d[i]
    //3 elems in Camera 3x1 in total
    for(int k = 0; k < 3; k++){
        TrackedPoint_Camera3D[i][k] = cvmGet(Camera, k, 0);
    }
    // Set the last elem to be 1
    TrackedPoint_Camera3D[i][3] = 1;
    
}

void ofApp::compute_T_Cam2World(int i){
    
    //Change the format of the points to be used
    CvMat    src_point_2D;// Mat for 2D pixie points in Camera Display
    CvMat    dst_point_3D;// Mat for DST datas in World space
    CvMat    *R_from_RVec; // 3 x 3 Rotation Matrix transformed from RVec
    
    src_point_2D = cvMat( NUM_POINTS, 2, CV_64FC1, pixie_4points[i]);
    dst_point_3D = cvMat( NUM_POINTS, 3, CV_64FC1, DST_Points_3D);
    R_from_RVec = cvCreateMat(3, 3, CV_64FC1);
    
    cvFindExtrinsicCameraParams2(&dst_point_3D, &src_point_2D, CameraIntrinsic[i], DVec[i], RVec, TVec);
    cvRodrigues2(RVec, R_from_RVec);// Get Rotation Matrix from RVec. 注意Cross Product求出的第三列是按照右手法则求出来的，因此如果按照画面坐标系的那种设定，X x Y会得到方向向下的Z
    
    // Scalar s = TVec[2,0];
    double s = cvmGet(TVec, 2, 0);
    //cout << " The scalar is : " << s << endl;
    
    //由于X, Y的叉乘得到的结果向量是符合右手法则，而且是从X乘到Y，因此Z默认会朝下，于是手动反转R的第三列
    //0811 为了确保结果正确，暂时保留反转
    // for (int m = 0; m < 3; m++)
    //    cvmSet(R_from_RVec, m, 2, -1*cvmGet(R_from_RVec, m, 2));
    
    // R^-1 = R^Transpose
    cvTranspose(R_from_RVec, R_from_RVec);
    
    // temp will be the result of -R^T * t. It is 3x1
    CvMat *temp = cvCreateMat(3, 1, CV_64FC1);
    cvScale(R_from_RVec,R_from_RVec,-1);// Now R^T = - R^T
    cvmMul(R_from_RVec, TVec, temp); // -R^T * t
    cvScale(R_from_RVec,R_from_RVec,-1);// bring back R^T to plus
    
    //Copy R^T to left upper 3x3 of T[i]
    for ( int m = 0; m < 3; m++ ) {
        for ( int n = 0; n < 3; n++ ) {
            cvmSet(T[i], m, n, cvmGet(R_from_RVec, m, n));
        }
    }
    //乘上Scalar s
    cvScale(T[i],T[i],s);
    
    //copy -R^T * t to T's last col
    //0810
    //Pi-Oi法在这里给Rvec_World[i]赋值,先让它默认等于两个原点的值
    //0812
    //在这里存上两个原点的值
    for (int j = 0; j < 3; j++)
    {
        cvmSet(T[i], j, 3, cvmGet(temp, j, 0));
        cvmSet(Camera_Origin[i], j, 0, cvmGet(temp, j, 0));
    }
    
    //Set the last row of T to be 0,0,0,1
    for (int m = 0; m < 3; m++)
        cvmSet(T[i], 3, m, 0);
    cvmSet(T[i], 3, 3, 1);// Ti[3][3] = 1
    
    cout << "The Camera->World Matrix is :" << endl;
    for ( int m = 0; m < 4; m++ ) {
        for ( int n = 0; n < 4; n++ ) {
            cout << cvmGet( T[i], m, n ) << " ";
        }
        cout << endl;
    }
    
    
    /* T is 4x4
     T = ( R^T  -R^T * t )
     (  0, 0, 0  1    )
     */
}
//--------------------------------------------------------------
double ofApp::Dmnop(int m, int n, int o, int p){
    /*
     mua = ( d1343 d4321 - d1321 d4343 ) / ( d2121 d4343 - d4321 d4321 )
     mub = ( d1343 + mua d4321 ) / d4343
     dmnop = (xm - xn)(xo - xp) + (ym - yn)(yo - yp) + (zm - zn)(zo - zp)
     */
    double xm,xn,xo,xp,ym,yn,yo,yp,zm,zn,zo,zp;
    double result;
    xm = cvmGet(P[m], 0, 0);
    ym = cvmGet(P[m], 1, 0);
    zm = cvmGet(P[m], 2, 0);
    
    xn = cvmGet(P[n], 0, 0);
    yn = cvmGet(P[n], 1, 0);
    zn = cvmGet(P[n], 2, 0);
    
    xo = cvmGet(P[o], 0, 0);
    yo = cvmGet(P[o], 1, 0);
    zo = cvmGet(P[o], 2, 0);
    
    xp = cvmGet(P[p], 0, 0);
    yp = cvmGet(P[p], 1, 0);
    zp = cvmGet(P[p], 2, 0);
    
    result = (xm - xn)*(xo - xp) + (ym - yn)*(yo - yp) + (zm - zn)*(zo - zp);
    return result;
}



//--------------------------------------------------------------
void ofApp::computeDirectionalVector(int i, double X, double Y){
    //首先，得到屏幕上四周的四个点
    //之后，按照Image->Camera->World的顺序将这些点转化到世界坐标系中的同一平面上
    //横向向量为V1，竖向为V2
    //RVec_World = V1 x V2. 然后单位化之即可
}


//--------------------------------------------------------------
void ofApp::keyReleased(int key){
    
}

//--------------------------------------------------------------
void ofApp::mouseMoved(int x, int y ){
    
}

//--------------------------------------------------------------
void ofApp::mouseDragged(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::mousePressed(int x, int y, int button){
    
    cout << "mousePressed: " << x << ", " << y << " button: " << button << endl;
    
    
    //0820 Adds UDP Communication here
    /*
    ofxOscMessage m;
    m.setAddress("/test1");
    m.addFloatArg(x);
    m.addFloatArg(y);
    m.addFloatArg(0);
    sender.sendMessage(m);
    */
    
    
    count ++;
    Set = count / 5;
    
    
    pixie_4points[Set][SetIndex++] = x;
    pixie_4points[Set][SetIndex++] = y;
    if (SetIndex == 8) //得到第一组点之后，计算第一个Homography:Homography_Camera[0] ，然后计算第二个Homography_Camera[1]
    {
        SetIndex = 0;
        
        
        //由于是在一个画面中取的点，所以需要把第二个相机取得的所有点减去camHeight来统一成同一个尺寸的Image Space
        if (Set == 1) {
            pixie_4points[Set][0] -= camWidth;
            pixie_4points[Set][2] -= camWidth;
            pixie_4points[Set][4] -= camWidth;
            pixie_4points[Set][6] -= camWidth;
        }
        
        
        /*
         cout << "Point Group : " << Set << " is Ready, Start SolvePnp() " << endl;
         
         
         for (int k = 0; k < 8;)
         {
         cout << "(" << pixie_4points[Set][k++] << "," << pixie_4points[Set][k++] << ")" << endl;
         }
         */
        
        
        //Compute the Matrix T using the 4 points in the display the 4 corresponding ones in World Coor
        //For Camera i
        
        compute_T_Cam2World(Set);
        
        // Camera position in World Coordinate is T[i] * (0,0,0,1). (0,0,0,1)is the position of Camera Center
        // Which is the last col of T[i]
        cout << "Camera " << Set << " World Coordinate is :" << endl;
        
        for (int m = 0; m < 3; m++)
            cout << cvmGet(T[Set], m, 3) << " ";
        cout << endl;
        
        //homographyCompute(pixie_4points[i], Homography_Camera[i]);
    }
}

//--------------------------------------------------------------
void ofApp::mouseReleased(int x, int y, int button){
    
}

//--------------------------------------------------------------
void ofApp::windowResized(int w, int h){
    
}

//--------------------------------------------------------------
void ofApp::gotMessage(ofMessage msg){
    
}

//--------------------------------------------------------------
void ofApp::dragEvent(ofDragInfo dragInfo){
    
}
