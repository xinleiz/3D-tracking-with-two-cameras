#pragma once

#include "ofMain.h"
#include "ofxOpenCv.h"
#include "ofxOsc.h"

//Defination for UDP PORT Used.
#define HOST "localhost"//"172.27.254.24"
#define PORT 1234 // modified in 8/28, original is 7000

class ofApp : public ofBaseApp{
	
public:
    
    void setup();
    void update();
    void draw();
    
    void keyPressed(int key);
    void keyReleased(int key);
    void mouseMoved(int x, int y );
    void mouseDragged(int x, int y, int button);
    void mousePressed(int x, int y, int button);
    void mouseReleased(int x, int y, int button);
    void windowResized(int w, int h);
    void dragEvent(ofDragInfo dragInfo);
    void gotMessage(ofMessage msg);
    
    //Find the homography matrix based on a set of 4 points inputed.
    //Parameter :
    //pixie_4 is the array for orginal 4 points in the screen to be used as world coordinate
    //h is the matrix for homography with regard to DST_WIDTH, DST_HEIGHT
    //8/5号去掉了
    void homographyCompute(double pixie_4[], CvMat *h);
    
    //Change a 3x3 Homography Matrix into a Projection Matrix of 3x4
    //With the 3rd col of Projection Matrix is H.col(0) x H.col(1)
    void HomographytoProjection(const cv::Mat &H, cv::Mat &Proj);
    
    //Given 4 points in Display by pixie_4points
    //use 2 groups of corresponding 4 points to compute Matrix T for transforming
    //points in Camera 2D space to World 3D space
    //Parameter i stands for the index of the point group, and the number of camera[i] used
    //Version 2 will used different Mintrinsic and distortion coinefficents to Compute T as accurent as possible
    void compute_T_Cam2World(int i);
    
    //该函数的作用是利用各自的Minstrinsic[i]^-1将各自的blobX[i]和blobY[i]
    //转化为各自CameraCoor[i]上的3D点 (X,Y,Z)
    //输入为Camera Index i, 各自追踪到的blobX[i],blobY[i]，以及作为结果带回主调函数的TrackedPoint_Camera3D[i][4]
    //其中输出的数据格式为(X,Y,Z,1)
    //将TrackedPoint的各行指针数组当一维数组了.注意此时TrackedPoint_Camera3D[i]即为TrackedPoint_Camera3D[i][4]的首指针
    //实际使用时i纯用来指定CameraIntrinsic[i]和作为结果的TrackedPoint_Camera3D[i]用了
    void PixelToCamera(int i,double X,double Y, double TrackedPoint_Camera3D[][4]);
    
    //4点法计算与屏幕中某（u，v）点垂直的方向向量
    //输入：Image Space上的某点（u,v）
    //输出：过点(u,v)，指向Led 3D position的方向向量
    //注意单位化
    void computeDirectionalVector(int i,double X,double Y);
    
    //计算dmnop = (xm - xn)(xo - xp) + (ym - yn)(yo - yp) + (zm - zn)(zo - zp)的函数
    //每个点Pm(xm,ym,zm)用CvMat 3x1来表示
    //P1 = Origin1
    //P3 = Origin2
    //P2 = P1W
    //P4 = P2W
    /*
     mua = ( d1343 d4321 - d1321 d4343 ) / ( d2121 d4343 - d4321 d4321 )
     mub = ( d1343 + mua d4321 ) / d4343
     dmnop = (xm - xn)(xo - xp) + (ym - yn)(yo - yp) + (zm - zn)(zo - zp)
     dmnop = dopmn
     */
    double Dmnop(int m, int n, int o, int p);
    
    
    ofVideoGrabber 		vidGrabber[3];//Array of two cameras
    unsigned char * 	videoInverted;
    ofTexture			videoTexture[2];//Array of 2 Textures
    int 				camWidth;
    int 				camHeight;
    
    
    /*************************** Image Variables defination for two cameras******************************/
    ofxCvColorImage colorImg[2]; //オリジナルのカラー映像
    ofxCvGrayscaleImage grayImage[2]; //グレースケールに変換後
    ofxCvGrayscaleImage grayBg; //キャプチャーした背景画像, default to be black
    ofxCvGrayscaleImage grayDiff[2]; //現在の画像と背景との差分
    ofxCvColorImage   ColorHomography[2];//射影変換之后的图像
    /****************************************************************************************************/
    
    ofxCvContourFinder contourFinder[2]; //輪郭抽出のためのクラス
    
    /****************************************************************************************************/
    //
    double pixie_4points[2][8];//用来计算两个Homography Matrix的两组点.pixie_4points[0]为水平相机组pixie_4points[1]为竖直相机组.每组4个点.来自于鼠标点击的四个点
    double Pixel_4Points_3D[2][12]; // 用鼠标点击得到的3D坐标. 每个相机各一组，每组4个点，每个点三个元素x,y,z，故3x4 = 12
    
    //Blob检测出来的点. 两个相机各一组
    //blobX[0]是相机0的画面中检测到的blobX
    //blobY[0]是相机0的画面中检测到的blobY
    //blobX[1]同理
    double blobX[2];
    double blobY[2];
    /***********************************************************************************************/
    int threshold; //2値化の際の敷居値
    
    //Value of Camera Intrinsic. 3x3
    //更新各自的Intrinsic。更新于8/3
    //Calibrated in 8/29.
    double intrinsic[2][9] = {
        3.1148337828505043e+02, 0, 3.1526625635802731e+02,//Camera[0] Instrinsic Matrix
        0, 3.1180162979657894e+02, 2.8512630162223229e+02,
        0, 0, 1,
        3.1133547311714341e+02, 0, 3.0556647234349953e+02,
        0, 3.1129847725565293e+02, 2.3328674979430585e+02,//Camera[1] Instrinsic Matrix
        0, 0, 1
    };
    
    // 更新各自的DistortionVector of 5 x 1（原为4x1）
    double distortionVector[2][5] = {
        2.5545085291651874e-03, -5.4255210867477401e-02,//Camera[0] Distortion 5x1
        -3.3423286596530587e-04, 3.7022134949313812e-03,
        1.6985355409789748e-02,
        2.6941029413821240e-03, -2.5349919929231395e-02,//Camera[1] Distortion 5x1
        -1.6612592898836577e-03, -2.0433930411282112e-03,
        -2.1679990058715626e-02
    };
    
    CvMat    *CameraIntrinsic[2];//各Camera[i]用各自的Intrinsic Matrix
    CvMat    *Homography_Camera[2]; // Homography Matrix for camera 1 and camera 2
    CvMat    *T[2]; // T is a 4x4 Matrix for transforming a point in Camera space to World space
    // With Reference to http://stackoverflow.com/questions/13957150/opencv-computing-camera-position-rotation
    CvMat    *RVec; // 3 x 1 Rotation from Camera Coor - > World Coor
    CvMat    *TVec; // 3 x 1 Translation from Camera Coor - > World Coor
    CvMat    *DVec[2]; // 5 x 1 Distortion Vector for camera[i]. 更新于8.3
    
    CvMat    *Camera_Origin[2]; // Camera Position in real world for two Cameras respectively.
    CvMat    *RVec_World[2];// 3 x 1 Rotation Vector Used as the Directional Vector to Compute the 3d Postion of Tracked LED. 0 for Camera[0], 1 for Camera[1]
    CvMat    *Edge_A; // 3 x 1 Vector for P2W - P1W
    CvMat    *LED_Pos3D; // 3 x 1 Vector stands for the 3D position of LED tracked
    
    
    CvMat    *P[5];// Shortest distance法中的数组P
    CvMat    *P_opt[2];//两条线上各自距离Led 3d 位置最近的点. P_opt[0]在R1上，P_opt[1]在R2上
    // 0815 Shortest distance
    // Po1 = Origin1 + mua * r1;
    // Po2 = Origin2 + mub * r2;
    // r1 , r2注意单位化
    // Po1, Po2分别是R1,R2上距离互相最近的两个点
    double mua;
    double mub;
    
    
    // i代表第i组click的点. 0 - 1
    // j是每组点的index. 0 - 7
    // count是已click的点的个数. count/5 就会是组数.
    int Set, SetIndex, count = 0;
    
    // 用来选择当前的画面中显示哪个图像用的变量
    int videoMode = 1;
    
    
    //Homography的目标数值.
    //也是World Coordinates下4个Makerer点的对应数值
    //DST_WIDTH对应X， DST_HEIGHT对应Y
    //0828根据水池中的Maker位置进行了调整
    //OFFSET_X是最终确定的坐标轴上Maker原点的X坐标
    //OFFSET_Y是最终确定的坐标轴上Maker原点的Y坐标
    //OFFSET_Z是最终确定的坐标轴上Maker原点的Z坐标
    //水面到Marker是15mm，水面高560mm，Marker长于是作为原点的Marker的初始高度是560-15－250 = 295
    //OFFSET_X就是原点Marker的X轴偏移:
    //OFFSET_Z就是原点Marker的Z轴偏移:
    const double    OFFSET_X = 0;// 1000 - 110 = 890
    const double    OFFSET_Y = 0;// 1000+15 = 1015
    const double    OFFSET_Z = 0;// 75
    
    const double    DST_WIDTH = 250;// width to be transformed to
    const double    DST_HEIGHT = 250;// height to be transformed to
    const int    NUM_POINTS = 4;// Number of points. Also the Rows of a Matrix
    
    //原点->X轴(1,0,0)->Y轴(0,1,0)->(X,Y). (0,0,0)设置在了左上角
    double DST_Points_3D[12] = {
        OFFSET_X+0, OFFSET_Y+0, OFFSET_Z+0,//Origin :(0,0,0)
        OFFSET_X+DST_WIDTH, OFFSET_Y+0, OFFSET_Z+0,// X Axis (1,0,0)
        OFFSET_X+0,OFFSET_Y+DST_HEIGHT, OFFSET_Z+0,// Y Axis (0,1,0)
        OFFSET_X+DST_WIDTH, OFFSET_Y+DST_HEIGHT, OFFSET_Z+0 // (x,y,0) = (1,1,0)
    };
    
    //UDP Sender to send messages
    ofxOscSender sender;// UDP sender for communicating
};
