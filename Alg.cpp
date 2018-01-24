/**
 * @function Watershed_and_Distance_Transform.cpp
 * @brief Sample code showing how to segment overlapping objects using Laplacian filtering, in addition to Watershed and Distance Transformation
 * @author OpenCV Team
 */

#include <opencv2/opencv.hpp>
#include <iostream>
#include <math.h>
//#include "Alg.hpp"


using namespace std;
using namespace cv;

//Header
void init();

void loadImg(const string&, Mat &);

void resizeImg(const Mat&, Mat&, float);

void prepWork(Mat&);

void save(Mat &img, const string&);

void flt(Mat &src, Mat &dest, Mat &prod, Mat &height,int begin,int size);

void Erosion(int, Mat&);

void Dilation(int, Mat&);

int dust(Mat &src, int i, int j, Mat &map, int path);

void clean(Mat &src, int i, int j, Mat &map);

void cleanNoise(Mat &src, int pathMax);

vector<int> compression_params;

void getRoi(Mat&, Mat&);

int main()
{
	printf("\nflag0\n");
	init();
	Mat src, trc, ori, trc2, ori2, trtst;
printf("\nflag001\n");
	loadImg("ori.png", src);
printf("\nflag002\n");
	Mat kernel = (Mat_<float>(3,3) <<
            -1,  -1, -1,
            -1, 8, -1,
            -1,  -1, -1);
	Mat kernel72 = (Mat_<float>(7,7) <<
		1, 2, 4, 8, 4, 2, 1,
		2, 4, 8, 16, 8, 4, 2,
		4, 8, 16, 32, 16, 8, 4,
		8, 16, 32, 64, 32, 16, 8,
		4, 8, 16, 32, 16, 8, 4,
		2, 4, 8, 16, 8, 4, 2,
		1, 2, 4, 8, 4, 2, 1);
	Mat kernel7 = (Mat_<float>(7,7) <<
		1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1,
		1, 1, 1, 1, 1, 1, 1);
	float temp=0;
	for (int i=0; i<7; i++){
		for (int j=0; j<7; j++){
			temp +=	kernel7.at<float>(i, j);
		}
	}
	for (int i=0; i<7; i++){
		for (int j=0; j<7; j++){
			kernel7.at<float>(i, j) = kernel7.at<float>(i,j)/temp;
		}
	}
	Mat kernel3 = (Mat_<float>(3,3) <<
		1, 2, 1,
		2, 4, 2,
		1, 2, 1);
	
	Mat kernelx = (Mat_<float>(3,3) <<
		1, 0, -1,
		2, 0, -2,
		1, 0, -1);
	Mat kernely = (Mat_<float>(3,3) <<
		-1, 0, 1,
		-2, 0, 2,
		-1, 0, 1);

    Mat imgLaplacian, edgeX, edgeY, dst;
    Mat sharp = src; // copy source image to another temporary one
	filter2D(sharp, imgLaplacian, CV_32F, kernel);
	resize(imgLaplacian, sharp, Size(), 0.5, 0.5);
	//imwrite("4.png", sharp, compression_params);
    src.convertTo(sharp, CV_32F);
    Mat imgResult = imgLaplacian;
	printf("\nflag1\n");
	filter2D(sharp, sharp, CV_32F, kernel7);
	filter2D(sharp, edgeX, CV_32F, kernelx);
	filter2D(sharp, edgeY, CV_32F, kernely);
	imgResult.convertTo(imgResult, CV_8UC3);
	edgeX.convertTo(edgeX, CV_8UC3);
	edgeY.convertTo(edgeY, CV_8UC3);
	sharp.convertTo(sharp, CV_8UC3);
	//  sharp = imgResult*0.08 + src*0.92; 
// imshow( "Laplace Filtered Image", imgLaplacian );
	imwrite("01blured.png", sharp, compression_params);
	imwrite("02XtoLeftBoarder.png", edgeX, compression_params);
	imwrite("03XtoRighttBoarder.png", edgeY, compression_params);
printf("\nflag2\n");
	ori = edgeX;
	ori2 = edgeY;
	trc = Mat::zeros(src.rows, src.cols, CV_8UC1);
	trc2 = Mat::zeros(src.rows, src.cols, CV_8UC1);
	trtst = Mat::zeros(src.rows, src.cols, CV_32F);
	prepWork(ori);
	prepWork(ori2);
	prepWork(src);	
	prepWork(trc);
	prepWork(trc2);

	int i, j, N_ROWS=50, cont=0, desv=0;
	float mean=0, max=0, min=255;

	flt(src, trtst, ori, trc,-N_ROWS/2, N_ROWS);
	flt(src, trtst, ori2, trc2,-N_ROWS/2, N_ROWS);

	ori.convertTo(ori, CV_8UC1);
	//ori2.convertTo(ori2, CV_8UC1);

	//equalizeHist(ori, ori);
	//equalizeHist(ori2, ori2);


	//save(ori2, "mult3.png");		
	save(trtst, "04mascaraDesvioPadraoHorizontal.png");
	
	save(ori, "05produtoMascaraDPHSrc.png");
	
	ori.convertTo(ori, CV_8UC1);
	equalizeHist(ori, ori);
	save(ori, "05produtoMascaraDPHSrcEqualizado.png");
printf("\nflag3\n");
	prepWork(trtst);
//	prepWork(ori);
//	prepWork(ori2);

//	trtst = ori;
	src.convertTo(src, CV_8UC1);
	trc.convertTo(trc, CV_8UC1);
	trc2.convertTo(trc2, CV_8UC1);
	ori.convertTo(ori, CV_8UC1);
	ori2.convertTo(ori2, CV_8UC1);


	trc = Mat::zeros(src.rows, src.cols, CV_8UC1);

	//ori = trtst;
	for (i=ori.rows*3/5; i<ori.rows; i++){
		for (j=ori.cols*3/4; j<ori.cols; j++){
		//	trc.at<float>(i, j) = src.at<float>(i, j);
			float prev, next;
			prev = ori.at<uchar>(i, j-5)+ori.at<uchar>(i, j-4)+ori.at<uchar>(i, j-3)+ori.at<uchar>(i,j-2)+ori.at<uchar>(i,j-1);
			prev /= 5;
			next = ori.at<uchar>(i,j+1)+ori.at<uchar>(i,j-+2)+ori.at<uchar>(i,j+3)+ori.at<uchar>(i, j+4)+ori.at<uchar>(i, j+5);
			next /= 5;
			trc.at<uchar>(i,j)=0;
			if (abs(prev-next)>50)
				trc.at<uchar>(i,j)=255;
			/*prev = ori2.at<uchar>(i, j-5)+ori2.at<uchar>(i, j-4)+ori2.at<uchar>(i, j-3)+ori2.at<uchar>(i,j-2)+ori2.at<uchar>(i,j-1);
			prev /= 5;
			next = ori2.at<uchar>(i,j+1)+ori2.at<uchar>(i,j+2)+ori2.at<uchar>(i,j+3)+ori2.at<uchar>(i, j+4)+ori2.at<uchar>(i, j+5);
			next /= 5;
			trc2.at<uchar>(i,j)=0;
			if (abs(prev-next)>50)
				trc2.at<uchar>(i,j)=255;*/
		}
	}

	Erosion(2, trc);
	//Dilation(1, trc);
	//Erosion(2, trc);
	//Dilation(2, trc);

	//Erosion(2, trc2);
	//Dilation(1, trc2);
//	Erosion(2, trc2);
//	Dilation(2, trc2);

	save(trc, "06mascaraBordaSemTirarRuido.png");

	prepWork(trc);

	cleanNoise(trc, 1000);
printf("\nflag4\n");
	save(trc, "07mascaraBordaComTirarRuido.png.png");
	//prepWork(trc);
	//prepWork(trc2);
	trc2.convertTo(trc2, CV_8UC1);
	trc.convertTo(trc, CV_8UC1);

	Mat roi = Mat::zeros(src.rows, src.cols, CV_8UC1);

	getRoi(trc, roi);

	save(roi, "08regiaoDeInteresse.png");

	vector<Mat> channels;
    	channels.push_back(src);
   	channels.push_back(src);
    	channels.push_back(src+roi);
	merge(channels, trc);
	//trc.convertTo(trc, CV_8UC3);
	src.convertTo(src, CV_8UC3);
	
	//trc = 0.6*src + 0.4*trc;
	imwrite("09regiaoInteresseSobreposta.png", trc, compression_params);

    // Create a kernel that we will use for accuting/sharpening our image
    
    waitKey(0);
    return 0;
}

void init(){
	compression_params.push_back(CV_IMWRITE_PNG_COMPRESSION);
	compression_params.push_back(0);
}

void loadImg(const string& path, Mat &src){
	src = imread(path, CV_LOAD_IMAGE_GRAYSCALE);
	if (!src.data)
		exit(-1);
	src.convertTo(src, CV_8UC1);
}

void prepWork(Mat &img){
	img.convertTo(img, CV_32F);
}

void save(Mat &img, const string &name){
	img.convertTo(img, CV_8UC3);
	imwrite(name, img, compression_params);
}

void resizeImg(const Mat &src, Mat &output, float scale){
	resize(src, output, Size(), scale, scale);
}


void Erosion(int erosion_size, Mat &src)
{
  int erosion_type, erosion_elem=0;
  if( erosion_elem == 0 ){ erosion_type = MORPH_RECT; }
  else if( erosion_elem == 1 ){ erosion_type = MORPH_CROSS; }
  else if( erosion_elem == 2) { erosion_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( erosion_type,
                                       Size( 2*erosion_size + 1, 2*erosion_size+1 ),
                                       Point( erosion_size, erosion_size ) );
  /// Apply the erosion operation
  erode( src, src, element );
  //imshow( "Erosion Demo", erosion_dst );
}

void Dilation(int dilation_size, Mat &src)
{
  int dilation_type, dilation_elem=0;
  if( dilation_elem == 0 ){ dilation_type = MORPH_RECT; }
  else if( dilation_elem == 1 ){ dilation_type = MORPH_CROSS; }
  else if( dilation_elem == 2) { dilation_type = MORPH_ELLIPSE; }

  Mat element = getStructuringElement( dilation_type,
                                       Size( 2*dilation_size + 1, 2*dilation_size+1 ),
                                       Point( dilation_size, dilation_size ) );

  dilate( src, src, element );

}

void flt(Mat &src, Mat &dest, Mat &prod, Mat &height,int begin,int size){
	float sumWidth=0, sumHeight=0, mean=0, desvH=0, desv=0, meanH=0;
	Mat map = Mat::zeros(src.rows, src.cols, CV_8UC1);
	int i, j, cont, max=0, min=255;
	for(i=-3*begin; i<src.rows-1-3*(size+begin); i++){
		for (j=-begin; j<src.cols-1-(size+begin); j++){
			if(sumWidth==0){
			//---------Faz a media horizontal com N pixels. Se for o primeiro pixel, faz a media toda, se for nao for, usa a media do ultimo pixel para precisar apenas subtrair um pixel e adicionar outro.
			for(cont=begin; cont<(size+begin); cont++){
					sumWidth+=(float)src.at<float>(i,j+cont);
				//max = src.at<float>(i,j+cont) > max ? src.at<float>(i, j+cont) : max;
				//min = src.at<float>(i,j+cont) < min ? src.at<float>(i, j+cont) : min;
	
				}
			}else{
				sumWidth-=(float)src.at<float>(i,j+begin-1);
				sumWidth+=(float)src.at<float>(i,j+size+begin-1);
			}
			//---------------------------------------------Faz a media vertical usando o mesmo principio de otimizacao da media horizontal.
			if (sumHeight==0){
				for(cont=3*begin; cont<3*(size+begin); cont++)
					sumHeight+=(float)src.at<float>(i+cont,j);
			}else{
				sumHeight-=(float)src.at<float>(i+begin-1,j);
				sumHeight+=(float)src.at<float>(i+size+begin-1,j);
			}
			//Calcula as medias
			mean = (float)sumWidth/size;
			meanH = (float)sumHeight/(3*size);
			//Calcula desvios padroes
			for(cont=begin; cont<(size+begin); cont++){
				desv += abs(mean - src.at<float>(i, j+cont));
				desvH += abs(meanH - src.at<float>(i+cont, j));
			}
			desv/=(float)size;
			desvH/=(float)3*size;
			//if (desvH>3){
			dest.at<float>(i, j) = 10*(exp((src.at<float>(i, j)-mean)/desv)-1); 
			height.at<float>(i, j) = 10*(exp((src.at<float>(i, j)-meanH)/desvH)-1);
			//}
			
			//prod.at<float>(i, j) = sqrt(src.at<float>(i, j)*sqrt(dest.at<float>(i, j)*height.at<float>(i, j)));
		
			prod.at<float>(i, j) = sqrt(src.at<float>(i, j)*dest.at<float>(i, j));
//100*(src.at<float>(i, j)-mean)/desv;
	//		else
	//			trtst.at<float>(i, j) += 5*(src.at<float>(i, j)-mean)/desv;
			max = 0;
			min = 255;
			desv=0;
			desvH=0;
			
			sumWidth=0;
			sumHeight=0;
		}								
	}
}

void cleanNoise(Mat &src, int pathMax){
	Mat map = Mat::zeros(src.rows, src.cols, CV_8UC1);
	int i, j;
	Mat map2 = Mat::zeros(src.rows, src.cols, CV_8UC1);
	for(i=0; i<src.rows; i++){
		for (j=0; j<src.cols; j++){
			int path = dust(src, i, j, map, 0);
						
			if (path<pathMax && path!=0)
			clean(src, i, j, map2);
		}
	}
}

int dust(Mat &src, int i, int j, Mat &map, int path){
	if (map.at<uchar>(i, j)==1)
		return 0;

	map.at<uchar>(i, j)=1; //visitado

	if (src.at<float>(i, j)==0)
		return 0;

	if (src.at<float>(i, j)==255){
		path++;
		path += dust(src, i+1, j, map, path);
		path += dust(src, i+1, j+1, map, path);
		path += dust(src, i+1, j-1, map, path);
		path += dust(src, i, j-1, map, path);
		path += dust(src, i-1, j-1, map, path);
		path += dust(src, i-1, j, map, path);
		path += dust(src, i-1, j+1, map, path);
		path += dust(src, i, j+1, map, path);
	}

	return path;
}

void clean(Mat &src, int i, int j, Mat &map){
	if (map.at<uchar>(i, j)==1)
		return ;

	map.at<uchar>(i, j)=1; //visitado

	if (src.at<float>(i, j)==0)
		return ;

	if (src.at<float>(i, j)==255){
		src.at<float>(i, j) = 0;	
		clean(src, i+1, j, map);
		clean(src, i+1, j+1, map);
		clean(src, i+1, j-1, map);
		clean(src, i, j-1, map);
		clean(src, i-1, j-1, map);
		clean(src, i-1, j, map);
		clean(src, i-1, j+1, map);
		clean(src, i, j+1, map);
	}

	return ;
}

void getRoi(Mat &src, Mat &roi){

	bool foundRoi=false;
	int roiIni, roiEnd, i, j;
	int bTop=-1, bBot, bRight, bLeft;
	int bTopLast, bBotLast, bRightLast, bLeftLast;
	
	for (i=src.rows*3/5; i<src.rows-1; i++){
		for (j=src.cols-1; j>src.cols*3/4; j--){
			if(src.at<uchar>(i, j)==255){
				int cont, k;
				for (k=0; src.at<uchar>(i, j-k)==255; k++);
				for (cont=k; cont<k+20; cont++){
					if (src.at<uchar>(i, j-cont)==255){
						foundRoi=true;
						roiIni=j;
						roiEnd=j-cont;
					}
				}
				if(foundRoi){
					//for (cont=roiIni; cont>=roiEnd; cont--)
					//	roi.at<uchar>(i, cont) = 255;
					const int margin = 15;
					if(bTop==-1){
						bRight = roiIni;
						bLeft = roiEnd;
						bTop = i;
						bBot = i;
					}else if (abs(roiIni-bRight)<30 && abs(roiEnd -bLeft)<30 && abs(i-bBot)<50){
						bRight = roiIni > bRight ? roiIni : bRight;
						bBot = i;
						bLeft = roiEnd < bLeft ? roiEnd : bLeft;	
					}else{
						if (abs(bTop-bBot)>5 && abs(bRight-bLeft)>3){
							bTop -= margin;
							bBot += margin;
							bRight += margin;
							bLeft -= margin;
							rectangle(roi, Point(bLeft, bTop), Point(bRight, bBot), Scalar(255,0,0), 3, -1);
						}
						bTop = -1;
					}
					j=roiEnd;
					foundRoi=false;				
				}
			}	
		}	
	}
	if (bTop!=-1){
			if (abs(bTop-bBot)>5 && abs(bRight-bLeft)>3){
				const int margin = 15;
				bTop -= margin;
				bBot += margin;
				bRight += margin;
				bLeft -= margin;
				rectangle(roi, Point(bLeft, bTop), Point(bRight, bBot), Scalar(255,0,0), 3, -1);
				}
	}
}
