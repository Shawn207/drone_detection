#include <opencv2/opencv.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <math.h>
#include <vector>
#include <./darknet_ros/UV_detector.h>
#include <./darknet_ros/kalman_filter.h>
#include <queue>

using namespace std;
using namespace cv;

// UVbox

UVbox::UVbox()
{
    this->id = 0;
    this->toppest_parent_id = 0;
    this->bb = Rect(Point2f(0, 0), Point2f(0, 0));
}

UVbox::UVbox(int seg_id, int row, int left, int right)
{
    this->id = seg_id;
    this->toppest_parent_id = seg_id;
    this->bb = Rect(Point2f(left, row), Point2f(right, row));
}

UVbox merge_two_UVbox(UVbox father, UVbox son)
{
    // merge the bounding box
    int top =       (father.bb.tl().y < son.bb.tl().y)?father.bb.tl().y:son.bb.tl().y;
    int left =      (father.bb.tl().x < son.bb.tl().x)?father.bb.tl().x:son.bb.tl().x;
    int bottom =    (father.bb.br().y > son.bb.br().y)?father.bb.br().y:son.bb.br().y;
    int right =     (father.bb.br().x > son.bb.br().x)?father.bb.br().x:son.bb.br().x;
    father.bb = Rect(Point2f(left, top), Point2f(right, bottom));
    return father;
}

// UVtracker

UVtracker::UVtracker()
{
    this->overlap_threshold = 0.4;
}

void UVtracker::read_bb(vector<Rect> now_bb, vector<Rect> now_bb_D, vector<box3D> &box_3D)
{
    // measurement history
    this->pre_history = this->now_history;
    this->now_history.clear();
    this->now_history.resize(now_bb.size());
    // 3D box history
    this->pre_box_3D_history = this->now_box_3D_history;
    this->now_box_3D_history.clear();
    this->now_box_3D_history.resize(now_bb.size());
    // fix size flag history
    // this->pre_fix = this->now_fix;
    // this->now_fix.clear();
    // this->now_fix.resize(now_bb.size());
    // kalman filters
    this->pre_filter = this->now_filter;
    this->now_filter.clear();
    this->now_filter.resize(now_bb.size());
    // sum of velocity prediction
    this->pre_V = this->now_V;
    this->now_V.clear();
    this->now_V.resize(now_bb.size());
    // count of moving result
    // this->pre_count = this->now_count;
    // this->now_count.clear();
    // this->now_count.resize(now_bb.size());
    // bounding box
    this->pre_bb = this->now_bb;
    this->now_bb = now_bb;
    this->now_bb_D = now_bb_D;
    this->now_box_3D = box_3D;
    // pre_history only stores 30 frames record for each track
    // for (int i=0 ; i<this->pre_history.size() ; i++) {
    //     if (this->pre_history[i].size() > 10) {
    //         this->pre_history[i].erase(this->pre_history[i].begin());
    //     }
    // }  

    for (int i=0 ; i<this->pre_box_3D_history.size() ; i++) {
        // for each box track, fix the size after showing up for 30 frames
        if (this->pre_box_3D_history[i].size() > 10) {
            this->pre_box_3D_history[i].pop_front();
            // fix box size to be the last one showed fully in FOV
            // also store the value for future 
            // if (pre_fix[i]) {
            //     pre_fix[i] = false;
            // }
            this->now_box_3D[i].x_width = this->pre_box_3D_history[i].back().x_width;
            this->now_box_3D[i].y_width = this->pre_box_3D_history[i].back().y_width;
            this->now_box_3D[i].z_width = this->pre_box_3D_history[i].back().z_width;


            // keep queue sizes for all queues
            this->pre_history[i].erase(this->pre_history[i].begin());

        }
    }   

    // box_3D = this->now_box_3D;

    // keep queue sizes for all queues
    // for (int i=0 ; i<this->pre_V.size() ; i++) {
    //     if (this->pre_V[i].size() > 10) {
    //         this->pre_V[i].pop_front();
    //         double sum_Vx, sum_Vy;
    //         for (int j=0 ; j<this->pre_V[i].size() ; j++) {
    //             sum_Vx += this->pre_V[i][j](0,0);
    //             sum_Vy += this->pre_V[i][j](1,0);
    //         }
            
    //         // printf("queue size: %d\n",this->pre_V[i].size());
    //         // if (this->pre_V[i].size()>10)
    //         // {
    //         //     printf("++++++++++++++++++++++++++++++++++++++++++++++queue size: %d\n",this->pre_V[i].size());
    //         // }
    //         // printf("v of box %d in birdview 10mm/s: %f, %f\n",i,sum_Vx/10.0,sum_Vy/10.0 );
    //         if (std::sqrt(std::pow(sum_Vx/10.0,2) + std::pow(sum_Vy/10.0,2)) < 10) {
    //             printf("static! %f, %f\n",sum_Vx/10.0,sum_Vy/10.0);
    //         }
    //         else {
    //             printf("move ! %f, %f\n",sum_Vx/10.0,sum_Vy/10.0);
    //         }

    //     }
    // }

    // printf(" \n");
    
}
   
void UVtracker::check_status(vector<box3D> &box_3D)
{
    int f = 30; // Hz
    double ts = 1.0 / f; // s
    for(int now_id = 0; now_id < this->now_bb.size(); now_id++)
    {
        bool tracked = false;
        for(int pre_id = 0; pre_id < this->pre_bb.size(); pre_id++)
        {
            Rect overlap = this->now_bb[now_id] & this->pre_bb[pre_id];
           
            float dist = std::sqrt( std::pow((this->now_bb[now_id].x + 0.5 * this->now_bb[now_id].width)-(this->pre_bb[pre_id].x + 0.5 * this->pre_bb[pre_id].width),2) + std::pow((this->now_bb[now_id].y + 0.5 * this->now_bb[now_id].height-(this->pre_bb[pre_id].y + 0.5 * this->pre_bb[pre_id].height)),2) );
            float metric = std::sqrt( std::pow(this->now_bb[now_id].width+this->pre_bb[pre_id].width,2) + std::pow(this->now_bb[now_id].height+this->pre_bb[pre_id].height,2) )/2;
            // printf("now: %f, %f, %f, %f\n",this->now_bb[now_id].x,this->now_bb[now_id].y,this->now_bb[now_id].width,this->now_bb[now_id].height);
            
            // printf("dist : %f\n",dist);
            // printf("metric: %f\n", metric );
            if(max(overlap.area() / float(this->now_bb[now_id].area()), overlap.area() / float(this->pre_bb[pre_id].area())) >= this->overlap_threshold || dist<=metric)
            {
                tracked = true;
                std::cout<<"tracked"<<std::endl;
                // if (now_id != pre_id) {
                //     printf("TRACK PAIR: %d ,%d",now_id, pre_id);
                // }
                
                // inherit add current detection to history
                this->now_history[now_id] = this->pre_history[pre_id];
                this->now_history[now_id].push_back(Point2f(this->now_bb[now_id].x + 0.5 * this->now_bb[now_id].width, this->now_bb[now_id].y + 0.5 * this->now_bb[now_id].height));
                // update fix flag
                // this->now_fix[now_id] = this->pre_fix[pre_id];
                // inherit 3d box history 
                this->now_box_3D_history[now_id] = this->pre_box_3D_history[pre_id];
                // add current 3D box to box history only if when it is full in the FOV. otherwise, clear the history and track again
                if (this->now_bb_D[now_id].tl().x>5 && this->now_bb_D[now_id].tl().y>5 && this->now_bb_D[now_id].br().x<635 && this->now_bb_D[now_id].br().y<475) {
                    this->now_box_3D_history[now_id].push_back(this->now_box_3D[now_id]);
                }
                // else {
                //     if (this->pre_box_3D_history.size() > now_id) {
                //         this->pre_box_3D_history[now_id].clear();
                //     }
                // }

                // inherit velocity prediction  
                this->now_V[now_id] = this->pre_V[pre_id];
                // add measurement to previous filter
                this->now_filter[now_id] = this->pre_filter[pre_id];
                MatrixXd z(6,1); // measurement
                // printf("z=x,y,w,h: %f, %f, %f, %f\n",this->now_bb[now_id].x + 0.5 * this->now_bb[now_id].width, this->now_bb[now_id].y + 0.5 * this->now_bb[now_id].height, this->now_bb[now_id].width, this->now_bb[now_id].height);
                // printf("input center %f, %f\n",this->now_bb[now_id].x + 0.5 * this->now_bb[now_id].width, this->now_bb[now_id].y + 0.5 * this->now_bb[now_id].height);
                // calculate velocity of box centor from pre to now as the measrued velocity
                double z_vx =((this->now_bb[now_id].x + 0.5 * this->now_bb[now_id].width) - (this->pre_bb[pre_id].x + 0.5 * this->pre_bb[pre_id].width))/ts;
                double z_vy =((this->now_bb[now_id].y + 0.5 * this->now_bb[now_id].height) - (this->pre_bb[pre_id].y + 0.5 * this->pre_bb[pre_id].height))/ts;
                double z_cx = this->now_bb[now_id].x + 0.5 * this->now_bb[now_id].width;
                double z_cy = this->now_bb[now_id].y + 0.5 * this->now_bb[now_id].height;
                double z_bx = this->now_bb[now_id].width;
                double z_by = this->now_bb[now_id].height;
                // printf("now %d:x,y,w,h %f,%f,%f,%f\n",now_id, float(this->now_bb[now_id].x),float(this->now_bb[now_id].y),float(this->now_bb[now_id].width),float(this->now_bb[now_id].height));
                // printf("pre %d:x,y,w,h %f,%f,%f,%f\n",pre_id, float(this->pre_bb[pre_id].x),float(this->pre_bb[pre_id].y),float(this->pre_bb[pre_id].width),float(this->pre_bb[pre_id].height));
                // printf("input %d : %f,%f\n",now_id, z_vx,z_vy);
                z << z_cx, z_cy, z_vx, z_vy, z_bx, z_by;
                MatrixXd u(1,1); // input
                u << 0;   
                // printf("filter states before: %f,%f,%f,%f\n",this->now_filter[now_id].output(0),this->now_filter[now_id].output(1),this->now_filter[now_id].output(2),this->now_filter[now_id].output(3));
                // run the filter 
                this->now_filter[now_id].estimate(z, u);    
                // this->now_filter[now_id].states(2,0) = (this->now_filter[now_id].states(0,0) - this->pre_filter[pre_id].states(0,0))/ts;
                // this->now_filter[now_id].states(3,0) = (this->now_filter[now_id].states(1,0) - this->pre_filter[pre_id].states(1,0))/ts;
                MatrixXd V(2,1);
                V << this->now_filter[now_id].output(2),this->now_filter[now_id].output(3);
                // printf("before add: %d ,%d, %d\n",now_bb.size(), pre_bb.size(), this->now_V[now_id].size());
                // printf("add now id:%d v:%f, %f\n",now_id,V(0,0),V(1,0));
                this->now_V[now_id].push_back(V);
                // printf("filter states after: %f,%f,%f,%f\n",this->now_filter[now_id].output(0),this->now_filter[now_id].output(1),this->now_filter[now_id].output(2),this->now_filter[now_id].output(3));
                break;
            } 
            else {
                // printf("ratio: %f, %f\n",overlap.area() / float(this->now_bb[now_id].area()),overlap.area() / float(this->pre_bb[pre_id].area()) );
                // printf("dist : %f\n",dist);
                // printf("metric: %f\n", metric );
            }
        }
        if(!tracked)    
        {
            std::cout<<now_id<<" LOSS TRACK\n"<<std::endl;
            // add current detection to history
            this->now_history[now_id].push_back(Point2f(this->now_bb[now_id].x + 0.5 * this->now_bb[now_id].width, this->now_bb[now_id].y + 0.5 * this->now_bb[now_id].height));
            // this->now_fix[now_id] = true;
            MatrixXd V(2,1);
            V << 0.0, 0.0;
            this->now_V[now_id].push_back(V);
            // add current 3D box to box history only if when it is full in the FOV. otherwise, clear the history and track again
            if (this->now_bb_D[now_id].tl().x>5 && this->now_bb_D[now_id].tl().y>5 && this->now_bb_D[now_id].br().x<635 && this->now_bb_D[now_id].br().y<475) {
                this->now_box_3D_history[now_id].push_back(this->now_box_3D[now_id]);
            }
            // else {
            //     if (this->pre_box_3D_history.size() > now_id) {
            //         this->pre_box_3D_history[now_id].clear();
            //     }
                
            // }
            // initialize filter
            
            
            // model for center filter
            double e_p = 0.4;
            double e_ps = 0.0;
            double e_m = 0.3;
            double e_ms = 0.99;
            MatrixXd A(6, 6);
            A <<    1, 0, ts, 0, 0, 0, 
                    0, 1, 0, ts, 0, 0,
                    0, 0, 1, 0,  0, 0,
                    0, 0, 0, 1,  0, 0,
                    0, 0, 0, 0,  1, 0,
                    0, 0, 0, 0,  0, 1; 
            MatrixXd B(6, 1);
            B <<    0, 0, 0, 0, 0, 0;
            MatrixXd H(6,6);
            H <<    1, 0, 0, 0, 0, 0,
                    0, 1, 0, 0, 0, 0,
                    0, 0, 1, 0, 0, 0,
                    0, 0, 0, 1, 0, 0,
                    0, 0, 0, 0, 1, 0,
                    0, 0, 0, 0, 0, 1;
            MatrixXd P = MatrixXd::Identity(6, 6) * e_m;
            P(4,4) = e_ms; P(5,5) = e_ms;
            MatrixXd Q = MatrixXd::Identity(6, 6) * e_p;
            Q(4,4) = e_ps; Q(5,5) = e_ps;
            // MatrixXd R = MatrixXd::Identity(4, 4) * e_m;
            MatrixXd R = MatrixXd::Identity(6, 6) * e_m;

            // filter initialization
            MatrixXd states(6,1);
            // printf("input center %f, %f\n",this->now_bb[now_id].x + 0.5 * this->now_bb[now_id].width, this->now_bb[now_id].y + 0.5 * this->now_bb[now_id].height);
            states << this->now_bb[now_id].x + 0.5 * this->now_bb[now_id].width, this->now_bb[now_id].y + 0.5 * this->now_bb[now_id].height, 0, 0, this->now_bb[now_id].width, this->now_bb[now_id].height;
            kalman_filter my_filter;
            this->now_filter[now_id] = my_filter;
            this->now_filter[now_id].setup(states,
                            A,
                            B,
                            H,
                            P,     
                            Q,
                            R);
            // printf("filter states init: %f,%f,%f,%f",this->now_filter[now_id].output(0),this->now_filter[now_id].output(1),this->now_filter[now_id].output(2),this->now_filter[now_id].output(3));
        }
    }

    for (int i=0 ; i<this->now_V.size() ; i++) {
        if (this->now_V[i].size() > 10) {
            this->now_V[i].pop_front();
        }
        double sum_Vx = 0.0;
        double sum_Vy = 0.0;
        // printf("sum_Vx before: %f\n",sum_Vx);
        for (int j=0 ; j<this->now_V[i].size() ; j++) {
            sum_Vx += this->now_V[i][j](0,0);
            sum_Vy += this->now_V[i][j](1,0);
            // printf(" %f ",this->now_V[i][j](0,0));
            // if (this->now_V.size() < 10) {
            //     printf("v stack %d: %f, %f\n",i,this->now_V[i][j](0,0),this->now_V[i][j](1,0));
            // }
        }
        
        // printf("queue size: %d\n",this->pre_V[i].size());
        if (this->now_V[i].size()>10)
        {
            printf("++++++++++++++++++++++++++++++++++++++++++++++queue size: %d\n",this->now_V[i].size());
        }
        // printf("v of box %d in birdview 10mm/s: %f, %f\n",i,sum_Vx/10.0,sum_Vy/10.0 );
        
        // take average, convert 10mm/s to m/s
        // printf("sum_Vx after: %f\n",sum_Vx);
        // printf("queue length: %d\n",this->now_V[i].size());
        box_3D[i].Vx = (sum_Vx/this->now_V[i].size())/100;
        box_3D[i].Vy = (sum_Vy/this->now_V[i].size())/100;
        if (std::sqrt(std::pow(box_3D[i].Vx,2) + std::pow(box_3D[i].Vy,2)) < 0.3) {
            printf("%d static! %f, %f\n",i,box_3D[i].Vx,box_3D[i].Vy);
        }
        else {
            printf("%d move ! %f, %f\n",i,box_3D[i].Vx,box_3D[i].Vy);
        }
    }

}  

// UVdetector     

UVdetector::UVdetector()
{
    this->row_downsample = 4;
    this->col_scale = 0.5;
    this->min_dist = 10;
    this->max_dist = 5000;
    this->threshold_point = 3;
    this->threshold_line = 2;
    this->min_length_line = 6;
    this->show_bounding_box_U = true;
    // this->row_downsample = 2;
    // this->col_scale = 0.5;
    // this->min_dist = 10;
    // this->max_dist = 5000;
    // this->threshold_point = 5;
    // this->threshold_line = 2;
    // this->min_length_line = 6;
    this->show_bounding_box_U = true;
    // the following intrinsic parameters can be found in /camera/../camera_info
    this->fx = 608.08740234375;
    this->fy = 608.1791381835938;
    this->px = 317.48284912109375;
    this->py = 234.11557006835938;

    this->x0 = 0;
    this->y0 = 0;
}

void UVdetector::readdata(queue<Mat> depthq)
{
    this->depth = max(depthq.front(), depthq.back());
    double minVal; 
    double maxVal; 
    Point minLoc; 
    Point maxLoc;
    // resize(this->depth, this->depth, Size(50,50));
    minMaxLoc( this->depth, &minVal, &maxVal, &minLoc, &maxLoc );   
}

void UVdetector::readdepth(cv::Mat depth){
    this->depth = depth;
    double minVal; 
    double maxVal; 
    Point minLoc; 
    Point maxLoc;

    minMaxLoc( this->depth, &minVal, &maxVal, &minLoc, &maxLoc ); 
}

void UVdetector::readrgb(Mat RGB)
{
    this->RGB = RGB;
    resize(this->RGB, this->RGB, Size(720,400));
    // imshow("RGB", this->RGB);
}

void UVdetector::extract_U_map()
{
    // rescale depth map
    Mat depth_rescale;
    
    resize(this->depth, depth_rescale, Size(),this->col_scale , 1);
    Mat depth_low_res_temp = Mat::zeros(depth_rescale.rows, depth_rescale.cols, CV_8UC1);

    

    // construct the mask
    Rect mask_depth;
    uint8_t histSize = this->depth.rows / this->row_downsample;

    int bin_width = ceil((this->max_dist - this->min_dist) / float(histSize));

    this->U_map = Mat::zeros(histSize, depth_rescale.cols, CV_8UC1);

    for(int col = 0; col < depth_rescale.cols; col++)
    {
        for(int row = 0; row < depth_rescale.rows; row++)
        {
            if(depth_rescale.at<unsigned short>(row, col) > this->min_dist && depth_rescale.at<unsigned short>(row, col) < this->max_dist)
            {
                uint8_t bin_index = (depth_rescale.at<unsigned short>(row, col) - this->min_dist) / bin_width;
                depth_low_res_temp.at<uchar>(row, col) = bin_index;
                if(this->U_map.at<uchar>(bin_index, col) < 255)
                {
                    this->U_map.at<uchar>(bin_index, col) ++;
                }
            }
        }
    }
    this->depth_low_res = depth_low_res_temp;

    // smooth the U map
    
    GaussianBlur(this->U_map, this->U_map, Size(5,9), 10, 10);
    // printf("rescaled depth map: %d", depth_rescale.at<unsigned short>(testy,testx));
}

void UVdetector::extract_bb()
{
    // initialize a mask
    vector<vector<int> > mask(this->U_map.rows, vector<int>(this->U_map.cols, 0));
    // initialize parameters
    int u_min = this->threshold_point * this->row_downsample;
    int sum_line = 0;
    int max_line = 0;
    int length_line = 0;
    int seg_id = 0;
    vector<UVbox> UVboxes;
    for(int row = 0; row < this->U_map.rows; row++)
    {
        for(int col = 0; col < this->U_map.cols; col++)
        {
            // is a point of interest
            if(this->U_map.at<uchar>(row,col) >= u_min) // num of points at this depth >= u_min
            {
                // update current line info
                length_line++;
                sum_line += this->U_map.at<uchar>(row,col);
                max_line = (this->U_map.at<uchar>(row,col) > max_line)?this->U_map.at<uchar>(row,col):max_line;
            }
            // is not or is end of row
            if(this->U_map.at<uchar>(row,col) < u_min || col == this->U_map.cols - 1)
            {
                // is end of the row
                col = (col == this->U_map.cols - 1)? col + 1:col;
                // is good line candidate (length and sum)
                if(length_line > this->min_length_line && sum_line > this->threshold_line * max_line)
                {
                    seg_id++;
                    UVboxes.push_back(UVbox(seg_id, row, col - length_line, col - 1));
                    // overwrite the mask with segementation id
                    for(int c = col - length_line; c < col - 1; c++)
                    {
                        mask[row][c] = seg_id;
                    }
                    // when current row is not first row, we need to merge neighbour segementation
                    if(row != 0)
                    {
                        // merge all parents
                        for(int c = col - length_line; c < col - 1; c++)
                        {
                            if(mask[row - 1][c] != 0)
                            {
                                // initially, toppest parent id is just seg id.
                                // mask stores seg_ids
                                if(UVboxes[mask[row - 1][c] - 1].toppest_parent_id < UVboxes.back().toppest_parent_id)
                                {
                                    UVboxes.back().toppest_parent_id = UVboxes[mask[row - 1][c] - 1].toppest_parent_id;
                                }
                                else
                                {
                                    int temp = UVboxes[mask[row - 1][c] - 1].toppest_parent_id;
                                    for(int b = 0; b < UVboxes.size(); b++)
                                    {
                                        UVboxes[b].toppest_parent_id = (UVboxes[b].toppest_parent_id == temp)?UVboxes.back().toppest_parent_id:UVboxes[b].toppest_parent_id;
                                    }
                                }
                            }
                        }
                    }
                }
                sum_line = 0;
                max_line = 0;
                length_line = 0;
            }
        }
    }
    
    // group lines into boxes
    this->bounding_box_U.clear();
    // merge boxes with same toppest parent
    for(int b = 0; b < UVboxes.size(); b++)
    {
        if(UVboxes[b].id == UVboxes[b].toppest_parent_id)
        {
            for(int s = b + 1; s < UVboxes.size(); s++)
            {
                if(UVboxes[s].toppest_parent_id == UVboxes[b].id)
                {
                    UVboxes[b] = merge_two_UVbox(UVboxes[b], UVboxes[s]);

                }
            }
            // check box's size
            if(UVboxes[b].bb.area() >= 25)
            {
                this->bounding_box_U.push_back(UVboxes[b].bb);
            }
        }
    }
}

void UVdetector::detect()
{
    // extract U map from depth
    this->extract_U_map();

    // extract bounding box from U map
    this->extract_bb();

    // extract bounding box
    this->extract_bird_view();

    // extract object's height
    // this->extract_height();
}  
void UVdetector::display_depth()
{
    // in order to get a better visualization, we need normalized depth map in range (0, 255)
    Mat depth_normalized;
    this->depth.copyTo(depth_normalized);
    double min, max;
    cv::minMaxIdx(depth_normalized, &min, &max);
    cv::convertScaleAbs(depth_normalized, depth_normalized, 255. / max);
    depth_normalized.convertTo(depth_normalized, CV_8UC1);
    applyColorMap(depth_normalized, depth_normalized, COLORMAP_JET);

    // loop for adding bounding boxes
    for (int i=0;i<this->bounding_box_D.size();i++){
        rectangle(depth_normalized, bounding_box_D[i], cv::Scalar(0, 0, 255), 5, 8, 0);
       // printf("bbox %d br.x coord %d, tl.x coord %d\n",i , bounding_box_D[i].br().x, bounding_box_D[i].tl().x);
    }
//    imshow("Depth", depth_normalized);
    waitKey(1);
}

void UVdetector::extract_3Dbox()
{   
    // this function returns 3D boxes in world frame for publishing
    Mat depth_resize;
    resize(depth, depth_resize, Size(), this->col_scale, 1);
    float histSize = this->depth.rows / this->row_downsample;
    float bin_width = ceil((this->max_dist - this->min_dist) / histSize);
    
    int x;
    int y_up;
    int y_down;
    int width;
    int bin_index_small;
    int bin_index_large;
    float depth_in_near;
    float depth_of_depth;
    float depth_in_far;

    int im_frame_x;
    int im_frame_x_width;
    int im_frame_y;
    int im_frame_y_width;

    // GaussianBlur(depth_resize, depth_resize, Size(5,9), 0, 0);

    // parameter for tunning
    int num_check  = 15;

    this->box3Ds.clear();
    this->bounding_box_D.clear();

    for (int b = 0; b < this->bounding_box_U.size(); b++) {
        // 
        x = this->bounding_box_U[b].tl().x;
        width = this->bounding_box_U[b].width;

        y_up = depth_resize.rows;
        y_down = 0;
        bin_index_small = this->bounding_box_U[b].tl().y;
        bin_index_large = this->bounding_box_U[b].br().y;
        depth_in_near = (bin_index_small * bin_width + this->min_dist);
        depth_of_depth = (bin_index_large - bin_index_small) * bin_width;
        depth_in_far = depth_of_depth*1.8 + depth_in_near; // assumed depth was truncated because of occlusion

        // for debugging
        // cout << "-------------" << endl;
        // cout << depth_in_near << endl;
        // cout << depth_in_far << endl;
        // cout << "-------------" << endl;


        for (int i = x ; i < x + width; i++) { // for several middle coloums
            for (int j = 0; j < depth_resize.rows - 1; j++) { // for each row
                if (float(depth_resize.at<unsigned short>(j, i)) >= depth_in_near && float(depth_resize.at<unsigned short>(j, i)) <= depth_in_far) {
                    for (int check = 0; check < num_check; check++) { // check some more points in the coloum
                        if (float(depth_resize.at<unsigned short>(j + check + 1, i)) < depth_in_near || 
                        float(depth_resize.at<unsigned short>(j + check + 1, i)) > depth_in_far) {
                            // bad case
                            break;
                        }
                        if (check == num_check-1) {
                            if (y_up > j) y_up = j;
                            if (y_down < j) y_down = j;
                        }
                    }
                }
            }
        }

        // save bounding boxes in depth
        float bb_x = x / this->col_scale;
        float bb_width = width / this->col_scale;
        float bb_y = y_up;
        float bb_height = y_down-y_up;
        this->bounding_box_D.push_back(Rect(bb_x, bb_y, bb_width, bb_height));
    
        box3D curr_box;
        // extract x,y coordinates in input depth frame
        im_frame_x  = (x + width / 2) / this->col_scale;
        im_frame_x_width = width / this->col_scale;
        
        int Y_w = (depth_in_near + depth_in_far) / 2;
        im_frame_y = (y_down + y_up) / 2;
        im_frame_y_width = y_down - y_up;

        // x, y coord in original full image, instead of crop
        im_frame_x += x0;
        im_frame_y += y0;
        
        testx = im_frame_x;
        testy = im_frame_y;
        testby = bin_index_small;
        // image frame to world frame transformation
        // x in world frame is the depth direction, y in world frame is -x, z in world frame 
        // is align with -y in image frame 
        curr_box.y =  -(im_frame_x-this->px)*Y_w/this->fx;
        curr_box.z = -(im_frame_y-this->py)*Y_w/this->fy;
        curr_box.y_width = im_frame_x_width*Y_w/this->fx;
        curr_box.z_width = im_frame_y_width*Y_w/this->fy;
        curr_box.x = Y_w;
        curr_box.x_width = depth_in_far-depth_in_near;// depth of depth originally
        box3Ds.push_back(curr_box);
        // printf("depth in near: %f \n",depth_in_near);
    }
}
      
// void UVdetector::display_RGB()
// {

// }
void UVdetector::display_U_map()
{
    // visualize with bounding box
    if(this->show_bounding_box_U)
    {
        this->U_map = this->U_map * 10;
        cvtColor(this->U_map, this->U_map, cv::COLOR_GRAY2RGB);
        for(int b = 0; b < this->bounding_box_U.size(); b++)
        {
            Rect final_bb = Rect(this->bounding_box_U[b].tl(),Size(this->bounding_box_U[b].width, 2 * this->bounding_box_U[b].height));
            rectangle(this->U_map, final_bb, Scalar(0, 0, 255), 1, 8, 0);
            circle(this->U_map, Point2f(this->bounding_box_U[b].tl().x + 0.5 * this->bounding_box_U[b].width, this->bounding_box_U[b].br().y ), 2, Scalar(0, 0, 255), 5, 8, 0);
        }
    } 
    waitKey(1);
}

// unit is 10 mm?
// x, y is bottome left point
void UVdetector::extract_bird_view()
{
    // extract bounding boxes in bird's view
    uint8_t histSize = this->depth.rows / this->row_downsample;
    uint8_t bin_width = ceil((this->max_dist - this->min_dist) / float(histSize));
    this->bounding_box_B.clear();
    this->bounding_box_B.resize(this->bounding_box_U.size());

    for(int b = 0; b < this->bounding_box_U.size(); b++)
    {
        // U_map bounding box -> Bird's view bounding box conversion
        float bb_depth = this->bounding_box_U[b].br().y * bin_width / 10;
        float bb_width = bb_depth * this->bounding_box_U[b].width / this->fx;
        float bb_height = this->bounding_box_U[b].height * bin_width / 10;
        float bb_x = bb_depth * (this->bounding_box_U[b].tl().x / this->col_scale - this->px) / this->fx;
        float bb_y = bb_depth - 0.5 * bb_height;// assume farthest depth value is the depth of center point, then assume detected depth difference is the depth of the whole body. y is the depth direction
        this->bounding_box_B[b] = Rect(bb_x, bb_y, bb_width, bb_height);
    }

    // initialize the bird's view
    this->bird_view = Mat::zeros(500, 1000, CV_8UC1);
    cvtColor(this->bird_view, this->bird_view, cv::COLOR_GRAY2RGB);
}

void UVdetector::display_bird_view() 
{
    // center point
    Point2f center = Point2f(this->bird_view.cols / 2, this->bird_view.rows);
    Point2f left_end_to_center = Point2f( this->bird_view.rows * (0 - this->px) / this->fx, -this->bird_view.rows);
    Point2f right_end_to_center = Point2f( this->bird_view.rows * (this->depth.cols - this->px) / this->fx, -this->bird_view.rows);

    // draw the two side lines
    line(this->bird_view, center, center + left_end_to_center, Scalar(0, 255, 0), 3, 8, 0);
    line(this->bird_view, center, center + right_end_to_center, Scalar(0, 255, 0), 3, 8, 0);

    for(int b = 0; b < this->bounding_box_U.size(); b++)
    {
        // change coordinates
        Rect final_bb = this->bounding_box_B[b];
        final_bb.y = center.y - final_bb.y - final_bb.height;
        final_bb.x = final_bb.x + center.x; 
        // draw center 
        Point2f bb_center = Point2f(final_bb.x + 0.5 * final_bb.width, final_bb.y + 0.5 * final_bb.height);
        rectangle(this->bird_view, final_bb, Scalar(0, 0, 255), 3, 8, 0);
        circle(this->bird_view, bb_center, 3, Scalar(0, 0, 255), 5, 8, 0);
    }

    // show
    resize(this->bird_view, this->bird_view, Size(), 0.5, 0.5);
    imshow("Bird's View", this->bird_view);
    waitKey(1);
}

void UVdetector::add_tracking_result()
{
    Point2f center = Point2f(this->bird_view.cols / 2, this->bird_view.rows);
    for(int b = 0; b < this->tracker.now_bb.size(); b++)
    {
        // change coordinates
        Point2f estimated_center = Point2f(this->tracker.now_filter[b].output(0), this->tracker.now_filter[b].output(1));
        estimated_center.y = center.y - estimated_center.y;
        estimated_center.x = estimated_center.x + center.x; 
        // draw center 
        circle(this->bird_view, estimated_center, 5, Scalar(0, 255, 0), 5, 8, 0);
        // draw bounding box
        Point2f bb_size = Point2f(this->tracker.now_filter[b].output(4), this->tracker.now_filter[b].output(5));
        rectangle(this->bird_view, Rect(estimated_center - 0.5 * bb_size, estimated_center + 0.5 * bb_size), Scalar(0, 255, 0), 3, 8, 0);
        // draw velocity
        Point2f velocity = Point2f(this->tracker.now_filter[b].output(2), this->tracker.now_filter[b].output(3));
        velocity.y = -velocity.y;// y direction in birdvie map is in opposite of opencv::line default
        // printf("velocity in birdview 10mm/s: %f, %f , center x ,y: %f, %f, bbox size x, y:%f, %f\n", velocity.x, -velocity.y, estimated_center.x, estimated_center.y, bb_size.x, bb_size.y);
        line(this->bird_view, estimated_center, estimated_center + velocity, Scalar(255, 255, 255), 3, 8, 0);
        for(int h = 1; h < this->tracker.now_history[b].size(); h++)
        {
            // trajectory
            Point2f start = this->tracker.now_history[b][h - 1];
            start.y = center.y - start.y;
            start.x = start.x + center.x;
            Point2f end = this->tracker.now_history[b][h];
            end.y = center.y - end.y;
            end.x = end.x + center.x;
            line(this->bird_view, start, end, Scalar(0, 0, 255), 3, 8, 0);
        }
    }
}
   
void UVdetector::track()
{
    // float before = this->box3Ds[0].x_width;
    this->tracker.read_bb(this->bounding_box_B, this->bounding_box_D, this->box3Ds);
    this->tracker.check_status(this->box3Ds);
    this->add_tracking_result();
    // float after = this->box3Ds[0].x_width;
    // printf("verify : %f, %f", before, after);
}
