#include <iostream>
#include "mv_camera.hpp"
#include <eigen3/Eigen/Dense>

int main() {
    std::cout << "Hello,world！" << std::endl;
    Devices::MV_Camera c;
    c.open();
    cv::Mat read_img;
    cv::Mat img;
    namedWindow("test", cv::WINDOW_NORMAL);

    cv::Mat cameraMatrix;
    cv::Mat distCoeffs;

    cameraMatrix = (cv::Mat_<double>(3,3) << 1562.718391961961,    4.478471970184, 616.284509563135,
                                                             0, 1563.803986201450, 489.040600564709,
                                                             0,                 0,                1);                 
    
    distCoeffs = (cv::Mat_<double>(5,1) << -0.088250992965441, 0.154271559512706, -0.000829270875611, -0.001394637877056, 0);

    cv::Mat gray,mask,med;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    std::vector<cv::Point3f> shijie(4);
    std::vector<cv::Point2f> xiangji(4);

    cv::RotatedRect target_rect;
    cv::Point2f pts[4];
    cv::Point2f pts1[4];
    cv::Point2f pts2[4];
    cv::Point2f top1;
    cv::Point2f bottom1;
    cv::Point2f top2;
    cv::Point2f bottom2;




    double chang = 0.148;
    double kuan = 0.062;
    chang = chang/2;
    kuan = kuan/2;
    cv::Point3f wtop1(-chang,kuan,0);
    cv::Point3f wtop2(chang,kuan,0);
    cv::Point3f wbottom1(-chang,-kuan,0);
    cv::Point3f wbottom2(chang,-kuan,0);

    cv::Mat rvec;
    cv::Mat tvec;

    cv::Mat rotM;
    cv::Mat rotT;

    double theta_x;
    double theta_y;
    double theta_z;

    auto last_time = std::chrono::system_clock::now();
    auto new_time = std::chrono::system_clock::now();

    double k1,k2,b1,b2;
    cv::Point2f centre;

    //扩展卡尔曼变量
    Eigen::Matrix<double,6,1> X;
    //      X  vx   y  vy   z  vz
    X <<    0,  0,  0,  0,  0,  0;
    //上一次运算坐标位置
    Eigen::Matrix<double,3,1> X1;
    //     lx  ly  lz  
    X1 <<   0,  0,  0;

    //测量值矩阵
    Eigen::Matrix<double,3,1> X2;
    




    // Eigen::Matrix<double,6,6> P;
    Eigen::Matrix<double,6,6> P;
    P.setIdentity();

    // Eigen::Matrix<double,6,6> F;
    // Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> F; 这种自动大小的不能用
    Eigen::Matrix<double,6,6> F;
    F.setIdentity();

    //加速度a(待测量估计)
    double a = 0.02;
    double variable1;
    double variable2;
    double variable3;



    // Eigen::Matrix<double,6,6> Q;
    Eigen::Matrix<double,6,6> Q;
    Q.setIdentity();


    Eigen::Matrix<double,Eigen::Dynamic,Eigen::Dynamic> K;

    // Eigen::Matrix<double,3,6> H;
    // Eigen::Matrix<double,6,6> R;
    Eigen::Matrix<double,3,6> H;
    Eigen::Matrix<double,3,3> R;

    //单位矩阵
    Eigen::Matrix<double,6,6> I;
    I.setIdentity();

    //定义坐标变换输入坐标
    cv::Mat dian3 = (cv::Mat_<double>(3,1)<<0,0,0);
    cv::Mat dian2;

    cv::Point2f jieguo;

    cv::Mat X3(3,1,CV_64F);
    //test
    cv::Mat dian22;
    cv::Point2f jieguo1;






    


    while(true)
    {   
        auto start_time = std::chrono::system_clock::now();
        c.read(read_img);
        img = read_img.clone();
        cv::cvtColor(img,gray,cv::COLOR_BGR2GRAY);
        
        cv::medianBlur(gray,gray,3);
        cv::Scalar lower = 160;
        cv::Scalar upper = 255;
        cv::inRange(gray,lower,upper,mask);
        cv::findContours(mask, contours, hierarchy, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

        if(contours.size()>1)
            {
                cv::RotatedRect rect1 = cv::minAreaRect(contours[0]); 
                rect1.points(pts1);
                std::sort(pts1,pts1 + 4, [](const cv::Point2f & a,const cv::Point & b){return a.y < b.y;});
                top1 = (pts1[0] + pts1[1]) / 2;
                bottom1 = (pts1[2] + pts1[3]) / 2;
                cv::RotatedRect rect2 = cv::minAreaRect(contours[1]);
                rect2.points(pts2);
                std::sort(pts2,pts2 + 4, [](const cv::Point2f & a,const cv::Point & b){return a.y < b.y;});
                top2 = (pts2[0] + pts2[1]) / 2;
                bottom2 = (pts2[2] + pts2[3]) / 2;

                cv::line(img,top1,bottom2,cv::Scalar(0,255,0),1,8,0);
                cv::line(img,top2,bottom1,cv::Scalar(0,255,0),1,8,0);

                if(top1.x>top2.x){
                    cv::Point2f top3;
                    top3 = top1;
                    top1 = top2;
                    top2 = top3;
                    cv::Point2f bottom3;
                    bottom3 = bottom1;
                    bottom1 = bottom2;
                    bottom2 = bottom3;


                }

                xiangji[0] = top1;
                xiangji[1] = top2;
                xiangji[2] = bottom2;
                xiangji[3] = bottom1;

                shijie[0] = wtop1;
                shijie[1] = wtop2;
                shijie[2] = wbottom2;
                shijie[3] = wbottom1;

                cv::solvePnP(shijie,xiangji,cameraMatrix,distCoeffs,rvec,tvec,false,cv::SOLVEPNP_ITERATIVE);
                // std::cout<<"revc : "<<std::endl<<tvec.at<double>(2,0)<<std::endl;
                // std::cout<<"juli is  : "<<std::endl<<(tvec.at<double>(2,0))<<std::endl;
                // std::cout<<tvec<<std::endl;


                cv::putText(img,"0",xiangji[0],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0, 0, 255),4,8);
                cv::putText(img,"1",xiangji[1],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0, 0, 255),4,8);
                cv::putText(img,"2",xiangji[2],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0, 0, 255),4,8);
                cv::putText(img,"3",xiangji[3],cv::FONT_HERSHEY_SIMPLEX,1,cv::Scalar(0, 0, 255),4,8);


                //求两直线交点centre
                k1 = (top1.y - bottom2.y)/(top1.x - bottom2.x);
                b1 = top1.y - k1*top1.x;
                k2 = (bottom1.y - top2.y)/(bottom1.x - top2.x);
                b2 = bottom1.y - k2*bottom1.x;

                centre.x = (b2 - b1)/(k1 - k2);
                centre.y = k1*centre.x + b1;

                // //屏幕中心点
                // cv::Point2f screen_centre;
                // screen_centre.X = img.cols/2;
                // screen_centre.y = img.rows/2;
                // cv::circle(img,screen_centre,10,cv::Scalar(0,0,255),-1);


                

                cv::circle(img,centre,10,cv::Scalar(0,255,0),-1);

                //时间戳
                auto end_time = std::chrono::system_clock::now();
        
                new_time = end_time;
                // auto waste_time = std::chrono::duration_cast<std::chrono::microseconds>(end_time-start_time).count();
                // auto waste_time = std::chrono::duration_cast<std::chrono::milliseconds>(end_time-start_time).count();
                auto waste_time = std::chrono::duration_cast<std::chrono::microseconds>(new_time-last_time).count();
                last_time = end_time;
                double t = waste_time * 0.000001;
        
                // std::cout << t << std::endl;


                //扩展卡尔曼//

                //预测方程
                X(1,0) = (tvec.at<double>(0,0) - X1(0,0)) / t;
                X(3,0) = (tvec.at<double>(1,0) - X1(1,0)) / t;
                X(5,0) = (tvec.at<double>(2,0) - X1(2,0)) / t;

                //test
                // std::cout << X(0,1) <<std::endl;
                
                // X1(0,0) = X(0,0);
                // X1(0,1) = X(0,2);
                // X1(0,2) = X(0,4);

                

                X(0,0) = X(0,0) + X(1,0) * 0.1;
                X(2,0) = X(2,0) + X(3,0) * 0.1;
                X(4,0) = X(4,0) + X(5,0) * 0.1;

                
                //画出滤波后的点
                X3.at<double>(0,0) = X(0,0);
                X3.at<double>(1,0) = X(2,0);
                X3.at<double>(2,0) = X(4,0);
                //test
                std::cout<<"tvec is :"<<std::endl<<tvec<<std::endl;
                std::cout<<"X3 is :"<<std::endl<<X3<<std::endl;

                

                cv::projectPoints(dian3,rvec,X3,cameraMatrix,distCoeffs,dian2);
                //test
                cv::projectPoints(dian3,rvec,tvec,cameraMatrix,distCoeffs,dian22);
                jieguo1.x = dian22.at<double>(0,0);
                jieguo1.y = dian22.at<double>(0,1);
                cv::circle(img,jieguo1,10,cv::Scalar(255,0,0),-1);
                //test
                // std::cout<<"dian is :"<<std::endl<<dian2<<std::endl;
                // std::cout<<"dian.x is :"<<std::endl<<dian2.at<double>(0,0)<<std::endl;
                jieguo.x = dian2.at<double>(0,0);
                jieguo.y = dian2.at<double>(0,1);
                cv::circle(img,jieguo,5,cv::Scalar(0,0,255),-1);
                // std::cout<<"jieguo is :"<<std::endl<<jieguo<<std::endl;

                //test
                // std::cout << X(0,0) <<std::endl;

                

                F <<    1,  0.1,  0,  0,  0,  0,
                        0,  1,  0,  0,  0,  0,
                        0,  0,  1,  0.1,  0,  0,
                        0,  0,  0,  1,  0,  0,
                        0,  0,  0,  0,  1,  0.1,
                        0,  0,  0,  0,  0,  1;

                //test
                // F <<    1,  0,  0,  0,  0,  0,
                //         0,  1,  0,  0,  0,  0,
                //         0,  0,  1,  0,  0,  0,
                //         0,  0,  0,  1,  0,  0,
                //         0,  0,  0,  0,  1,  0,
                //         0,  0,  0,  0,  0,  1;
                // std::cout << F <<std::endl;   
                
     

                variable1 = pow(t,4) / 4;
                variable2 = pow(t,2);
                variable3 = pow(t,3) / 2;
                Q <<    variable1,  variable3,          0,          0,          0,          0,
                        variable3,  variable2,          0,          0,          0,          0,
                                0,          0,  variable1,  variable3,          0,          0,
                                0,          0,  variable3,  variable2,          0,          0,
                                0,          0,          0,          0,  variable1,  variable3,
                                0,          0,          0,          0,  variable3,  variable2;

                Q = Q * a;
                //test
                // std::cout << Q <<std::endl;   
                
                P = F * P * F.transpose() + Q;

                //test
                // std::cout << "P is : "<<std::endl<<P <<std::endl;   


                //更新方程
                //H
                H <<    1,  0,  0,  0,  0,  0,
                        0,  0,  1,  0,  0,  0,
                        0,  0,  0,  0,  1,  0,
                
                
                // std::cout<< H << std::endl;

                //(R矩阵中的值待估计）
                R <<    0,  0,  0,
                        0,  0,  0,
                        0,  0,  0;

                K = P * H.transpose() * (H * P * H.transpose() + R).inverse();
                //test
                // std::cout<<"K is : "<<std::endl<<K<<std::endl;

                
                X2 << tvec.at<double>(0,0),tvec.at<double>(1,0),tvec.at<double>(2,0);
                // X(0,0) = X(0,0) + K * (tvec.at<double>(0,0) - X(0,0));
                // X(0,2) = X(0,2) + K * (tvec.at<double>(0,1) - X(0,2));
                // X(0,4) = X(0,4) + K * (tvec.at<double>(0,2) - X(0,4));

                X = X + K * (X2 - H * X);
                //test
                // std::cout<<"X is : "<<std::endl<<X<<std::endl;


                P = (I - K * H) * P;
                //test
                // std::cout<<"P is : "<<std::endl<<P<<std::endl;

                



                X1(0,0) = tvec.at<double>(0,0);
                X1(1,0) = tvec.at<double>(1,0);
                X1(2,0) = tvec.at<double>(2,0);



                // //打印输出坐标
                // std::cout<<"zuobiao is : "<<std::endl;
                // std::cout<<X(0,0)<<std::endl;
                // std::cout<<X(2,0)<<std::endl;
                // std::cout<<X(4,0)<<std::endl;

                
                //画出滤波后的点
                // X3.at<double>(0,0) = X(0,0);
                // X3.at<double>(1,0) = X(2,0);
                // X3.at<double>(2,0) = X(4,0);
                //test
                std::cout<<"tvec is :"<<std::endl<<tvec<<std::endl;
                std::cout<<"X3 is :"<<std::endl<<X3<<std::endl;

                

                cv::projectPoints(dian3,rvec,X3,cameraMatrix,distCoeffs,dian2);
                //test
                cv::projectPoints(dian3,rvec,tvec,cameraMatrix,distCoeffs,dian22);
                jieguo1.x = dian22.at<double>(0,0);
                jieguo1.y = dian22.at<double>(0,1);
                cv::circle(img,jieguo1,10,cv::Scalar(255,0,0),-1);
                //test
                // std::cout<<"dian is :"<<std::endl<<dian2<<std::endl;
                // std::cout<<"dian.x is :"<<std::endl<<dian2.at<double>(0,0)<<std::endl;
                jieguo.x = dian2.at<double>(0,0);
                jieguo.y = dian2.at<double>(0,1);
                cv::circle(img,jieguo,5,cv::Scalar(0,0,255),-1);
                // std::cout<<"jieguo is :"<<std::endl<<jieguo<<std::endl;

            }

        
        // cv::imshow("GRAY",gray);
        cv::imshow("test",img);
        

        

        if(cv::waitKey(1) == 27)
        {
            break;
        }
    }
    cv::destroyAllWindows();
    c.close();
    return 0;

}

