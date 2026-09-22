#include "pool_viewer/depth_noise.hpp"
#include <cassert>
#include <iostream>

std::pair<double,double> statistics(const cv::Mat &depth,double truth) {
  double sum=0,squared=0;int count=0;
  for(int y=0;y<depth.rows;++y)for(int x=0;x<depth.cols;++x){double v=depth.at<float>(y,x);if(std::isfinite(v)){v-=truth;sum+=v;squared+=v*v;++count;}}
  return {sum/count,std::sqrt(squared/count-std::pow(sum/count,2))};
}
int main() {
  pool::DepthNoise model;model.enabled=false;std::mt19937 random(7);
  cv::Mat image(3,3,CV_32FC1,cv::Scalar(2));image.at<float>(0,0)=NAN;image.at<float>(0,1)=.01;image.at<float>(0,2)=20;
  model.apply(image,random);assert(std::isnan(image.at<float>(0,0)));assert(std::isnan(image.at<float>(0,1)));assert(std::isnan(image.at<float>(0,2)));assert(image.at<float>(1,1)==2);
  model.enabled=true;model.dropout=model.rangeDropout=model.edgeDropout=model.outliers=0;model.correlation=0;
  for(float z:{1.f,4.f}) {
    image=cv::Mat(400,400,CV_32FC1,cv::Scalar(z));model.apply(image,random);
    auto [mean,sigma]=statistics(image,z);
    assert(std::abs(mean)<model.sigma(z)*.015);assert(std::abs(sigma/model.sigma(z)-1)<.015);
  }
  model.correlation=.8;image=cv::Mat(400,400,CV_32FC1,cv::Scalar(3));model.apply(image,random);
  assert(std::abs(statistics(image,3).second/model.sigma(3)-1)<.05);
  model.dropout=1;model.apply(image,random);assert(std::isnan(image.at<float>(200,200)));
  model.dropout=0;model.maxRange=model.minRange;bool rejected=false;
  try{model.validate();}catch(const std::invalid_argument&){rejected=true;}assert(rejected);
  // Batched sampling keeps probabilities and seeded results consistent without
  // consuming OpenCV's process/thread-global random state.
  model=pool::DepthNoise{};model.dropout=.2;model.rangeDropout=model.edgeDropout=model.outliers=0;
  model.correlation=0;
  cv::Mat first(400,400,CV_32FC1,cv::Scalar(3)),second=first.clone();
  std::mt19937 seedA(42),seedB(42);
  const auto globalState=cv::theRNG().state;
  model.apply(first,seedA);model.apply(second,seedB);
  assert(cv::theRNG().state==globalState);
  int invalid=0;
  for(int y=0;y<first.rows;++y)for(int x=0;x<first.cols;++x){
    const float a=first.at<float>(y,x),b=second.at<float>(y,x);
    if(std::isnan(a)){++invalid;assert(std::isnan(b));}else assert(a==b);
  }
  assert(std::abs(double(invalid)/first.total()-.2)<.005);
  // Outlier draws use their own probability channel and keep the expected
  // uniform +/-25% depth distribution when Gaussian noise is disabled.
  model.dropout=0;model.outliers=1;model.baseSigma=model.rangeSigma=0;
  first=cv::Mat(400,400,CV_32FC1,cv::Scalar(3));model.apply(first,seedA);
  const auto outlierStats=statistics(first,3);
  assert(std::abs(outlierStats.first)<.01);
  assert(std::abs(outlierStats.second-.75/std::sqrt(3.))<.01);
  std::cout<<"Depth clipping, ideal mode, range variance, correlated variance and dropout passed\n";
}
