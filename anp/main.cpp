#include <fstream>
#include <iostream>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <boost/filesystem.hpp>

int main(int argc, char** argv) {
    std::vector<std::string> imageFilenamesL;
    std::string inputDir = "/home/one/calib_data/test";
    std::string fileExtension = ".bmp";
    // std::cout << "argv[1]: " << std::stoi(argv[1]) << std::endl;
    std::string prefixL = argv[1];
    std::cout << "prefixL is " << prefixL << std::endl;
    if (!boost::filesystem::exists(inputDir) && !boost::filesystem::is_directory(inputDir))
    {
        std::cerr << "# ERROR: Cannot find input directory " << inputDir << "." << std::endl;
        return 1;
    }
    boost::filesystem::directory_iterator itr;
    for (boost::filesystem::directory_iterator itr(inputDir); itr != boost::filesystem::directory_iterator(); ++itr)
    {
        if (!boost::filesystem::is_regular_file(itr->status()))
            continue;

        std::string filename = itr->path().filename().string();

        // check if file extension matches
        if (filename.compare(filename.length() - fileExtension.length(), fileExtension.length(), fileExtension) != 0)
            continue;

        // check if prefix matches
        if (prefixL.empty() || (!prefixL.empty() && (filename.compare(0, prefixL.length(), prefixL) == 0)))
        {
            imageFilenamesL.push_back(itr->path().string());
        }
    }
    std::sort(imageFilenamesL.begin(), imageFilenamesL.end());
    cv::Mat imageL = cv::imread(imageFilenamesL.front(), -1);
    cv::Mat rImg;
    for (size_t i = 0; i < imageFilenamesL.size(); ++i)
    {
        imageL = cv::imread(imageFilenamesL.at(i), -1);
        cv::circle(imageL, cv::Point2f(960,540), 20, cv::Scalar(0, 255, 255), 4);
        cv::line(imageL, cv::Point2f(960,0), cv::Point2f(960,1080), cv::Scalar(255, 0, 0));
        cv::line(imageL, cv::Point2f(0,540), cv::Point2f(1920,540), cv::Scalar(255, 0, 0));
        cv::resize(imageL, rImg, cv::Size(imageL.cols * 0.5, imageL.rows * 0.5));
        cv::imshow(prefixL, rImg);
        char c = cv::waitKey(0);
        if( c == 27 || c == 'q' || c == 'Q' )
            break;   
    }    

    return 0;
}
