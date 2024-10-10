#include <iostream>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include "LineDetection3D.h"
#include "nanoflann.hpp"
#include "utils.h"
#include "Timer.h"

using namespace cv;
using namespace std;
using namespace nanoflann;

void readDataFromPLY(std::string filepath, PointCloud<double> &cloud) {
    cloud.pts.reserve(10000000); // Reserve memory for points
    std::ifstream plyFile(filepath);

    if (!plyFile.is_open()) {
        std::cerr << "Error opening PLY file: " << filepath << std::endl;
        return;
    }

    std::string line;
    bool headerEnded = false;
    int vertexCount = 0;
    int lineCount = 0;

    // Read the header and find the number of vertices (points)
    while (std::getline(plyFile, line)) {
        if (line.substr(0, 14) == "element vertex") {
            // Parse the number of vertices
            std::istringstream iss(line);
            std::string element, vertex;
            iss >> element >> vertex >> vertexCount;
        }
        if (line == "end_header") {
            headerEnded = true;
            break;
        }
    }

    if (!headerEnded) {
        std::cerr << "Invalid PLY file: Header not found!" << std::endl;
        return;
    }

    // Read the point data (x, y, z, intensity, r, g, b)
    double x, y, z, intensity;
    int r, g, b;
    
    while (lineCount < vertexCount && std::getline(plyFile, line)) {
        std::istringstream iss(line);
        if (iss >> x >> y >> z >> intensity >> r >> g >> b) {
            // Store the point data in the PointCloud structure
            cloud.pts.push_back(PointCloud<double>::PtData(x, y, z));
        }
        lineCount++;
    }

    plyFile.close();
    std::cout << "Total number of points: " << cloud.pts.size() << std::endl;
}

void readDataFromFile( std::string filepath, PointCloud<double> &cloud )
{
	cloud.pts.reserve(10000000);
	cout<<"Reading data ..."<<endl;

	// 1. read in point data
	std::ifstream ptReader( filepath );
	std::vector<cv::Point3d> lidarPoints;
	double x = 0, y = 0, z = 0, color = 0;
	double nx, ny, nz;
	int a = 0, b = 0, c = 0; 
	int labelIdx = 0;
	int count = 0;
	int countTotal = 0;
	if( ptReader.is_open() )
	{
		while ( !ptReader.eof() ) 
		{
			//ptReader >> x >> y >> z >> a >> b >> c >> labelIdx;
			//ptReader >> x >> y >> z >> a >> b >> c >> color;
			//ptReader >> x >> y >> z >> color >> a >> b >> c;
			//ptReader >> x >> y >> z >> a >> b >> c ;
			ptReader >> x >> y >> z;
			//ptReader >> x >> y >> z >> color;
			//ptReader >> x >> y >> z >> nx >> ny >> nz;

			cloud.pts.push_back(PointCloud<double>::PtData(x,y,z));

		}
		ptReader.close();
	}

	std::cout << "Total num of points: " << cloud.pts.size() << "\n";
}

void readDataFromColmapFile(std::string filepath, PointCloud<double> &cloud) 
{
    cloud.pts.reserve(10000000);
    cout << "Reading data..." << endl;

    // Open the file
    std::ifstream ptReader(filepath);
    double x = 0, y = 0, z = 0;
    int r = 0, g = 0, b = 0;
    int point3D_ID;
    double error;
    string line;
    
    if (ptReader.is_open()) {
        while (getline(ptReader, line)) {
            // Skip lines starting with '#' (comments)
            if (line[0] == '#') continue;

            // Read the data from each line (POINT3D_ID X Y Z R G B ERROR)
            std::istringstream iss(line);
            iss >> point3D_ID >> x >> y >> z >> r >> g >> b >> error;

            // Store the 3D points in the PointCloud structure
            cloud.pts.push_back(PointCloud<double>::PtData(x, y, z));
        }
        ptReader.close();
    }

    std::cout << "Total number of points: " << cloud.pts.size() << "\n";
}

void writeOutPlanes( string filePath, std::vector<PLANE> &planes, double scale )
{
	// write out bounding polygon result
	string fileEdgePoints = filePath + "planes.txt";
	FILE *fp2 = fopen( fileEdgePoints.c_str(), "w");
	for (int p=0; p<planes.size(); ++p)
	{
		int R = rand()%255;
		int G = rand()%255;
		int B = rand()%255;

		for (int i=0; i<planes[p].lines3d.size(); ++i)
		{
			for (int j=0; j<planes[p].lines3d[i].size(); ++j)
			{
				cv::Point3d dev = planes[p].lines3d[i][j][1] - planes[p].lines3d[i][j][0];
				double L = sqrt(dev.x*dev.x + dev.y*dev.y + dev.z*dev.z);
				int k = L/(scale/10);

				double x = planes[p].lines3d[i][j][0].x, y = planes[p].lines3d[i][j][0].y, z = planes[p].lines3d[i][j][0].z;
				double dx = dev.x/k, dy = dev.y/k, dz = dev.z/k;
				for ( int j=0; j<k; ++j)
				{
					x += dx;
					y += dy;
					z += dz;

					fprintf( fp2, "%.6lf   %.6lf   %.6lf    ", x, y, z );
					fprintf( fp2, "%d   %d   %d   %d\n", R, G, B, p );
				}
			}
		}
	}
	fclose( fp2 );
}

void writeOutPlanesObj(string filePath, std::vector<PLANE> &planes,
                       double scale) {
  string fileEdgePoints = filePath + "planes.obj";
  std::ofstream file;
  file.open(fileEdgePoints.c_str());

  int cnt = 0;
  for (int p = 0; p < planes.size(); ++p) {
    for (int i = 0; i < planes[p].lines3d.size(); ++i) {
      for (int j = 0; j < planes[p].lines3d[i].size(); ++j) {
        cnt++;
        auto &a = planes[p].lines3d[i][j][0], &b = planes[p].lines3d[i][j][1];
        file << "v " << a.x << ' ' << a.y << ' ' << a.z << std::endl
             << "v " << b.x << ' ' << b.y << ' ' << b.z << std::endl;
      }
    }
  }
  for (int p = 0; p < cnt; ++p) {
    file << "l " << p * 2 + 1 << ' ' << p * 2 + 2 << std::endl;
  }
  file.close();
}

void writeOutLines( string filePath, std::vector<std::vector<cv::Point3d> > &lines, double scale )
{
	// write out bounding polygon result
	string fileEdgePoints = filePath + "lines.txt";
	FILE *fp2 = fopen( fileEdgePoints.c_str(), "w");
	for (int p=0; p<lines.size(); ++p)
	{
		int R = rand()%255;
		int G = rand()%255;
		int B = rand()%255;

		cv::Point3d dev = lines[p][1] - lines[p][0];
		double L = sqrt(dev.x*dev.x + dev.y*dev.y + dev.z*dev.z);
		int k = L/(scale/10);

		double x = lines[p][0].x, y = lines[p][0].y, z = lines[p][0].z;
		double dx = dev.x/k, dy = dev.y/k, dz = dev.z/k;
		for ( int j=0; j<k; ++j)
		{
			x += dx;
			y += dy;
			z += dz;

			fprintf( fp2, "%.6lf   %.6lf   %.6lf    ", x, y, z );
			fprintf( fp2, "%d   %d   %d   %d\n", R, G, B, p );
		}
	}
	fclose( fp2 );
}

void writeOutLinesObj(string filePath,
                      std::vector<std::vector<cv::Point3d>> &lines,
                      double scale) {
  string fileEdgePoints = filePath + "lines.obj";
  std::ofstream file;
  file.open(fileEdgePoints.c_str());

  for (int p = 0; p < lines.size(); ++p) {
    file << "v " << lines[p][0].x << ' ' << lines[p][0].y << ' '
         << lines[p][0].z << std::endl
         << "v " << lines[p][1].x << ' ' << lines[p][1].y << ' '
         << lines[p][1].z << std::endl;
  }

  for (int p = 0; p < lines.size(); ++p) {
    file << "l " << p * 2 + 1 << ' ' << p * 2 + 2 << std::endl;
  }
  file.close();
}

int main(int argc, char *argv[]) 
{
    // Check if proper arguments are passed
    if (argc < 5) {
        std::cerr << "Usage: " << argv[0] << " <input_file> <output_directory> <input_format: txt/ply/colmap> <output_format: obj/txt>" << std::endl;
        return 1;
    }

    // Read command-line arguments
    std::string fileData = argv[1];  // Input file path
    std::string fileOut  = argv[2];  // Output directory path
    std::string inputFormat = argv[3];  // Input format: txt/ply/colmap
    std::string outputFormat = argv[4]; // Output format: obj/txt

    // Read in data
    PointCloud<double> pointData;

    // Choose input format
    if (inputFormat == "txt") {
        readDataFromFile(fileData, pointData);  // Read .txt data
    } else if (inputFormat == "ply") {
        readDataFromPLY(fileData, pointData);   // Read .ply data
    } else if (inputFormat == "colmap") {
        readDataFromColmapFile(fileData, pointData);  // Read .colmap data
    } else {
        std::cerr << "Invalid input format: " << inputFormat << ". Choose from txt/ply/colmap." << std::endl;
        return 1;
    }

    // Process the data
    int k = 20;
    LineDetection3D detector;
    std::vector<PLANE> planes;
    std::vector<std::vector<cv::Point3d>> lines;
    std::vector<double> ts;
    detector.run(pointData, k, planes, lines, ts);
    
    std::cout << "Lines number: " << lines.size() << std::endl;
    std::cout << "Planes number: " << planes.size() << std::endl;

    // Choose output format
    if (outputFormat == "obj") {
        writeOutPlanesObj(fileOut, planes, detector.scale);  // Write planes in .obj format
        writeOutLinesObj(fileOut, lines, detector.scale);    // Write lines in .obj format
    } else if (outputFormat == "txt") {
        writeOutPlanes(fileOut, planes, detector.scale);     // Write planes in .txt format
        writeOutLines(fileOut, lines, detector.scale);       // Write lines in .txt format
    } else {
        std::cerr << "Invalid output format: " << outputFormat << ". Choose from obj/txt." << std::endl;
        return 1;
    }

    return 0;
}
