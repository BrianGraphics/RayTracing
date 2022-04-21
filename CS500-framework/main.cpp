///////////////////////////////////////////////////////////////////////
// Provides the framework a raytracer.
//
// Gary Herron
//
// Copyright 2012 DigiPen Institute of Technology
////////////////////////////////////////////////////////////////////////

#include <fstream>
#include <sstream>
#include <vector>
#include <string.h>
#include <ctime>

#ifdef _WIN32
    // Includes for Windows
    #include <windows.h>
    #include <cstdlib>
    #include <limits>
    #include <crtdbg.h>
#else
    // Includes for Linux
    #include <stdlib.h>
    #include <time.h> 
#endif

#include "geom.h"
#include "raytrace.h"

// Read a scene file by parsing each line as a command and calling
// scene->Command(...) with the results.
void ReadScene(const std::string inName, Scene* scene)
{
    std::ifstream input(inName.c_str());
    if (input.fail()) {
        std::cerr << "File not found: "  << inName << std::endl;
        fflush(stderr);
        exit(-1); }

    // For each line in file
    for (std::string line; getline(input, line); ) {
        std::vector<std::string> strings;
        std::vector<float> floats;
        
        // Parse as parallel lists of strings and floats
        std::stringstream lineStream(line);
        for (std::string s; lineStream >> s; ) { // Parses space-separated strings until EOL
            float f;
            //std::stringstream(s) >> f; // Parses an initial float into f, or zero if illegal
            if (!(std::stringstream(s) >> f)) f = nan(""); // An alternate that produced NANs
            floats.push_back(f);
            strings.push_back(s); }

        if (strings.size() == 0) continue; // Skip blanks lines
        if (strings[0][0] == '#') continue; // Skip comment lines
        
        // Pass the line's data to Command(...)
        scene->Command(strings, floats);
    }

    input.close();
}

// Write the image as a HDR(RGBE) image.  
#include "rgbe.h"
void WriteHdrImage(const std::string outName, const int width, const int height, Color* image)
{
    // Turn image from a 2D-bottom-up array of Vector3D to an top-down-array of floats
    float* data = new float[width*height*3];
    float* dp = data;
    for (int y=height-1;  y>=0;  --y) {
        for (int x=0;  x<width;  ++x) {
            Color pixel = image[y*width + x];
            *dp++ = pixel[0];
            *dp++ = pixel[1];
            *dp++ = pixel[2]; } }

    // Write image to file in HDR (a.k.a RADIANCE) format
    rgbe_header_info info;
    char errbuf[100] = {0};

    FILE* fp  =  fopen(outName.c_str(), "wb");
    info.valid = false;
    int r = RGBE_WriteHeader(fp, width, height, &info, errbuf);
    if (r != RGBE_RETURN_SUCCESS)
        printf("error: %s\n", errbuf);

    r = RGBE_WritePixels_RLE(fp, data, width,  height, errbuf);
    if (r != RGBE_RETURN_SUCCESS)
        printf("error: %s\n", errbuf);
    fclose(fp);
    
    delete data;
}

void ReadHdrImage(const std::string inName, Scene* scene)
{
    // Turn image from a 2D-bottom-up array of Vector3D to an top-down-array of floats
    int width  = 0;
    int height = 0;

    // Write image to file in HDR (a.k.a RADIANCE) format
    rgbe_header_info info;
    char errbuf[100] = { 0 };

    FILE* fp = fopen(inName.c_str(), "rb");
    info.valid = false;
    int r = RGBE_ReadHeader(fp, &width, &height, &info, errbuf);
    if (r != RGBE_RETURN_SUCCESS)
        printf("error: %s\n", errbuf);
    
    float* data = new float[width * height * 3];
    scene->sky = new Sky();
    scene->sky->width  = width;
    scene->sky->height = height;
    scene->sky->hdr = new Color[width * height];
    for (int y = 0; y < height; y++)
        for (int x = 0; x < width; x++)
            scene->sky->hdr[y * width + x] = Color(0, 0, 0);

    scene->sky->radius = 1000.0f;

    r = RGBE_ReadPixels_RLE(fp, data, width, height, errbuf);
    if (r != RGBE_RETURN_SUCCESS)
        printf("error: %s\n", errbuf);
    fclose(fp);

    float* dp = data;
    for (int y = height - 1; y >= 0; --y) {
        for (int x = 0; x < width; ++x) {
            scene->sky->hdr[y * width + x][0] = *dp++;
            scene->sky->hdr[y * width + x][1] = *dp++;
            scene->sky->hdr[y * width + x][2] = *dp++;
        }
    }

    delete data;
}

////////////////////////////////////////////////////////////////////////
int main(int argc, char** argv)
{
    Scene* scene = new Scene();

    // Read the command line argument
    std::string inName =  (argc > 1) ? argv[1] : "testscene.scn";
    std::string hdrName = inName;

    hdrName.replace(hdrName.size()-3, hdrName.size(), "hdr");

    // Read the scene, calling scene.Command for each line.
    ReadHdrImage("Mono_Lake_B_Ref.hdr", scene);
    scene->sky->PreProcessing();
    ReadScene(inName, scene);    

    // Allocate and clear an image array
    Color *image =  new Color[scene->width*scene->height];    
    for (int y=0;  y<scene->height;  y++)
        for (int x=0;  x<scene->width;  x++)
            image[y*scene->width + x] = Color(0,0,0);

    // RayTrace the image
    scene->TraceImage(image, 512);
    //image = scene->sky->hdr;

    // Write the image
    WriteHdrImage(hdrName, scene->width, scene->height, image);

}
