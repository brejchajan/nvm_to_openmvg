#ifndef NVMLOADER_H
#define NVMLOADER_H
/**
* @Author: Jan Brejcha <janbrejcha>
* @Date:   11. 8. 2020
* @Email:  brejcha@adobe.com, ibrejcha@fit.vutbr.cz, brejchaja@gmail.com
*/

//stl headers
#include <string>
#include <vector>
#include <stdexcept>

//openMVG headers
#include "openMVG/sfm/sfm_data.hpp"
#include "openMVG/types.hpp"
#include "openMVG/cameras/Camera_Pinhole.hpp"
#include "openMVG/geometry/pose3.hpp"

//nvm util headers
#include "ioutil.h"
#include "DataInterface.h"

//OSG headers
#include "osgDB/ReadFile"
#include "osg/Image"

//Boost headers
#include "boost/filesystem.hpp"
#include "boost/algorithm/string.hpp"

using namespace openMVG;
using namespace openMVG::sfm;
using namespace openMVG::cameras;
using namespace std;

class NVMLoader {

public:
    static SfM_Data load(string filename)
    {
        vector<CameraT> cameras;
        vector<Point3D> points;
        vector<Point2D> measurements;
        vector<int> ptidx;
        vector<int> camidx;
        vector<string> names;
        vector<int> ptc;

        SfM_Data data;

        namespace fs = boost::filesystem;
        const fs::path p(filename);
        fs::path img_path = p.parent_path();

        LoadModelFile(filename.c_str(), cameras, points, measurements,
                      ptidx, camidx, names, ptc);

        int cidx = 0;
        IndexT viewId = 0;
        IndexT intrId = 0;
        IndexT poseId = 0;
        std::vector<int>::iterator it = ptidx.begin();
        std::cout << "before iterating cameras..." << std::endl;
        for (CameraT &c : cameras)
        {
            //TODO get correct image width and height
            fs::path cimg_path = img_path / names[cidx];
            osg::ref_ptr<osg::Image> photoImg = osgDB::readImageFile(cimg_path.string());
            if (!photoImg.valid())
            {
                throw std::runtime_error("Unable to load image file: " + cimg_path.string());
            }
            unsigned int width = photoImg->s();
            unsigned int height = photoImg->t();
            std::cout << "Loaded camera info: " << names[cidx] << std::endl;

            float cam_center[3];
            /*THIS IS CAMERA CENTER IN WORLD COORDS, NOT PRINCIPAL POINT!
             * c.GetCameraCenter(cam_center);
            cam_center[0] /= cam_center[2];
            cam_center[1] /= cam_center[2];*/

            std::cout << "w: " << width << ", h: " << height << ", focal: " << c.GetFocalLength() << std::endl;
            shared_ptr<IntrinsicBase> intrinsic(new Pinhole_Intrinsic(width, height, c.GetFocalLength(), 0.5 * width, 0.5 * height)); //for now
            data.intrinsics.insert(std::make_pair(intrId, intrinsic));

            float cf[3];
            double r[9];
            c.GetCameraCenter(cf);
            c.GetMatrixRotation(r);
            Mat3 rot = Eigen::Map<Mat3>(&r[0], 3, 3);
            rot.transposeInPlace();
            openMVG::Vec3 center(cf[0], cf[1], cf[2]);
            Pose3 pose(rot, center);
            data.poses.insert(std::make_pair(poseId, pose));
            sfm::View *v = new sfm::View(names[cidx], viewId, intrId,
                                         poseId, width, height);
            std::shared_ptr<sfm::View> pv(v);
            data.views.insert(std::make_pair(viewId, pv));
            ++viewId;
            ++intrId;
            ++poseId;
            ++cidx;
        }

        IndexT trackId = 0;
        IndexT id_feat = 0;
        for (Point3D &p3D : points)
        {
            float x, y, z;
            p3D.GetPoint(x, y, z);
            Landmark l;
            l.X = Eigen::Vector3d(x, y, z);

            /*std::vector<int>::iterator it = std::find(ptidx.begin(),
                                                      ptidx.end(),
                                                      trackId);*/
            while (*it == trackId)
            {
                //get the 2D points
                int midx = std::distance(ptidx.begin(), it);
                IndexT viewId = (IndexT)camidx[midx];
                Point2D m = measurements[midx];
                openMVG::Vec2 p2d(m.x, m.y);
                Observation obs(p2d, id_feat);
                l.obs.insert(std::make_pair(viewId, obs));
                ++id_feat;
                ++it;
            }

            data.structure.insert(std::make_pair(trackId, l));
            ++trackId;
        }
        std::cout << "Number of observations: " << id_feat + 1 << std::endl;
        return data;
    }

};


#endif // NVMLOADER_H
