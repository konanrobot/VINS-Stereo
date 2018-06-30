#include "camodocal/camera_models/CameraFactory.h"

#include <boost/algorithm/string.hpp>


#include "camodocal/camera_models/CataCamera.h"
#include "camodocal/camera_models/EquidistantCamera.h"
#include "camodocal/camera_models/PinholeCamera.h"
#include "camodocal/camera_models/ScaramuzzaCamera.h"

#include "ceres/ceres.h"

namespace camodocal
{

boost::shared_ptr<CameraFactory> CameraFactory::m_instance;

CameraFactory::CameraFactory()
{

}

boost::shared_ptr<CameraFactory>
CameraFactory::instance(void)
{
    if (m_instance.get() == 0)
    {
        m_instance.reset(new CameraFactory);
    }

    return m_instance;
}

CameraPtr
CameraFactory::generateCamera(Camera::ModelType modelType,
                              const std::string& cameraName,
                              cv::Size imageSize) const
{
    switch (modelType)
    {
    case Camera::KANNALA_BRANDT:
    {
        EquidistantCameraPtr camera(new EquidistantCamera);

        EquidistantCamera::Parameters params = camera->getParameters();
        params.cameraName() = cameraName;
        params.imageWidth() = imageSize.width;
        params.imageHeight() = imageSize.height;
        camera->setParameters(params);
        return camera;
    }
    case Camera::PINHOLE:
    {
        PinholeCameraPtr camera(new PinholeCamera);

        PinholeCamera::Parameters params = camera->getParameters();
        params.cameraName() = cameraName;
        params.imageWidth() = imageSize.width;
        params.imageHeight() = imageSize.height;
        camera->setParameters(params);
        return camera;
    }
    case Camera::SCARAMUZZA:
    {
        OCAMCameraPtr camera(new OCAMCamera);

        OCAMCamera::Parameters params = camera->getParameters();
        params.cameraName() = cameraName;
        params.imageWidth() = imageSize.width;
        params.imageHeight() = imageSize.height;
        camera->setParameters(params);
        return camera;
    }
    case Camera::MEI:
    default:
    {
        CataCameraPtr camera(new CataCamera);

        CataCamera::Parameters params = camera->getParameters();
        params.cameraName() = cameraName;
        params.imageWidth() = imageSize.width;
        params.imageHeight() = imageSize.height;
        camera->setParameters(params);
        return camera;
    }
    }
}

CameraPtr
CameraFactory::generateCameraFromYamlFile(const std::string& filename)
{
    cv::FileStorage fs(filename, cv::FileStorage::READ);

    if (!fs.isOpened())
    {
        return CameraPtr();
    }

    Camera::ModelType modelType = Camera::MEI;
    if (!fs["model_type"].isNone())
    {
        std::string sModelType;
        fs["model_type"] >> sModelType;

        if (boost::iequals(sModelType, "kannala_brandt"))
        {
            modelType = Camera::KANNALA_BRANDT;
        }
        else if (boost::iequals(sModelType, "mei"))
        {
            modelType = Camera::MEI;
        }
        else if (boost::iequals(sModelType, "scaramuzza"))
        {
            modelType = Camera::SCARAMUZZA;
        }
        else if (boost::iequals(sModelType, "pinhole"))
        {
            modelType = Camera::PINHOLE;
        }
        else
        {
            std::cerr << "# ERROR: Unknown camera model: " << sModelType << std::endl;
            return CameraPtr();
        }
    }

    switch (modelType)
    {
    case Camera::KANNALA_BRANDT:
    {
        EquidistantCameraPtr camera(new EquidistantCamera);

        EquidistantCamera::Parameters params = camera->getParameters();
        params.readFromYamlFile(filename);
        camera->setParameters(params);
        return camera;
    }
    case Camera::PINHOLE:
    {
        PinholeCameraPtr camera(new PinholeCamera);

        PinholeCamera::Parameters params = camera->getParameters();
        params.readFromYamlFile(filename);
        camera->setParameters(params);
        return camera;
    }
    case Camera::SCARAMUZZA:
    {
        OCAMCameraPtr camera(new OCAMCamera);

        OCAMCamera::Parameters params = camera->getParameters();
        params.readFromYamlFile(filename);
        camera->setParameters(params);
        return camera;
    }
    case Camera::MEI:
    default:
    {
        CataCameraPtr camera(new CataCamera);

        CataCamera::Parameters params = camera->getParameters();
        params.readFromYamlFile(filename);
        camera->setParameters(params);
        return camera;
    }
    }

    return CameraPtr();
}

CameraPtr
CameraFactory::generateCameraFromValue(std::string sModelType, int nWidth, int 
nHeight, std::vector<double>vIntrinsics,  std::vector<double> vDistortionCoeffs)
{
    Camera::ModelType modelType = Camera::MEI;

    if (boost::iequals(sModelType, "kannala_brandt"))
    {
        modelType = Camera::KANNALA_BRANDT;
    }
    else if (boost::iequals(sModelType, "mei"))
    {
        modelType = Camera::MEI;
    }
    else if (boost::iequals(sModelType, "scaramuzza"))
    {
        modelType = Camera::SCARAMUZZA;
    }
    else if (boost::iequals(sModelType, "pinhole"))
    {
        modelType = Camera::PINHOLE;
    }
    else
    {
        std::cerr << "# ERROR: Unknown camera model: " << sModelType << std::endl;
        return CameraPtr();
    }

   std::vector<double> k = vIntrinsics;
   std::vector<double>d = vDistortionCoeffs;
           
    switch (modelType)
    {
    case Camera::KANNALA_BRANDT:
    {
        EquidistantCameraPtr camera(new EquidistantCamera);

       // pinhole-equaldistance
        EquidistantCamera::Parameters params("kannala_brandt", nWidth, nHeight, d[0], d[1], d[2], d[3], k[0], k[1], k[2], k[3]);
        camera->setParameters(params);
        return camera;
    }
    case Camera::PINHOLE:
    {
        PinholeCameraPtr camera(new PinholeCamera);

        // pinhole-radtan
        PinholeCamera::Parameters params("pinhole", nWidth, nHeight, d[0], d[1], d[2], d[3], k[0], k[1], k[2], k[3]);
        camera->setParameters(params);
        return camera;
    }
    case Camera::SCARAMUZZA:
    {
        OCAMCameraPtr camera(new OCAMCamera);

        // not support
        OCAMCamera::Parameters params = camera->getParameters();
        params.C() = k[0];
        params.D() = k[1];
        params.E() = k[2];
        std::cout<<"not supported:scaramuzza"<<std::endl;
        //OCAMCamera::Parameters ("scaramuzza", nWidth, nHeight, d[0], d[1], d[2], d[3], k[0], k[1], k[2], k[3]);
        camera->setParameters(params);
        return camera;
    }
    case Camera::MEI:
    default:
    {
        CataCameraPtr camera(new CataCamera);

        // mei
        CataCamera::Parameters  params("mei", nWidth, nHeight, k[0], d[0], d[1], d[2], d[3],  k[1], k[2], k[3], k[4]);
        camera->setParameters(params);
        return camera;
    }
    }

    return CameraPtr();
}


}

